
#include <Wire.h>
#include <math.h>
#include <string.h>

// =================== CONFIGURACIÓN GENERAL ===================

static const uint8_t NUM_MOTORES = 5;

// Dirección I2C con la que el Arduino actuará como esclavo frente al ESP32
static const uint8_t ARDUINO_I2C_ADDRESS = 0x10;

// Dirección I2C por defecto del multiplexor PCA9548A/TCA9548A
static const uint8_t MUX_ADDRESS = 0x70;

// Dirección I2C del codificador magnético AS5600
static const uint8_t AS5600_ADDRESS = 0x36;

// Pines PWM (RPWM y LPWM) para cada uno de los 5 drivers BTS7960 conectados al Arduino.
// Ajusta los valores de estos arreglos de acuerdo a tu cableado real.
static const uint8_t RPWM_PINS[NUM_MOTORES] = {5, 4, 9, 10, 0};
static const uint8_t LPWM_PINS[NUM_MOTORES] = {6, 11, 13, 12, 0};

// Pines de habilitación (R_EN y L_EN) para cada BTS7960.
// Usa -1 si dejas ese pin permanentemente en HIGH (por ejemplo, cableado a 5 V).
static const int8_t REN_PINS[NUM_MOTORES] = {7, -1, -1, -1, -1};
static const int8_t LEN_PINS[NUM_MOTORES] = {8, -1, -1, -1, -1};

// =================== PARÁMETROS DE CONTROL ===================

static const float ANGULO_MIN = 0.0f;
static const float ANGULO_MAX = 360.0f;
static const float TOLERANCIA_GRADOS = 4.0f;    // error permitido
static const float ERROR_APLICA_PWM_MIN = 10.0f; // por encima de este error se usa PWM_MIN
static const float ERROR_REPETICION_OBJETIVO = 1.0f; // margen para reintentar objetivo automáticamente
static const uint8_t PWM_MIN = 60;              // velocidad mínima para vencer fricción
static const uint8_t PWM_MIN_CERCANIA = 40;     // PWM mínimo cuando estamos cerca del objetivo
static const uint8_t PWM_MAX = 255;             // velocidad máxima
static const unsigned long TIEMPO_MAX_SIN_MOVIMIENTO_MS = 1000; // tiempo máximo sin detectar avance
static const float MIN_VARIACION_ANGULO_ATASCO = 0.5f;          // cambio mínimo para considerar movimiento

static const uint8_t PWM_CALIBRACION = 70;                       // velocidad mínima para buscar topes
static const float CALIB_UMBRAL_MOVIMIENTO = 0.5f;               // cambio mínimo para considerar avance en calibración
static const unsigned long CALIB_TIEMPO_SIN_CAMBIO_MS = 1000;    // tiempo sin cambio para asumir tope
static const unsigned long CALIB_TIEMPO_MAX_BUSQUEDA_MS = 15000; // límite de búsqueda por dirección
static const float CALIB_HOLGURA_RETORNO = 2.0f;                 // grados para separarse del tope tras hallarlo

// Ganancias PID. Ajusta según la respuesta mecánica real.
static const float GANANCIA_KP = 0.8f;
static const float GANANCIA_KI = 0.23f;
static const float GANANCIA_KD = 0.45f;

// Límite del término integral para evitar "wind-up".
static const float LIMITE_INTEGRAL = 110.0f;

// Coeficiente de filtrado exponencial para la derivada (0-1). Valores altos = más filtrado.
static const float FILTRO_DERIVADA = 1.0f;
static const unsigned long CONTROL_INTERVAL_MS = 30;
static const unsigned long TIEMPO_MAX_MOV_MS = 3000; // tiempo máximo por movimiento

// Comandos I2C aceptados desde el ESP32
static const uint8_t COMANDO_I2C_OBJETIVO = 0x01;

// Longitud del paquete de estado entregado al ESP32
static const size_t ESTADO_FLOTES_POR_MOTOR = 4;
static const size_t ESTADO_BUFFER_LENGTH = 1 + NUM_MOTORES * (1 + ESTADO_FLOTES_POR_MOTOR * sizeof(float));

// =================== DECLARACIÓN DE FUNCIONES ===================

void seleccionarCanalMux(uint8_t canal);
bool leerAS5600Raw(uint8_t canal, uint16_t &valor);
float leerAnguloGrados(uint8_t canal);
void detenerMotor(uint8_t motor);
void fijarMotor(uint8_t motor, int16_t pwm);
bool leerAnguloAcumulado(uint8_t motor, float &anguloAcumulado);
void inicializarSeguimientoAngulo(uint8_t motor, uint16_t lecturaRaw);
bool motorTienePWMPins(uint8_t motor);
bool moverMotorAAngulo(uint8_t motor, float objetivo, bool reiniciarControl = true, bool verbose = true);
bool buscarLimiteMotor(uint8_t motor, int16_t pwm, float &anguloLimite);
bool calibrarMotor(uint8_t motor);
float obtenerAnguloRelativo(uint8_t motor, float anguloAbsoluto);
float convertirObjetivoRelativoAAbsoluto(uint8_t motor, float objetivoRelativo, bool &dentroRango);
void procesarComandosSerial();
void imprimirAnguloMotor(uint8_t motor);
void reportarAngulos();
bool detectarEncoder(uint8_t canal);
void mantenerObjetivosActivos();
void procesarComandosI2C();
void actualizarBufferEstado();
void onI2CReceive(int bytesRecibidos);
void onI2CRequest();

// =================== ESTADO DE ENCÓDERS DETECTADOS ===================

static bool encoderDetectado[NUM_MOTORES] = {false};
static float ultimoErrorMotor[NUM_MOTORES] = {0.0f};
static float integralErrorMotor[NUM_MOTORES] = {0.0f};
static float derivadaFiltradaMotor[NUM_MOTORES] = {0.0f};
static float anguloAcumuladoMotor[NUM_MOTORES] = {0.0f};
static float ultimoAnguloMedidoMotor[NUM_MOTORES] = {0.0f};
static uint16_t ultimoValorRawMotor[NUM_MOTORES] = {0};
static long contadorVueltasMotor[NUM_MOTORES] = {0};
static long ticksAcumuladosMotor[NUM_MOTORES] = {0};
static bool seguimientoInicializado[NUM_MOTORES] = {false};
static bool objetivoValidoMotor[NUM_MOTORES] = {false};
static float objetivoMotor[NUM_MOTORES] = {0.0f};
static unsigned long ultimaCorreccionMotor[NUM_MOTORES] = {0};
static unsigned long ultimoCambioEncoderMotor[NUM_MOTORES] = {0};
static float ultimoAnguloMovimientoMotor[NUM_MOTORES] = {0.0f};
static bool motorAtascado[NUM_MOTORES] = {false};
static float offsetAnguloMotor[NUM_MOTORES] = {0.0f};
static float rangoAnguloMotor[NUM_MOTORES] = {0.0f};
static float progresoCalibracionMotor[NUM_MOTORES] = {0.0f};

enum CalibracionEstado : uint8_t
{
  CALIB_NO_INICIADA = 0,
  CALIB_BUSCANDO_MIN = 1,
  CALIB_BUSCANDO_MAX = 2,
  CALIB_COMPLETA = 3,
  CALIB_ERROR = 4
};

static CalibracionEstado estadoCalibracionMotor[NUM_MOTORES] = {CALIB_NO_INICIADA};

static volatile bool comandoI2CPendiente = false;
static volatile uint8_t comandoI2CMotor = 0;
static volatile float comandoI2CAngulo = 0.0f;
static volatile bool comandoI2CReinicio = true;

static uint8_t estadoI2CBuffer[ESTADO_BUFFER_LENGTH] = {0};

// =================== SETUP ===================

void setup()
{
  Serial.begin(115200);
  while (!Serial) { /* espera a que se abra la consola */ }

  Wire.begin(ARDUINO_I2C_ADDRESS);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  Wire.setClock(400000UL);

  for (uint8_t i = 0; i < NUM_MOTORES; ++i)
  {
    if (REN_PINS[i] > 0)
    {
      uint8_t pin = static_cast<uint8_t>(REN_PINS[i]);
      pinMode(pin, OUTPUT);
      digitalWrite(pin, HIGH);
    }
    if (LEN_PINS[i] > 0)
    {
      uint8_t pin = static_cast<uint8_t>(LEN_PINS[i]);
      pinMode(pin, OUTPUT);
      digitalWrite(pin, HIGH);
    }
    if (motorTienePWMPins(i))
    {
      pinMode(RPWM_PINS[i], OUTPUT);
      pinMode(LPWM_PINS[i], OUTPUT);
      detenerMotor(i);
    }
  }

  Serial.println(F("Escaneando encoders AS5600..."));
  for (uint8_t i = 0; i < NUM_MOTORES; ++i)
  {
    encoderDetectado[i] = detectarEncoder(i);
    Serial.print(F("Motor "));
    Serial.print(i + 1);
    if (encoderDetectado[i])
    {
      Serial.println(F(": encoder detectado."));
    }
    else
    {
      Serial.println(F(": encoder no detectado."));
    }
  }

  for (uint8_t i = 0; i < NUM_MOTORES; ++i)
  {
    if (encoderDetectado[i])
    {
      uint16_t lecturaRaw = 0;
      if (leerAS5600Raw(i, lecturaRaw))
      {
        inicializarSeguimientoAngulo(i, lecturaRaw);
      }
      else
      {
        Serial.print(F("No se pudo leer el encoder del motor "));
        Serial.print(i + 1);
        Serial.println(F(" durante la inicializacion."));
      }
      ultimoErrorMotor[i] = 0.0f;
      offsetAnguloMotor[i] = anguloAcumuladoMotor[i];
      rangoAnguloMotor[i] = 0.0f;
      progresoCalibracionMotor[i] = 0.0f;
      estadoCalibracionMotor[i] = CALIB_NO_INICIADA;
      if (motorTienePWMPins(i))
      {
        Serial.print(F("Calibrando limites del motor "));
        Serial.print(i + 1);
        Serial.println(F("..."));
        bool calibrado = calibrarMotor(i);
        if (calibrado)
        {
          Serial.print(F("Motor "));
          Serial.print(i + 1);
          Serial.print(F(" calibrado. Rango 0 - "));
          Serial.print(rangoAnguloMotor[i], 2);
          Serial.println(F(" grados."));
        }
        else
        {
          Serial.print(F("Fallo la calibracion del motor "));
          Serial.print(i + 1);
          Serial.println(F(". Revise encoder o topes mecanicos."));
        }
      }
      else
      {
        Serial.print(F("Motor "));
        Serial.print(i + 1);
        Serial.println(F(" detectado sin pines PWM configurados: omitiendo calibracion."));
      }
    }
  }

  Serial.print(F("Sistema listo. Escriba: <motor 1-"));
  Serial.print(NUM_MOTORES);
  Serial.println(F("> <angulo objetivo>. Puede usar valores mayores a 360 o negativos."));

  actualizarBufferEstado();
}

// =================== LOOP PRINCIPAL ===================

void loop()
{
  procesarComandosSerial();
  procesarComandosI2C();
  mantenerObjetivosActivos();
  actualizarBufferEstado();
}

// =================== IMPLEMENTACIONES ===================

void procesarComandosSerial()
{
  if (!Serial.available())
  {
    return;
  }

  String linea = Serial.readStringUntil('\n');
  linea.trim();

  if (linea.length() == 0)
  {
    return;
  }

  if (linea.equalsIgnoreCase("estado") ||
      linea.equalsIgnoreCase("angulos") ||
      linea.equalsIgnoreCase("status"))
  {
    reportarAngulos();
    return;
  }

  int separador = linea.indexOf(' ');
  if (separador < 0)
  {
    bool esNumero = true;
    for (uint16_t i = 0; i < linea.length(); ++i)
    {
      if (!isDigit(linea.charAt(i)))
      {
        esNumero = false;
        break;
      }
    }

    if (esNumero)
    {
      int motorConsulta = linea.toInt();
      if (motorConsulta < 1 || motorConsulta > NUM_MOTORES)
      {
        Serial.print(F("Numero de motor fuera de rango (1-"));
        Serial.print(NUM_MOTORES);
        Serial.println(F(")."));
      }
      else
      {
        imprimirAnguloMotor(static_cast<uint8_t>(motorConsulta - 1));
      }
      return;
    }

    Serial.println(F("Formato invalido. Use: <motor> <angulo> o escriba 'estado'."));
    return;
  }

  int motor = linea.substring(0, separador).toInt();
  float anguloObjetivo = linea.substring(separador + 1).toFloat();

  if (motor < 1 || motor > NUM_MOTORES)
  {
    Serial.print(F("Numero de motor fuera de rango (1-"));
    Serial.print(NUM_MOTORES);
    Serial.println(F(")."));
    return;
  }

  if (!encoderDetectado[motor - 1])
  {
    Serial.println(F("Encoder no detectado para ese motor. Movimiento cancelado."));
    return;
  }

  uint8_t motorIndex = static_cast<uint8_t>(motor - 1);
  if (estadoCalibracionMotor[motorIndex] != CALIB_COMPLETA)
  {
    Serial.println(F("El motor aun no esta calibrado. Espere a que finalice la busqueda de topes."));
    return;
  }

  bool dentroRango = true;
  float objetivoAbsoluto = convertirObjetivoRelativoAAbsoluto(motorIndex, anguloObjetivo, dentroRango);
  if (!dentroRango)
  {
    Serial.print(F("Objetivo fuera de rango. Limites: 0 - "));
    Serial.print(rangoAnguloMotor[motorIndex], 2);
    Serial.println(F(" grados."));
    return;
  }

  bool reiniciarControl = true;
  if (objetivoValidoMotor[motorIndex])
  {
    if (fabs(objetivoMotor[motorIndex] - objetivoAbsoluto) < 0.01f)
    {
      reiniciarControl = false;
    }
  }

  Serial.print(F("Moviendo motor "));
  Serial.print(motor);
  Serial.print(F(" hacia "));
  Serial.print(anguloObjetivo, 2);
  Serial.println(F(" grados."));

  motorAtascado[motorIndex] = false;
  bool exito = moverMotorAAngulo(motorIndex, objetivoAbsoluto, reiniciarControl, true);

  objetivoMotor[motorIndex] = objetivoAbsoluto;
  objetivoValidoMotor[motorIndex] = !motorAtascado[motorIndex];
  ultimaCorreccionMotor[motorIndex] = millis();

  if (!exito && !reiniciarControl)
  {
    // Permite un reintento pronto en caso de fallos usando el objetivo previo.
    ultimaCorreccionMotor[motorIndex] = 0;
  }
}

void procesarComandosI2C()
{
  bool pendiente = false;
  uint8_t motor = 0;
  float objetivo = 0.0f;
  bool reinicioSolicitado = true;

  noInterrupts();
  if (comandoI2CPendiente)
  {
    motor = comandoI2CMotor;
    objetivo = comandoI2CAngulo;
    reinicioSolicitado = comandoI2CReinicio;
    comandoI2CPendiente = false;
    pendiente = true;
  }
  interrupts();

  if (!pendiente)
  {
    return;
  }

  if (motor >= NUM_MOTORES)
  {
    Serial.print(F("[I2C] Motor fuera de rango: "));
    Serial.println(motor);
    return;
  }

  if (!encoderDetectado[motor])
  {
    Serial.print(F("[I2C] Motor "));
    Serial.print(motor + 1);
    Serial.println(F(" sin encoder detectado. Ignorando comando."));
    return;
  }

  if (estadoCalibracionMotor[motor] != CALIB_COMPLETA)
  {
    Serial.print(F("[I2C] Motor "));
    Serial.print(motor + 1);
    Serial.println(F(" aun no calibrado. Comando descartado."));
    return;
  }

  bool dentroRango = true;
  float objetivoAbsoluto = convertirObjetivoRelativoAAbsoluto(motor, objetivo, dentroRango);
  if (!dentroRango)
  {
    Serial.print(F("[I2C] Objetivo fuera de rango para motor "));
    Serial.print(motor + 1);
    Serial.print(F(". Limites 0 - "));
    Serial.print(rangoAnguloMotor[motor], 2);
    Serial.println(F(" grados."));
    return;
  }

  bool reiniciarControl = true;
  if (objetivoValidoMotor[motor])
  {
    if (!reinicioSolicitado)
    {
      reiniciarControl = false;
    }
    else if (fabs(objetivoMotor[motor] - objetivoAbsoluto) < 0.01f)
    {
      reiniciarControl = false;
    }
  }

  Serial.print(F("[I2C] Moviendo motor "));
  Serial.print(motor + 1);
  Serial.print(F(" hacia "));
  Serial.print(objetivo, 2);
  Serial.println(F(" grados."));

  motorAtascado[motor] = false;
  bool exito = moverMotorAAngulo(motor, objetivoAbsoluto, reiniciarControl, true);

  objetivoMotor[motor] = objetivoAbsoluto;
  objetivoValidoMotor[motor] = !motorAtascado[motor];
  ultimaCorreccionMotor[motor] = millis();

  if (!exito && !reiniciarControl)
  {
    ultimaCorreccionMotor[motor] = 0;
  }
}

void reportarAngulos()
{
  Serial.println(F("Estado de motores:"));
  for (uint8_t i = 0; i < NUM_MOTORES; ++i)
  {
    imprimirAnguloMotor(i);
  }
}

void mantenerObjetivosActivos()
{
  unsigned long momentoActual = millis();
  for (uint8_t motor = 0; motor < NUM_MOTORES; ++motor)
  {
    if (!objetivoValidoMotor[motor])
    {
      continue;
    }
    if (!encoderDetectado[motor])
    {
      continue;
    }
    if (!motorTienePWMPins(motor))
    {
      continue;
    }
    if (motorAtascado[motor])
    {
      continue;
    }
    if (estadoCalibracionMotor[motor] != CALIB_COMPLETA)
    {
      continue;
    }
    if ((momentoActual - ultimaCorreccionMotor[motor]) < CONTROL_INTERVAL_MS)
    {
      continue;
    }

    float anguloActual = 0.0f;
    if (!leerAnguloAcumulado(motor, anguloActual))
    {
      detenerMotor(motor);
      seguimientoInicializado[motor] = false;
      ultimaCorreccionMotor[motor] = momentoActual;
      Serial.print(F("Fallo de lectura al mantener motor "));
      Serial.print(motor + 1);
      Serial.println(F(". Reintentando en el siguiente ciclo."));
      continue;
    }

    float error = objetivoMotor[motor] - anguloActual;
    if (fabs(error) >= ERROR_REPETICION_OBJETIVO)
    {
      moverMotorAAngulo(motor, objetivoMotor[motor], false, false);
      ultimaCorreccionMotor[motor] = millis();
    }
    else
    {
      ultimaCorreccionMotor[motor] = momentoActual;
    }
  }
}

void actualizarBufferEstado()
{
  static unsigned long ultimaActualizacion = 0;
  unsigned long ahora = millis();
  if ((ahora - ultimaActualizacion) < CONTROL_INTERVAL_MS)
  {
    return;
  }

  ultimaActualizacion = ahora;

  uint8_t bufferLocal[ESTADO_BUFFER_LENGTH] = {0};
  bufferLocal[0] = NUM_MOTORES;
  uint8_t cursor = 1;

  for (uint8_t motor = 0; motor < NUM_MOTORES; ++motor)
  {
    uint8_t flags = 0;
    if (encoderDetectado[motor])
    {
      flags |= 0x01;
    }
    if (objetivoValidoMotor[motor])
    {
      flags |= 0x02;
    }
    float diferencia = fabs(objetivoMotor[motor] - anguloAcumuladoMotor[motor]);
    if (encoderDetectado[motor] && objetivoValidoMotor[motor] && diferencia > TOLERANCIA_GRADOS)
    {
      flags |= 0x04;
    }
    if (estadoCalibracionMotor[motor] == CALIB_COMPLETA)
    {
      flags |= 0x08;
    }
    if (estadoCalibracionMotor[motor] == CALIB_BUSCANDO_MIN || estadoCalibracionMotor[motor] == CALIB_BUSCANDO_MAX)
    {
      flags |= 0x10;
    }
    if (estadoCalibracionMotor[motor] == CALIB_ERROR)
    {
      flags |= 0x20;
    }

    bufferLocal[cursor++] = flags;

    union
    {
      float valor;
      uint8_t bytes[sizeof(float)];
    } conversion;

    float anguloRelativo = obtenerAnguloRelativo(motor, anguloAcumuladoMotor[motor]);
    float rangoMinimo = 0.0f;
    float rangoMaximo = rangoAnguloMotor[motor];
    float progreso = progresoCalibracionMotor[motor];
    if (estadoCalibracionMotor[motor] == CALIB_COMPLETA)
    {
      progreso = 1.0f;
    }
    else if (estadoCalibracionMotor[motor] == CALIB_NO_INICIADA)
    {
      progreso = 0.0f;
    }
    else if (estadoCalibracionMotor[motor] == CALIB_ERROR)
    {
      progreso = -1.0f;
    }

    conversion.valor = anguloRelativo;
    memcpy(&bufferLocal[cursor], conversion.bytes, sizeof(float));
    cursor += sizeof(float);

    conversion.valor = rangoMinimo;
    memcpy(&bufferLocal[cursor], conversion.bytes, sizeof(float));
    cursor += sizeof(float);

    conversion.valor = rangoMaximo;
    memcpy(&bufferLocal[cursor], conversion.bytes, sizeof(float));
    cursor += sizeof(float);

    conversion.valor = progreso;
    memcpy(&bufferLocal[cursor], conversion.bytes, sizeof(float));
    cursor += sizeof(float);
  }

  noInterrupts();
  memcpy(estadoI2CBuffer, bufferLocal, ESTADO_BUFFER_LENGTH);
  interrupts();
}

void imprimirAnguloMotor(uint8_t motor)
{
  if (motor >= NUM_MOTORES)
  {
    return;
  }

  Serial.print(F("Motor "));
  Serial.print(motor + 1);

  if (!encoderDetectado[motor])
  {
    Serial.println(F(": sin encoder detectado."));
    return;
  }

  float angulo = 0.0f;
  if (!leerAnguloAcumulado(motor, angulo))
  {
    Serial.println(F(": error al leer el encoder."));
    return;
  }

  if (estadoCalibracionMotor[motor] == CALIB_ERROR)
  {
    Serial.println(F(": calibracion fallida. Revise limites mecanicos."));
    return;
  }

  if (estadoCalibracionMotor[motor] != CALIB_COMPLETA)
  {
    Serial.print(F(": calibracion en curso. Angulo acumulado "));
    Serial.print(angulo, 2);
    Serial.println(F(" grados."));
    return;
  }

  float relativo = obtenerAnguloRelativo(motor, angulo);
  Serial.print(F(": "));
  Serial.print(relativo, 2);
  Serial.print(F(" grados (rango 0 - "));
  Serial.print(rangoAnguloMotor[motor], 2);
  Serial.println(F(")."));
}

bool moverMotorAAngulo(uint8_t motor, float objetivo, bool reiniciarControl, bool verbose)
{
  if (motor >= NUM_MOTORES)
  {
    return false;
  }

  if (!motorTienePWMPins(motor))
  {
    if (verbose)
    {
      Serial.print(F("Motor "));
      Serial.print(motor + 1);
      Serial.println(F(" sin pines PWM configurados. Movimiento cancelado."));
    }
    return false;
  }

  if (!encoderDetectado[motor])
  {
    if (verbose)
    {
      Serial.print(F("Motor "));
      Serial.print(motor + 1);
      Serial.println(F(" sin encoder: no es posible mover a un angulo especifico."));
    }
    return false;
  }

  if (reiniciarControl)
  {
    ultimoErrorMotor[motor] = 0.0f;
    integralErrorMotor[motor] = 0.0f;
    derivadaFiltradaMotor[motor] = 0.0f;
  }

  unsigned long inicio = millis();
  unsigned long instanteAnterior = inicio;

  float anguloActual = 0.0f;
  if (!leerAnguloAcumulado(motor, anguloActual))
  {
    if (verbose)
    {
      Serial.print(F("Lectura inicial de encoder fallida para motor "));
      Serial.print(motor + 1);
      Serial.println(F("."));
    }
    return false;
  }

  ultimoAnguloMovimientoMotor[motor] = anguloActual;
  ultimoCambioEncoderMotor[motor] = millis();
  motorAtascado[motor] = false;

  while (millis() - inicio <= TIEMPO_MAX_MOV_MS)
  {
    if (!leerAnguloAcumulado(motor, anguloActual))
    {
      detenerMotor(motor);
      integralErrorMotor[motor] = 0.0f;
      derivadaFiltradaMotor[motor] = 0.0f;
      ultimoErrorMotor[motor] = 0.0f;
      seguimientoInicializado[motor] = false;
      if (verbose)
      {
        Serial.print(F("Lectura de encoder fallida para motor "));
        Serial.print(motor + 1);
        Serial.println(F("."));
      }
      return false;
    }
    float error = objetivo - anguloActual;

    unsigned long instanteActual = millis();
    float deltaTiempo = static_cast<float>(instanteActual - instanteAnterior) / 1000.0f;
    if (deltaTiempo <= 0.0f)
    {
      deltaTiempo = static_cast<float>(CONTROL_INTERVAL_MS) / 1000.0f;
    }

    float absError = fabs(error);
    bool cambioSentido = (ultimoErrorMotor[motor] > 0 && error < 0) ||
                         (ultimoErrorMotor[motor] < 0 && error > 0);

    if (absError <= TOLERANCIA_GRADOS || (cambioSentido && absError < (TOLERANCIA_GRADOS * 2.0f)))
    {
      detenerMotor(motor);
      if (reiniciarControl)
      {
        integralErrorMotor[motor] = 0.0f;
        derivadaFiltradaMotor[motor] = 0.0f;
        ultimoErrorMotor[motor] = 0.0f;
      }
      if (verbose)
      {
        Serial.print(F("Motor "));
        Serial.print(motor + 1);
        Serial.print(F(" posicionado en "));
        Serial.print(anguloActual, 2);
        Serial.println(F(" grados."));
      }
      return true;
    }

    integralErrorMotor[motor] += error * deltaTiempo;
    integralErrorMotor[motor] = constrain(integralErrorMotor[motor], -LIMITE_INTEGRAL, LIMITE_INTEGRAL);

    float derivada = (error - ultimoErrorMotor[motor]) / deltaTiempo;
    derivadaFiltradaMotor[motor] = (FILTRO_DERIVADA * derivadaFiltradaMotor[motor]) +
                                   ((1.0f - FILTRO_DERIVADA) * derivada);

    float salidaPID = (GANANCIA_KP * error) +
                      (GANANCIA_KI * integralErrorMotor[motor]) +
                      (GANANCIA_KD * derivadaFiltradaMotor[motor]);

    int16_t pwm = static_cast<int16_t>((salidaPID >= 0.0f) ? (salidaPID + 0.5f) : (salidaPID - 0.5f));
    pwm = constrain(pwm, -static_cast<int16_t>(PWM_MAX), static_cast<int16_t>(PWM_MAX));

    if (pwm != 0)
    {
      uint8_t minimoAplicable = PWM_MIN_CERCANIA / 2;
      if (minimoAplicable == 0)
      {
        minimoAplicable = 1;
      }
      if (absError >= ERROR_APLICA_PWM_MIN)
      {
        minimoAplicable = PWM_MIN;
      }
      else
      {
        float factor = absError / ERROR_APLICA_PWM_MIN;
        uint8_t pwmSuave = static_cast<uint8_t>(PWM_MIN_CERCANIA * factor);
        if (pwmSuave > minimoAplicable)
        {
          minimoAplicable = pwmSuave;
        }
      }

      int16_t pwmAbsoluto = abs(pwm);
      if (pwmAbsoluto < minimoAplicable)
      {
        pwm = (pwm > 0) ? static_cast<int16_t>(minimoAplicable)
                        : -static_cast<int16_t>(minimoAplicable);
      }
    }

    bool sinMovimiento = (instanteActual - ultimoCambioEncoderMotor[motor]) >= TIEMPO_MAX_SIN_MOVIMIENTO_MS;
    if (pwm != 0 && sinMovimiento)
    {
      detenerMotor(motor);
      integralErrorMotor[motor] = 0.0f;
      derivadaFiltradaMotor[motor] = 0.0f;
      ultimoErrorMotor[motor] = 0.0f;
      motorAtascado[motor] = true;
      objetivoValidoMotor[motor] = false;
      if (verbose)
      {
        Serial.print(F("Motor "));
        Serial.print(motor + 1);
        Serial.println(F(" detenido por posible atasco (sin movimiento detectado)."));
      }
      return false;
    }

    fijarMotor(motor, pwm);

    instanteAnterior = instanteActual;
    ultimoErrorMotor[motor] = error;
    delay(CONTROL_INTERVAL_MS);
  }

  detenerMotor(motor);
  if (reiniciarControl)
  {
    integralErrorMotor[motor] = 0.0f;
    derivadaFiltradaMotor[motor] = 0.0f;
    ultimoErrorMotor[motor] = 0.0f;
  }
  if (verbose)
  {
    Serial.print(F("Tiempo agotado para motor "));
    Serial.print(motor + 1);
    Serial.println(F(" sin alcanzar el objetivo."));
  }
  return false;
}

float obtenerAnguloRelativo(uint8_t motor, float anguloAbsoluto)
{
  if (motor >= NUM_MOTORES)
  {
    return anguloAbsoluto;
  }

  if (estadoCalibracionMotor[motor] != CALIB_COMPLETA)
  {
    return anguloAbsoluto;
  }

  float relativo = anguloAbsoluto - offsetAnguloMotor[motor];
  if (relativo < 0.0f)
  {
    relativo = 0.0f;
  }
  if (rangoAnguloMotor[motor] > 0.0f)
  {
    if (relativo > rangoAnguloMotor[motor])
    {
      relativo = rangoAnguloMotor[motor];
    }
  }
  return relativo;
}

float convertirObjetivoRelativoAAbsoluto(uint8_t motor, float objetivoRelativo, bool &dentroRango)
{
  dentroRango = true;
  if (motor >= NUM_MOTORES)
  {
    return objetivoRelativo;
  }

  if (estadoCalibracionMotor[motor] != CALIB_COMPLETA)
  {
    return objetivoRelativo;
  }

  float rango = rangoAnguloMotor[motor];
  if (objetivoRelativo < 0.0f - 0.0001f || objetivoRelativo > rango + 0.0001f)
  {
    dentroRango = false;
  }

  float objetivoClampeado = objetivoRelativo;
  if (rango > 0.0f)
  {
    objetivoClampeado = constrain(objetivoRelativo, 0.0f, rango);
  }

  return offsetAnguloMotor[motor] + objetivoClampeado;
}

bool buscarLimiteMotor(uint8_t motor, int16_t pwm, float &anguloLimite)
{
  if (motor >= NUM_MOTORES)
  {
    return false;
  }

  if (!motorTienePWMPins(motor))
  {
    return false;
  }

  float anguloActual = 0.0f;
  if (!leerAnguloAcumulado(motor, anguloActual))
  {
    return false;
  }

  unsigned long inicio = millis();
  unsigned long ultimoMovimiento = millis();
  float anguloAnterior = anguloActual;
  bool seMovio = false;

  fijarMotor(motor, pwm);

  while ((millis() - inicio) <= CALIB_TIEMPO_MAX_BUSQUEDA_MS)
  {
    delay(CONTROL_INTERVAL_MS);
    float nuevoAngulo = 0.0f;
    if (!leerAnguloAcumulado(motor, nuevoAngulo))
    {
      detenerMotor(motor);
      return false;
    }

    float delta = fabs(nuevoAngulo - anguloAnterior);
    if (delta >= CALIB_UMBRAL_MOVIMIENTO)
    {
      ultimoMovimiento = millis();
      anguloAnterior = nuevoAngulo;
      seMovio = true;
    }
    else if (seMovio && (millis() - ultimoMovimiento) >= CALIB_TIEMPO_SIN_CAMBIO_MS)
    {
      anguloLimite = nuevoAngulo;
      detenerMotor(motor);
      return true;
    }
  }

  detenerMotor(motor);
  if (!seMovio)
  {
    anguloLimite = anguloAnterior;
  }
  return false;
}

bool calibrarMotor(uint8_t motor)
{
  if (motor >= NUM_MOTORES)
  {
    return false;
  }

  if (!motorTienePWMPins(motor) || !encoderDetectado[motor])
  {
    return false;
  }

  objetivoValidoMotor[motor] = false;
  motorAtascado[motor] = false;

  float limiteMin = 0.0f;
  float limiteMax = 0.0f;

  estadoCalibracionMotor[motor] = CALIB_BUSCANDO_MIN;
  progresoCalibracionMotor[motor] = 0.05f;

  if (!buscarLimiteMotor(motor, -static_cast<int16_t>(PWM_CALIBRACION), limiteMin))
  {
    estadoCalibracionMotor[motor] = CALIB_ERROR;
    progresoCalibracionMotor[motor] = 0.0f;
    detenerMotor(motor);
    return false;
  }

  progresoCalibracionMotor[motor] = 0.5f;

  fijarMotor(motor, static_cast<int16_t>(PWM_CALIBRACION));
  delay(200);
  detenerMotor(motor);
  delay(100);

  estadoCalibracionMotor[motor] = CALIB_BUSCANDO_MAX;

  if (!buscarLimiteMotor(motor, static_cast<int16_t>(PWM_CALIBRACION), limiteMax))
  {
    estadoCalibracionMotor[motor] = CALIB_ERROR;
    progresoCalibracionMotor[motor] = 0.5f;
    detenerMotor(motor);
    return false;
  }

  float rango = limiteMax - limiteMin;
  if (rango <= 1.0f)
  {
    estadoCalibracionMotor[motor] = CALIB_ERROR;
    progresoCalibracionMotor[motor] = 0.5f;
    return false;
  }

  offsetAnguloMotor[motor] = limiteMin;
  rangoAnguloMotor[motor] = rango;

  float posicionDescanso = limiteMin + CALIB_HOLGURA_RETORNO;
  if (posicionDescanso > limiteMax)
  {
    posicionDescanso = limiteMin;
  }

  if (!moverMotorAAngulo(motor, posicionDescanso, true, false))
  {
    estadoCalibracionMotor[motor] = CALIB_ERROR;
    progresoCalibracionMotor[motor] = 0.8f;
    return false;
  }

  objetivoMotor[motor] = posicionDescanso;
  objetivoValidoMotor[motor] = true;
  ultimaCorreccionMotor[motor] = millis();
  ultimoCambioEncoderMotor[motor] = millis();
  ultimoAnguloMovimientoMotor[motor] = posicionDescanso;

  estadoCalibracionMotor[motor] = CALIB_COMPLETA;
  progresoCalibracionMotor[motor] = 1.0f;

  return true;
}

void fijarMotor(uint8_t motor, int16_t pwm)
{
  if (motor >= NUM_MOTORES)
  {
    return;
  }

  if (!motorTienePWMPins(motor))
  {
    return;
  }

  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);

  if (pwm > 0)
  {
    analogWrite(RPWM_PINS[motor], pwm);
    analogWrite(LPWM_PINS[motor], 0);
  }
  else if (pwm < 0)
  {
    analogWrite(RPWM_PINS[motor], 0);
    analogWrite(LPWM_PINS[motor], -pwm);
  }
  else
  {
    detenerMotor(motor);
  }
}

void detenerMotor(uint8_t motor)
{
  if (motor >= NUM_MOTORES)
  {
    return;
  }

  if (!motorTienePWMPins(motor))
  {
    return;
  }

  analogWrite(RPWM_PINS[motor], 0);
  analogWrite(LPWM_PINS[motor], 0);
}

bool motorTienePWMPins(uint8_t motor)
{
  if (motor >= NUM_MOTORES)
  {
    return false;
  }

  return (RPWM_PINS[motor] != 0) || (LPWM_PINS[motor] != 0);
}

void inicializarSeguimientoAngulo(uint8_t motor, uint16_t lecturaRaw)
{
  if (motor >= NUM_MOTORES)
  {
    return;
  }

  ultimoValorRawMotor[motor] = lecturaRaw;
  ticksAcumuladosMotor[motor] = lecturaRaw;
  contadorVueltasMotor[motor] = 0;
  anguloAcumuladoMotor[motor] = (static_cast<float>(ticksAcumuladosMotor[motor]) * 360.0f) / 4096.0f;
  ultimoAnguloMedidoMotor[motor] = anguloAcumuladoMotor[motor];
  ultimoAnguloMovimientoMotor[motor] = anguloAcumuladoMotor[motor];
  ultimoCambioEncoderMotor[motor] = millis();
  motorAtascado[motor] = false;
  seguimientoInicializado[motor] = true;
}

bool leerAnguloAcumulado(uint8_t motor, float &anguloAcumulado)
{
  if (motor >= NUM_MOTORES)
  {
    return false;
  }

  uint16_t lecturaRaw = 0;
  if (!leerAS5600Raw(motor, lecturaRaw))
  {
    return false;
  }

  if (!seguimientoInicializado[motor])
  {
    inicializarSeguimientoAngulo(motor, lecturaRaw);
  }
  else
  {
    int32_t deltaRaw = static_cast<int32_t>(lecturaRaw) - static_cast<int32_t>(ultimoValorRawMotor[motor]);
    int vueltasDelta = 0;

    while (deltaRaw > 2048)
    {
      deltaRaw -= 4096;
      --vueltasDelta;
    }
    while (deltaRaw < -2048)
    {
      deltaRaw += 4096;
      ++vueltasDelta;
    }

    ticksAcumuladosMotor[motor] += deltaRaw;
    contadorVueltasMotor[motor] += vueltasDelta;
  }

  ultimoValorRawMotor[motor] = lecturaRaw;

  anguloAcumuladoMotor[motor] = (static_cast<float>(ticksAcumuladosMotor[motor]) * 360.0f) / 4096.0f;
  ultimoAnguloMedidoMotor[motor] = anguloAcumuladoMotor[motor];
  anguloAcumulado = anguloAcumuladoMotor[motor];

  float deltaMovimiento = fabs(anguloAcumuladoMotor[motor] - ultimoAnguloMovimientoMotor[motor]);
  if (deltaMovimiento >= MIN_VARIACION_ANGULO_ATASCO)
  {
    ultimoAnguloMovimientoMotor[motor] = anguloAcumuladoMotor[motor];
    ultimoCambioEncoderMotor[motor] = millis();
    motorAtascado[motor] = false;
  }
  return true;
}

float leerAnguloGrados(uint8_t canal)
{
  uint16_t valorRaw = 0;
  if (!leerAS5600Raw(canal, valorRaw))
  {
    return NAN;
  }
  return (valorRaw * 360.0f) / 4096.0f;
}

bool leerAS5600Raw(uint8_t canal, uint16_t &valor)
{
  seleccionarCanalMux(canal);

  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(0x0C); // registro RAW ANGLE (MSB)
  if (Wire.endTransmission(false) != 0)
  {
    return false;
  }

  uint8_t recibidos = Wire.requestFrom(AS5600_ADDRESS, (uint8_t)2);
  if (recibidos < 2)
  {
    while (Wire.available())
    {
      Wire.read();
    }
    return false;
  }

  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();

  valor = ((uint16_t)msb << 8 | lsb) & 0x0FFF;
  return true;
}

void seleccionarCanalMux(uint8_t canal)
{
  if (canal >= 8)
  {
    return;
  }

  Wire.beginTransmission(MUX_ADDRESS);
  Wire.write(1 << canal);
  Wire.endTransmission();
}

bool detectarEncoder(uint8_t canal)
{
  uint16_t valor = 0;
  return leerAS5600Raw(canal, valor);
}

void onI2CReceive(int bytesRecibidos)
{
  if (bytesRecibidos <= 0)
  {
    return;
  }

  uint8_t comando = Wire.read();
  --bytesRecibidos;

  if (comando == COMANDO_I2C_OBJETIVO)
  {
    if (bytesRecibidos < 5)
    {
      while (Wire.available())
      {
        Wire.read();
      }
      return;
    }

    uint8_t motor = 0;
    if (Wire.available())
    {
      motor = Wire.read();
      --bytesRecibidos;
    }

    union
    {
      float valor;
      uint8_t bytes[sizeof(float)];
    } conversion;

    for (uint8_t i = 0; i < sizeof(float); ++i)
    {
      if (Wire.available())
      {
        conversion.bytes[i] = Wire.read();
        --bytesRecibidos;
      }
      else
      {
        conversion.bytes[i] = 0;
      }
    }

    bool reinicio = true;
    if (Wire.available())
    {
      reinicio = Wire.read() != 0;
      --bytesRecibidos;
    }

    while (Wire.available())
    {
      Wire.read();
    }

    comandoI2CMotor = motor;
    comandoI2CAngulo = conversion.valor;
    comandoI2CReinicio = reinicio;
    comandoI2CPendiente = true;
  }
  else
  {
    while (Wire.available())
    {
      Wire.read();
    }
  }
}

void onI2CRequest()
{
  Wire.write(estadoI2CBuffer, ESTADO_BUFFER_LENGTH);
}
