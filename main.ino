
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
static const size_t ESTADO_BUFFER_LENGTH = 1 + NUM_MOTORES * (1 + sizeof(float));

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
      if (motorTienePWMPins(i))
      {
        Serial.print(F("Alineando motor "));
        Serial.print(i + 1);
        Serial.println(F(" a 0 grados."));
        bool homingExitoso = moverMotorAAngulo(i, 0.0f, true, true);
        if (homingExitoso)
        {
          objetivoMotor[i] = 0.0f;
          objetivoValidoMotor[i] = true;
          ultimaCorreccionMotor[i] = millis();
        }
      }
      else
      {
        Serial.print(F("Motor "));
        Serial.print(i + 1);
        Serial.println(F(" detectado sin pines PWM configurados: omitiendo alineacion."));
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
  bool reiniciarControl = true;
  if (objetivoValidoMotor[motorIndex])
  {
    if (fabs(objetivoMotor[motorIndex] - anguloObjetivo) < 0.01f)
    {
      reiniciarControl = false;
    }
  }

  Serial.print(F("Moviendo motor "));
  Serial.print(motor);
  Serial.print(F(" hacia "));
  Serial.print(anguloObjetivo, 2);
  Serial.println(F(" grados."));

  bool exito = moverMotorAAngulo(motorIndex, anguloObjetivo, reiniciarControl, true);

  objetivoMotor[motorIndex] = anguloObjetivo;
  objetivoValidoMotor[motorIndex] = true;
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

  bool reiniciarControl = true;
  if (objetivoValidoMotor[motor])
  {
    if (!reinicioSolicitado)
    {
      reiniciarControl = false;
    }
    else if (fabs(objetivoMotor[motor] - objetivo) < 0.01f)
    {
      reiniciarControl = false;
    }
  }

  Serial.print(F("[I2C] Moviendo motor "));
  Serial.print(motor + 1);
  Serial.print(F(" hacia "));
  Serial.print(objetivo, 2);
  Serial.println(F(" grados."));

  bool exito = moverMotorAAngulo(motor, objetivo, reiniciarControl, true);

  objetivoMotor[motor] = objetivo;
  objetivoValidoMotor[motor] = true;
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

    bufferLocal[cursor++] = flags;

    union
    {
      float valor;
      uint8_t bytes[sizeof(float)];
    } conversion;

    conversion.valor = anguloAcumuladoMotor[motor];
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

  Serial.print(F(": "));
  Serial.print(angulo, 2);
  Serial.println(F(" grados (acumulados)."));
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
