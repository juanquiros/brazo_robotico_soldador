
#include <Wire.h>
#include <math.h>

// =================== CONFIGURACIÓN GENERAL ===================

static const uint8_t NUM_MOTORES = 5;

// Dirección I2C por defecto del multiplexor PCA9548A/TCA9548A
static const uint8_t MUX_ADDRESS = 0x70;

// Dirección I2C del codificador magnético AS5600
static const uint8_t AS5600_ADDRESS = 0x36;

// Pines PWM (RPWM y LPWM) para cada uno de los 5 drivers BTS7960 conectados al Arduino.
// Ajusta los valores de estos arreglos de acuerdo a tu cableado real.
static const uint8_t RPWM_PINS[NUM_MOTORES] = {5, 6, 9, 10, 11};
static const uint8_t LPWM_PINS[NUM_MOTORES] = {4, 7, 8, 12, 13};

// Pines de habilitación (R_EN y L_EN) para cada BTS7960.
// Usa -1 si dejas ese pin permanentemente en HIGH (por ejemplo, cableado a 5 V).
static const int8_t REN_PINS[NUM_MOTORES] = {-1, -1, -1, -1, -1};
static const int8_t LEN_PINS[NUM_MOTORES] = {-1, -1, -1, -1, -1};

// =================== PARÁMETROS DE CONTROL ===================

static const float ANGULO_MIN = 0.0f;
static const float ANGULO_MAX = 360.0f;
static const float TOLERANCIA_GRADOS = 1.5f;   // error permitido
static const uint8_t PWM_MIN = 60;              // velocidad mínima para vencer fricción
static const uint8_t PWM_MAX = 255;             // velocidad máxima
static const float GANANCIA_P = 2.0f;           // Ganancia proporcional simple
static const unsigned long CONTROL_INTERVAL_MS = 30;
static const unsigned long TIEMPO_MAX_MOV_MS = 8000; // tiempo máximo por movimiento

// =================== DECLARACIÓN DE FUNCIONES ===================

void seleccionarCanalMux(uint8_t canal);
bool leerAS5600Raw(uint8_t canal, uint16_t &valor);
float leerAnguloGrados(uint8_t canal);
void detenerMotor(uint8_t motor);
void fijarMotor(uint8_t motor, int16_t pwm);
float ajustarRangoGrados(float grados);
float errorAngular(float objetivo, float actual);
void moverMotorAAngulo(uint8_t motor, float objetivo);
void procesarComandosSerial();
bool detectarEncoder(uint8_t canal);

// =================== ESTADO DE ENCÓDERS DETECTADOS ===================

static bool encoderDetectado[NUM_MOTORES] = {false};

// =================== SETUP ===================

void setup()
{
  Serial.begin(115200);
  while (!Serial) { /* espera a que se abra la consola */ }

  Wire.begin();

  for (uint8_t i = 0; i < NUM_MOTORES; ++i)
  {
    if (REN_PINS[i] >= 0)
    {
      uint8_t pin = static_cast<uint8_t>(REN_PINS[i]);
      pinMode(pin, OUTPUT);
      digitalWrite(pin, HIGH);
    }
    if (LEN_PINS[i] >= 0)
    {
      uint8_t pin = static_cast<uint8_t>(LEN_PINS[i]);
      pinMode(pin, OUTPUT);
      digitalWrite(pin, HIGH);
    }
    pinMode(RPWM_PINS[i], OUTPUT);
    pinMode(LPWM_PINS[i], OUTPUT);
    detenerMotor(i);
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
      Serial.print(F("Alineando motor "));
      Serial.print(i + 1);
      Serial.println(F(" a 0 grados."));
      moverMotorAAngulo(i, 0.0f);
    }
  }

  Serial.println(F("Sistema listo. Escriba: <motor 1-5> <angulo 0-360>."));
}

// =================== LOOP PRINCIPAL ===================

void loop()
{
  procesarComandosSerial();
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

  int separador = linea.indexOf(' ');
  if (separador < 0)
  {
    Serial.println(F("Formato invalido. Use: <motor> <angulo>."));
    return;
  }

  int motor = linea.substring(0, separador).toInt();
  float anguloObjetivo = linea.substring(separador + 1).toFloat();

  if (motor < 1 || motor > NUM_MOTORES)
  {
    Serial.println(F("Numero de motor fuera de rango (1-5)."));
    return;
  }

  if (!encoderDetectado[motor - 1])
  {
    Serial.println(F("Encoder no detectado para ese motor. Movimiento cancelado."));
    return;
  }

  anguloObjetivo = constrain(anguloObjetivo, ANGULO_MIN, ANGULO_MAX);

  Serial.print(F("Moviendo motor "));
  Serial.print(motor);
  Serial.print(F(" hacia "));
  Serial.print(anguloObjetivo, 2);
  Serial.println(F(" grados."));

  moverMotorAAngulo(static_cast<uint8_t>(motor - 1), anguloObjetivo);
}

void moverMotorAAngulo(uint8_t motor, float objetivo)
{
  if (motor >= NUM_MOTORES)
  {
    return;
  }

  if (!encoderDetectado[motor])
  {
    Serial.print(F("Motor "));
    Serial.print(motor + 1);
    Serial.println(F(" sin encoder: no es posible mover a un angulo especifico."));
    return;
  }

  unsigned long inicio = millis();

  while (millis() - inicio <= TIEMPO_MAX_MOV_MS)
  {
    float anguloActual = leerAnguloGrados(motor);
    if (isnan(anguloActual))
    {
      detenerMotor(motor);
      Serial.print(F("Lectura de encoder fallida para motor "));
      Serial.print(motor + 1);
      Serial.println(F("."));
      return;
    }
    float error = errorAngular(objetivo, anguloActual);

    if (fabs(error) <= TOLERANCIA_GRADOS)
    {
      detenerMotor(motor);
      Serial.print(F("Motor "));
      Serial.print(motor + 1);
      Serial.print(F(" posicionado en "));
      Serial.print(anguloActual, 2);
      Serial.println(F(" grados."));
      return;
    }

    int16_t pwm = static_cast<int16_t>(fabs(error) * GANANCIA_P);
    pwm = constrain(pwm, PWM_MIN, PWM_MAX);

    if (error > 0)
    {
      fijarMotor(motor, pwm);
    }
    else
    {
      fijarMotor(motor, -pwm);
    }

    delay(CONTROL_INTERVAL_MS);
  }

  detenerMotor(motor);
  Serial.print(F("Tiempo agotado para motor "));
  Serial.print(motor + 1);
  Serial.println(F(" sin alcanzar el objetivo."));
}

void fijarMotor(uint8_t motor, int16_t pwm)
{
  if (motor >= NUM_MOTORES)
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

  analogWrite(RPWM_PINS[motor], 0);
  analogWrite(LPWM_PINS[motor], 0);
}

float errorAngular(float objetivo, float actual)
{
  float objetivoAjustado = ajustarRangoGrados(objetivo);
  float actualAjustado = ajustarRangoGrados(actual);

  float diferencia = objetivoAjustado - actualAjustado;

  while (diferencia > 180.0f)
  {
    diferencia -= 360.0f;
  }
  while (diferencia < -180.0f)
  {
    diferencia += 360.0f;
  }

  return diferencia;
}

float ajustarRangoGrados(float grados)
{
  while (grados < ANGULO_MIN)
  {
    grados += 360.0f;
  }
  while (grados >= ANGULO_MAX)
  {
    grados -= 360.0f;
  }
  return grados;
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
