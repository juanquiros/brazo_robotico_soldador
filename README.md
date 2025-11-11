# Control de Motores con BTS7960, Multiplexor I2C y Encoders AS5600

Este proyecto para Arduino permite manejar hasta cinco motores de corriente continua usando drivers **BTS7960**, leer su posición mediante codificadores magnéticos **AS5600** conectados a través de un multiplexor I2C **PCA9548A/TCA9548A** y comandar los movimientos a un ángulo específico desde la consola serial.

## Componentes

- Arduino (Uno, Mega o compatible con soporte para `analogWrite`).
- 5 drivers puente H BTS7960 (uno por motor).
- Motores DC con sensor magnético AS5600 para medición de ángulos.
- Multiplexor I2C PCA9548A/TCA9548A.
- Módulo ESP32 (opcional) para la interfaz SCADA inalámbrica.
- Fuente de alimentación adecuada para los motores y el Arduino.
- Cables de conexión.

## Esquema de Conexión

### Señales de Control

| Motor | Pin R_EN Arduino | Pin L_EN Arduino | Pin RPWM Arduino | Pin LPWM Arduino | Canal Multiplexor |
|-------|------------------|------------------|------------------|------------------|-------------------|
| 1     | D7 (o 5V)        | D8 (o 5V)        | D5               | D6               | 0                 |
| 2     | 5V               | 5V               | D4               | D11              | 1                 |
| 3     | 5V               | 5V               | D9               | D13              | 2                 |
| 4     | 5V               | 5V               | D10              | D12              | 3                 |
| 5     | — (configurable) | — (configurable) | — (configurable) | — (configurable) | 4                 |

\* Ajusta los pines `R_EN`/`L_EN` según tu placa. Puedes fijarlos directamente a 5V si no deseas controlarlos por software. Si los conectas al Arduino, actualiza los arreglos `REN_PINS` y `LEN_PINS` en `main.ino` con los números de pin correspondientes.

> El quinto motor queda sin asignar por defecto para que completes los pines necesarios en `main.ino` si tu instalación realmente usa los cinco canales.

> Ajusta los pines según tu cableado real. Los canales del multiplexor pueden reasignarse siempre que coincidan con el índice del motor en el código (`motor 1` → canal `0`, etc.).

### Conexiones del BTS7960

Para cada motor:

1. Conecta `R_EN`, `L_EN`, `RPWM` y `LPWM` del driver BTS7960 a los pines listados en la tabla (o alimenta `R_EN/L_EN` directamente a 5V si los mantienes siempre habilitados).
2. Alimenta el BTS7960 con la fuente de motores (VCC motor y GND motor).
3. Une la masa del Arduino con la masa de la fuente de los motores.
4. Conecta el motor a las salidas `L_OUT` y `R_OUT` del BTS7960.

### Conexiones del AS5600

1. Alimenta cada encoder con 5V (o 3.3V) y GND desde el multiplexor.
2. Conecta `SDA` y `SCL` de cada AS5600 al canal correspondiente del multiplexor.
3. Conecta `SDA` y `SCL` del multiplexor a los pines I2C del Arduino (`A4` y `A5` en Arduino Uno).
4. Conecta `VIN` del multiplexor a 5V y `GND` a tierra común.

### Enlace ESP32 ↔ Arduino

Si vas a utilizar la interfaz web industrial:

1. Conecta los pines `SDA` y `SCL` del ESP32 (por defecto GPIO 21 y GPIO 22) a los pines I2C del Arduino (A4 y A5 en un Arduino Uno). Mantén la longitud del cable lo más corta posible.
2. Une las masas del ESP32 y del Arduino.
3. No es necesario ningún nivelador lógico: ambas placas operan a 3.3 V en las líneas I2C y el Arduino UNO incorpora resistencias de pull-up compatibles.

| ESP32 | Arduino | Descripción |
|-------|---------|-------------|
| GPIO21 (SDA) | A4 (SDA) | Datos I2C hacia el multiplexor y los encoders. |
| GPIO22 (SCL) | A5 (SCL) | Reloj I2C compartido con el Arduino y el PCA9548A. |
| GND | GND | Referencia común obligatoria entre ambas placas. |
| VIN (5 V) o 3V3 | 5 V (si la fuente lo permite) o fuente externa | Alimenta el ESP32 desde el regulador de 5 V del Arduino o desde una fuente dedicada estable. |

## Funcionamiento del Programa

1. **Inicio:** Al encender el Arduino, el programa inicializa la comunicación I2C y configura los pines PWM de cada BTS7960.
2. **Detección de encoders:** Recorre los 5 canales del multiplexor y detecta si hay un AS5600 presente leyendo el registro `RAW_ANGLE`.
3. **Calibración de límites:** Para cada motor con encoder detectado, el Arduino activa el driver a `PWM_CALIBRACION = 70` y lo hace girar lentamente hasta que el AS5600 deja de registrar movimiento durante `CALIB_TIEMPO_SIN_CAMBIO_MS = 1000 ms`. Ese punto se toma como “cero” mecánico. A continuación libera el tope, invierte el giro y repite la búsqueda para capturar el máximo recorrido disponible. Si no se detecta desplazamiento suficiente o la ventana de búsqueda (`CALIB_TIEMPO_MAX_BUSQUEDA_MS = 15 s`) expira, la calibración del motor se marca en error y no se aceptarán órdenes para ese eje.
4. **Seguimiento multi-vuelta:** Aunque los comandos posteriores trabajan con ángulos relativos al cero encontrado, internamente el firmware sigue desplegando las lecturas de 12 bits del AS5600 para evitar saltos al cruzar 0°/360°. Esto permite conservar la resolución del encoder incluso en transmisiones con reductora y detectar el estancamiento real del eje.
5. **Control serial:** Tras la calibración, introduce comandos en el monitor serial (115200 baudios). Comandos disponibles:
   - `<numero_motor> <angulo>`: mueve el motor indicado a un ángulo dentro del rango aprendido (`0°` hasta el máximo detectado en la calibración). Si envías un valor fuera de rango, el firmware lo rechaza y te informa de los límites disponibles.
   - `<numero_motor>`: muestra por serial el ángulo relativo (0–máximo) del motor indicado siempre que tenga encoder disponible y la calibración se haya completado.
   - `estado` (alias `angulos` o `status`): lista el ángulo relativo y el estado de calibración de todos los motores detectados.
6. **Control PID:** El código calcula la velocidad del motor mediante un lazo PID discreto (`GANANCIA_KP = 0.8`, `GANANCIA_KI = 0.23`, `GANANCIA_KD = 0.45`) con limitación del término integral y derivada sin filtrado adicional (`FILTRO_DERIVADA = 1.0`). Sobre esa salida se aplican mínimos dinámicos de PWM (`PWM_MIN = 60`, `PWM_MIN_CERCANIA = 40`) que permiten vencer la fricción sin generar oscilaciones al aproximarse al objetivo. Si no logra alcanzar el ángulo dentro de `TIEMPO_MAX_MOV_MS = 3000 ms`, se detiene e informa por serial.
7. **Repetición del objetivo:** El último ángulo solicitado queda almacenado y se reintenta automáticamente cuando el error acumulado supera `ERROR_REPETICION_OBJETIVO = 1.0°`. Si vuelves a enviar el mismo valor por serial, el controlador no reinicia el PID, por lo que conserva el término integral acumulado y corrige la deriva remanente del movimiento anterior.
8. **Protecciones:** Si se pierde la lectura del encoder durante un movimiento, el motor se detiene y se notifica el error. No se aceptan comandos para motores sin encoder detectado.
9. **Detección de atascos:** El firmware comprueba continuamente que cada motor avance al menos `0.5°` cuando está activo. Si transcurre más de `1 s` aplicando PWM sin detectar movimiento en el encoder, se asume un atasco mecánico: el controlador detiene el motor, descarta el objetivo y deja constancia por consola para evitar daños.
10. **Límites aprendidos:** Los ángulos enviados por serial o por I2C deben residir entre `0°` y el máximo determinado en la calibración. Los comandos fuera del campo operativo se rechazan y el estado del motor permanece inalterado.

## Control web industrial con ESP32

El archivo `esp32_control.ino` añade una HMI estilo SCADA ejecutada en un ESP32 que se enlaza con el Arduino por I2C.

### Flujo general

1. El ESP32 crea una red Wi-Fi propia (`Brazo-SCADA`, contraseña `Soldador360`) y levanta un servidor HTTP en el puerto 80.
2. Desde cualquier dispositivo conectado a esa red accede a `http://192.168.4.1/` para visualizar el panel.
3. El panel muestra tarjetas con el estado de cada motor: disponibilidad del encoder, ángulo relativo, objetivo activo, progreso de calibración y rangos válidos. Durante la búsqueda de topes verás la barra de avance y alertas en tiempo real.
4. Una sección de "Campo Operativo" permite introducir una coordenada simultánea para todos los motores. Antes de despachar las órdenes, la web valida que cada valor se encuentre dentro del rango detectado y notifica si la coordenada cae fuera del espacio de trabajo.
5. Cada 1.5 s el ESP32 consulta por I2C al Arduino y actualiza la interfaz con efectos visuales y resaltados industriales.

### Protocolo I2C ESP32 ↔ Arduino

- **Dirección del Arduino:** `0x10`. Ajusta `ARDUINO_I2C_ADDRESS` en ambos firmwares si necesitas otra dirección.
- **Comando `0x01` (establecer objetivo):**
  - Byte 0: `0x01`.
  - Byte 1: índice de motor (0–4).
  - Bytes 2‑5: ángulo objetivo en formato `float` IEEE 754 (little endian).
  - Byte 6: bandera de reinicio (`1` reinicia el PID salvo repetición del mismo objetivo, `0` preserva el control integral).
- **Respuesta de estado:** cada petición `Wire.requestFrom` devuelve 1 byte con `NUM_MOTORES` seguido, por motor, de un byte de banderas (`bit0` encoder detectado, `bit1` setpoint válido, `bit2` en corrección, `bit3` calibración finalizada, `bit4` calibración en curso, `bit5` error de calibración) y cuatro `float`: ángulo relativo (`0°–máximo`), mínimo del rango (0°), máximo del rango y progreso de calibración (`0`–`1`, `-1` en caso de error).
- El ESP32 conserva el último objetivo que envió con éxito; cuando el setpoint activo proviene de la web lo muestra junto al ángulo y, si el Arduino recibe una orden externa, indica que el origen del comando es distinto.
- El Arduino actualiza un buffer con la misma frecuencia que el lazo PID (`CONTROL_INTERVAL_MS`) para que el ESP32 obtenga lecturas coherentes sin bloquear el bus.

### Interfaz SCADA

- Diseño oscuro con paneles translúcidos, acentos cian y animaciones breves que evocan sistemas industriales.
- Tarjetas informativas por motor con etiquetas dinámicas (`Encoder`, `Setpoint activo`, `Origen de la orden`, `Corrigiendo`, etc.).
- Formulario de órdenes que envía comandos en segundo plano (`fetch`) y muestra notificaciones tipo *toast* ante éxito o fallo.
- Widget de estado superior con hora de la última actualización, nombre de la red y dirección IP del punto de acceso.
- Panel de campo operativo que valida los ángulos de cada motor contra los límites aprendidos antes de transmitirlos y muestra mensajes claros cuando el punto solicitado está fuera del espacio de trabajo.

## Ajustes y Calibración

- **Reasignar pines:** Modifica los arreglos `RPWM_PINS`, `LPWM_PINS`, `REN_PINS` y `LEN_PINS` en `main.ino` para adaptarlos a tu hardware. Usa `-1` cuando un pin `R_EN/L_EN` esté cableado permanentemente a 5V.
- **Parámetros de control:** Ajusta `GANANCIA_KP`, `GANANCIA_KI`, `GANANCIA_KD`, `LIMITE_INTEGRAL`, `FILTRO_DERIVADA`, `PWM_MIN`, `PWM_MIN_CERCANIA`, `PWM_MAX`, `ERROR_APLICA_PWM_MIN`, `TOLERANCIA_GRADOS`, `ERROR_REPETICION_OBJETIVO` y `TIEMPO_MAX_MOV_MS` para refinar la respuesta de tu sistema mecánico. Recuerda que el objetivo se compara contra el ángulo acumulado, por lo que los signos positivos hacen girar en sentido horario (incrementando grados) y los negativos en sentido antihorario.
- **Calibración automática:** Los parámetros `PWM_CALIBRACION`, `CALIB_UMBRAL_MOVIMIENTO`, `CALIB_TIEMPO_SIN_CAMBIO_MS`, `CALIB_TIEMPO_MAX_BUSQUEDA_MS` y `CALIB_HOLGURA_RETORNO` determinan la velocidad de exploración, la sensibilidad del encoder y la holgura con la que el motor se separa de los topes mecánicos tras detectarlos.
- **Número de motores:** Cambia `NUM_MOTORES` si utilizas menos o más canales, y actualiza los arreglos correspondientes.

## Requisitos de Software

- Arduino IDE o plataforma compatible.
- Librería estándar `Wire` (incluida en el núcleo de Arduino).
- Núcleo ESP32 para Arduino IDE (si compilas `esp32_control.ino`).
- Librerías `WiFi`, `WebServer` y `Wire` para ESP32 (incluidas en el núcleo oficial).

## Uso

1. Abre el proyecto en el Arduino IDE y selecciona la tarjeta y puerto adecuados.
2. Carga `main.ino` al Arduino.
3. Abre el Monitor Serial a 115200 baudios con salto de línea (`NL`) como terminador.
4. Introduce los comandos para mover cada motor según sea necesario.
5. (Opcional) Carga `esp32_control.ino` en un ESP32, conéctalo al bus I2C del Arduino y accede a la interfaz web para controlar y supervisar los motores sin usar la consola serial.

## Solución de Problemas

- Si un motor no responde, verifica que su encoder esté correctamente cableado y alimentado. El monitor serial mostrará "encoder no detectado" en caso de fallo.
- Si el movimiento es errático, revisa la alineación del imán del AS5600 y la alimentación del BTS7960.
- Si un motor se detiene por "atasco" revisa que el eje pueda girar libremente, que el embrague o reductora no estén bloqueados y que exista un cambio perceptible en el encoder al moverlo manualmente antes de volver a ordenar el movimiento.
- Asegura una masa común entre el Arduino, el multiplexor, los encoders y los drivers BTS7960.

