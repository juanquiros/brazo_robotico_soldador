# Control de Motores con BTS7960, Multiplexor I2C y Encoders AS5600

Este proyecto para Arduino permite manejar hasta cinco motores de corriente continua usando drivers **BTS7960**, leer su posición mediante codificadores magnéticos **AS5600** conectados a través de un multiplexor I2C **PCA9548A/TCA9548A** y comandar los movimientos a un ángulo específico desde la consola serial.

## Componentes

- Arduino (Uno, Mega o compatible con soporte para `analogWrite`).
- 5 drivers puente H BTS7960 (uno por motor).
- Motores DC con sensor magnético AS5600 para medición de ángulos.
- Multiplexor I2C PCA9548A/TCA9548A.
- Fuente de alimentación adecuada para los motores y el Arduino.
- Cables de conexión.

## Esquema de Conexión

### Señales de Control

| Motor | Pin RPWM Arduino | Pin LPWM Arduino | Canal Multiplexor |
|-------|------------------|------------------|-------------------|
| 1     | D5               | D4               | 0                 |
| 2     | D6               | D7               | 1                 |
| 3     | D9               | D8               | 2                 |
| 4     | D10              | D12              | 3                 |
| 5     | D11              | D13              | 4                 |

> Ajusta los pines según tu cableado real. Los canales del multiplexor pueden reasignarse siempre que coincidan con el índice del motor en el código (`motor 1` → canal `0`, etc.).

### Conexiones del BTS7960

Para cada motor:

1. Conecta `RPWM` y `LPWM` del driver BTS7960 a los pines listados en la tabla.
2. Alimenta el BTS7960 con la fuente de motores (VCC motor y GND motor).
3. Une la masa del Arduino con la masa de la fuente de los motores.
4. Conecta el motor a las salidas `L_OUT` y `R_OUT` del BTS7960.

### Conexiones del AS5600

1. Alimenta cada encoder con 5V (o 3.3V) y GND desde el multiplexor.
2. Conecta `SDA` y `SCL` de cada AS5600 al canal correspondiente del multiplexor.
3. Conecta `SDA` y `SCL` del multiplexor a los pines I2C del Arduino (`A4` y `A5` en Arduino Uno).
4. Conecta `VIN` del multiplexor a 5V y `GND` a tierra común.

## Funcionamiento del Programa

1. **Inicio:** Al encender el Arduino, el programa inicializa la comunicación I2C y configura los pines PWM de cada BTS7960.
2. **Detección de encoders:** Recorre los 5 canales del multiplexor y detecta si hay un AS5600 presente leyendo el registro `RAW_ANGLE`.
3. **Homing automático:** Para cada motor con encoder detectado, se ejecuta un movimiento de alineación hacia los `0°`. Si un canal no tiene encoder, ese motor se mantiene detenido.
4. **Control serial:** Una vez inicializado, puedes introducir comandos en el monitor serial (115200 baudios) con el formato:
   ```
   <numero_motor> <angulo>
   ```
   Ejemplo: `2 180` moverá el motor 2 a 180°. El ángulo se limita al rango `0° – 360°`.
5. **Control proporcional:** El código aplica un control proporcional sencillo (parámetro `GANANCIA_P`) y limita el PWM entre `PWM_MIN` y `PWM_MAX` para aproximar el motor al ángulo solicitado. Si no logra alcanzar el objetivo dentro de `TIEMPO_MAX_MOV_MS`, se detiene e informa por serial.
6. **Protecciones:** Si se pierde la lectura del encoder durante un movimiento, el motor se detiene y se notifica el error. No se aceptan comandos para motores sin encoder detectado.

## Ajustes y Calibración

- **Reasignar pines:** Modifica los arreglos `RPWM_PINS` y `LPWM_PINS` en `main.ino` para adaptarlos a tu hardware.
- **Parámetros de control:** Ajusta `GANANCIA_P`, `PWM_MIN`, `PWM_MAX`, `TOLERANCIA_GRADOS` y `TIEMPO_MAX_MOV_MS` para refinar la respuesta de tu sistema mecánico.
- **Número de motores:** Cambia `NUM_MOTORES` si utilizas menos o más canales, y actualiza los arreglos correspondientes.

## Requisitos de Software

- Arduino IDE o plataforma compatible.
- Librería estándar `Wire` (incluida en el núcleo de Arduino).

## Uso

1. Abre el proyecto en el Arduino IDE y selecciona la tarjeta y puerto adecuados.
2. Carga `main.ino` al Arduino.
3. Abre el Monitor Serial a 115200 baudios con salto de línea (`NL`) como terminador.
4. Introduce los comandos para mover cada motor según sea necesario.

## Solución de Problemas

- Si un motor no responde, verifica que su encoder esté correctamente cableado y alimentado. El monitor serial mostrará "encoder no detectado" en caso de fallo.
- Si el movimiento es errático, revisa la alineación del imán del AS5600 y la alimentación del BTS7960.
- Asegura una masa común entre el Arduino, el multiplexor, los encoders y los drivers BTS7960.

