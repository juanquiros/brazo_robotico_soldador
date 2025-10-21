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

| Motor | Pin R_EN Arduino | Pin L_EN Arduino | Pin RPWM Arduino | Pin LPWM Arduino | Canal Multiplexor |
|-------|------------------|------------------|------------------|------------------|-------------------|
| 1     | 5V (o D2*)       | 5V (o D3*)       | D5               | D4               | 0                 |
| 2     | 5V (o D14*)      | 5V (o D15*)      | D6               | D7               | 1                 |
| 3     | 5V (o D16*)      | 5V (o D17*)      | D9               | D8               | 2                 |
| 4     | 5V (o D18*)      | 5V (o D19*)      | D10              | D12              | 3                 |
| 5     | 5V (o D20*)      | 5V (o D21*)      | D11              | D13              | 4                 |

\* Ajusta los pines `R_EN`/`L_EN` según tu placa (en Arduino Uno, `D14–D19` corresponden a los pines analógicos `A0–A5`). Puedes fijarlos directamente a 5V si no deseas controlarlos por software. Si los conectas al Arduino, actualiza los arreglos `REN_PINS` y `LEN_PINS` en `main.ino` con los números de pin correspondientes.

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

## Funcionamiento del Programa

1. **Inicio:** Al encender el Arduino, el programa inicializa la comunicación I2C y configura los pines PWM de cada BTS7960.
2. **Detección de encoders:** Recorre los 5 canales del multiplexor y detecta si hay un AS5600 presente leyendo el registro `RAW_ANGLE`.
3. **Homing automático:** Para cada motor con encoder detectado, se ejecuta un movimiento de alineación hacia los `0°`. Si un canal no tiene encoder, ese motor se mantiene detenido.
4. **Control serial:** Una vez inicializado, puedes introducir comandos en el monitor serial (115200 baudios). Comandos disponibles:
   - `<numero_motor> <angulo>`: mueve el motor indicado al ángulo solicitado. Ejemplo: `2 180` moverá el motor 2 a 180°. El ángulo se limita al rango `0° – 360°`.
   - `<numero_motor>`: muestra por serial el ángulo actual del motor indicado siempre que tenga encoder disponible.
   - `estado` (alias `angulos` o `status`): lista el ángulo actual de todos los motores detectados.
5. **Control PID:** El código calcula la velocidad del motor mediante un lazo PID discreto (`GANANCIA_KP`, `GANANCIA_KI`, `GANANCIA_KD`) con limitación del término integral y filtrado de la derivada para reducir el ruido del encoder. Sobre esa salida se aplican mínimos dinámicos de PWM (`PWM_MIN`, `PWM_MIN_CERCANIA`) que permiten vencer la fricción sin generar oscilaciones al aproximarse al objetivo. Si no logra alcanzar el ángulo dentro de `TIEMPO_MAX_MOV_MS`, se detiene e informa por serial.
6. **Protecciones:** Si se pierde la lectura del encoder durante un movimiento, el motor se detiene y se notifica el error. No se aceptan comandos para motores sin encoder detectado.

## Ajustes y Calibración

- **Reasignar pines:** Modifica los arreglos `RPWM_PINS`, `LPWM_PINS`, `REN_PINS` y `LEN_PINS` en `main.ino` para adaptarlos a tu hardware. Usa `-1` cuando un pin `R_EN/L_EN` esté cableado permanentemente a 5V.
- **Parámetros de control:** Ajusta `GANANCIA_KP`, `GANANCIA_KI`, `GANANCIA_KD`, `LIMITE_INTEGRAL`, `FILTRO_DERIVADA`, `PWM_MIN`, `PWM_MIN_CERCANIA`, `PWM_MAX`, `ERROR_APLICA_PWM_MIN`, `TOLERANCIA_GRADOS` y `TIEMPO_MAX_MOV_MS` para refinar la respuesta de tu sistema mecánico.
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

