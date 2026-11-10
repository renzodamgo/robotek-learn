# Parámetros del Obstacle v3 en el coche

Qué cambia cada parámetro, su valor por defecto y de dónde sale. Los del lanzamiento se pasan al final del comando
(`nombre:=valor`):

```bash
ros2 launch robotek_course car.launch.py strategy:=robotek_obstacle_v3 camera_driver:=picam start_lot:=true \
  lot_exit_speed:=0.9 recover_back:=0.5
```

Valores por defecto = la primera ronda completa desde el aparcamiento (9 nov 2026).

## Del lanzamiento (`car.launch.py`)

### Salida del aparcamiento

| Parámetro | Por defecto | Qué hace |
|---|---|---|
| `start_lot` | `false` | `true`: el coche empieza dentro del aparcamiento, pegado al muro exterior y a ~1 cm del tope de atrás. Mira la dirección con el LiDAR, sale con `lot_exit_steps` y luego da las vueltas sin aparcar. También activa el golpe de arranque y los pulsos del motor y quita las rampas, solo durante la salida (estado `LOT_EXIT`), y pone `motor_min_rps` en 1,5 si no se da. |
| `lot_exit_speed` | `0.9` | m/s de cada pulso de la salida. Con 0,12–0,25 casi no se movía; con 0,7–0,9 sale. |
| `lot_exit_steps` | `L+0.45 R-0.55 L+0.45 R-0.55 L+0.45 R-0.55 L+0.78 C-0.4` | Los pulsos de la salida, separados por espacios. Cada uno es `<lado><sentido><segundos>`: lado `L` (todo a la izquierda, lejos del muro), `R` (todo a la derecha) o `C` (ruedas al centro, servo ~1403 µs); sentido `+` adelante o `-` atrás. Escrito para antihorario; en horario se aplica en espejo. Un paso mal escrito para el nodo al lanzar. |

### Recuperación frontal (en las vueltas)

| Parámetro | Por defecto | Qué hace |
|---|---|---|
| `recover_back` | `0.5` | Segundos de marcha atrás cuando el coche va a chocar de frente. El servo va todo a la derecha en antihorario y todo a la izquierda en horario. Cada repetición en el mismo sitio suma 0,6 s. |

### Motor

| Parámetro | Por defecto | Qué hace |
|---|---|---|
| `motor_min_rps` | `1.5` con `start_lot`, si no el de `robot.yaml` (0) | Mínimo de rps mientras se mueve: el motor 310 no arranca el coche a ~1 rps. |
| `motor_max_rps` | `auto` (4 para el v3) | Tope de rps (~0,9 m/s). |
| `motor_kick_rps`, `motor_pulse_rps` | 1,8 con `start_lot`, si no 0 | Golpe de arranque y pulsos por debajo de esa velocidad (solo en `LOT_EXIT` con `start_lot`). 0 = apagado. |
| `motor_kick_s`, `motor_pulse_period_s` | `0.2`, `0.5` | Duración del golpe y periodo de los pulsos. |

### Arranque y sensores

| Parámetro | Por defecto | Qué hace |
|---|---|---|
| `button` | `true` | `false`: arranca solo a los `start_delay` s, sin botón. |
| `start_delay` | `auto` (20 s para el v3) | Espera antes del arranque automático (`button:=false`). |
| `gpio_button`, `gpio_button_pin` | `true`, `16` | El pulsador del GPIO del Pi. También arranca la ronda (además del botón de la placa). |
| `rear_range` | `true` | El sensor de distancia trasero (VL53L0X). `false` lo quita. |
| `camera_driver` | `camera_publisher` | `picam` para la cámara IMX708 del coche (pone también `strategy_params` con `config/car_picam.yaml`). |
| `strategy_params` | `auto` | Archivo YAML con más parámetros del v3 (abajo). |

## De la estrategia (`settings.py`, también por `strategy_params`)

Los de la salida y la recuperación (`start_lot`, `lot_exit_speed`, `recover_back`) son estos mismos. Otros que se
tocan en el coche:

| Parámetro | Por defecto | Qué hace |
|---|---|---|
| `steer_limit` | `0.307` (`car_picam.yaml`: `0.3054`, 17,5°) | Giro máximo de las ruedas, en rad. |
| `cruise_speed`, `pillar_speed`, `finish_speed` | `0.28`, `0.30`, `0.15` | m/s en carril libre, junto a un pilar y al final. |
| `park` | `0` | `1`: aparca al final (con `start_lot` no se usa). |

## Fijos en el código

No se pasan al lanzar; se cambian en el código.

| Constante | Valor | Dónde | Qué hace |
|---|---|---|---|
| `RECOVER_LOOK` | 0,18 m | `mission.py` | Distancia a lo largo de la curva que lleva el coche que tiene que estar libre: si algo del frente cae ahí, frena. |
| `RECOVER_NOSE` | 0,05 m | `mission.py` | Solo cuenta la parte delantera del coche: los lados ya no disparan al pasar un pilar de cerca. |
| `RECOVER_CORRIDOR` | medio ancho + 1 cm | `mission.py` | Ancho del pasillo que se mira. |
| `RECOVER_BRAKE` | 0,2 s | `mission.py` | Parada en seco (`/robotek/estop`) antes de retroceder. |
| `RECOVER_REPEAT`, `RECOVER_STRAIGHT` | 0,3 m y 8 s, 0,6 s | `mission.py` | Una recuperación a menos de 0,3 m y 8 s de la anterior es una repetición: suma 0,6 s de marcha atrás (hasta 3). |
| `CAMERA_MAX_AGE` | 0,8 s | `strategy.py` | Una imagen más vieja para el coche (WAIT_SENSOR). LiDAR y giroscopio: 0,35 s. |
| `PRESTEER_S`, `SETTLE_S` | 0,5 s, 0,4 s | `lot_exit.py` | Antes de cada pulso gira las ruedas parado; después, quieto. |
| `REAR_STOP_MM` | 20 mm | `lot_exit.py` | En los pasos hacia atrás, el sensor trasero a menos de esto frena en seco. |
| `WALL_GAP` | 1 cm | `lot_exit.py` | Hueco supuesto entre el coche y el muro en el aparcamiento (el LiDAR no lo ve a menos de 12 cm). |

## Del sensor trasero (`rear_range`)

| Parámetro | Por defecto | Qué hace |
|---|---|---|
| `correct` | `true` | `false`: publica la lectura cruda, sin la tabla de corrección. |
| `table_raw_mm`, `table_mm` | `[70, 75, 77, 83, 96, 138, 231, 476]`, `[10, 20, 30, 40, 50, 85, 170, 420]` | Tabla crudo → mm medida contra una pared. Fuera de la tabla, pendiente 1. |
