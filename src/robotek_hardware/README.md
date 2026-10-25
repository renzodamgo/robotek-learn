# robotek_hardware

Solo para el **coche real** (Raspberry Pi). Conecta tu estrategia con el hardware del coche de Robotek:
tu nodo `drive` lee `/scan`, `/imu`, `/camera/image` y `/start` y publica `Twist` en `/cmd_vel_raw`, igual que en
el simulador. Estos nodos traducen esos nombres a la placa HiWonder RRC Lite (servo, IMU, botón, LEDs), al motor
(puente TB6612) y al LiDAR LD19, y al revés. Tu código no cambia entre el simulador y el coche.

```
LiDAR LD19  -> /scan (driver)        -> scan_adapter -> /robotek/scan  (0 rad al frente, +-110 grados) -> tu /scan
IMU de la RRC Lite -> /rrc/imu_raw   -> imu_adapter  -> /robotek/imu   (x adelante, y izquierda, z arriba) -> tu /imu
Cámara USB -> /usb_cam/image_raw                                                                         -> tu /camera/image
Botón de la RRC Lite -> start_button -> /start (primera pulsación), /robotek/stop (las siguientes)
tu /cmd_vel_raw -> base_adapter -> servo (microsegundos, /rrc/acker_servo/set_state) + /motor_vel -> motor_driver (TB6612)
```

Se lanza todo con `ros2 launch robotek_course car.launch.py strategy:=my_open` (ver
[Lanzar](#lanzar-en-el-coche)).

## Nodos

| Nodo | Qué hace |
|---|---|
| `base_adapter` | `/cmd_vel_raw` (`Twist`) a servo de la RRC Lite (µs, tabla de ruedas medida, tope de 25 grados) y motor (% con signo, calibrado). Limita aceleración (`accel_limit`) y deceleración (`decel_limit`) y para el motor si dejan de llegar comandos (`watchdog`) |
| `motor_driver` | Puente TB6612FNG por GPIO (`lgpio`) con marcha atrás: `/motor_vel` de -100 a 100 %, pone STBY en alto, parada suave en rampa (`stop_ramp_s`), parada de emergencia `/robotek/estop` |
| `scan_adapter` | El scan del driver del LD19 (su cero apunta hacia atrás) a `/robotek/scan` con 0 rad al frente. Descarta lo que el coche no ve (más allá de +-110 grados los rayos pegan en el propio coche) |
| `imu_adapter` | IMU de la placa a ejes del coche (x adelante, y izquierda, z arriba). Las estrategias integran el giroscopio z, así que los ejes tienen que estar bien |
| `start_button` | Botón de la placa: primera pulsación publica `/start` (y pita), cualquier otra publica `/robotek/stop` y el coche queda parado hasta lanzar de nuevo |
| `battery_guard` | Si la batería de la placa (`/rrc/battery`) baja de 6,8 V durante 5 lecturas seguidas, avisa en rojo parpadeante (`status_leds`). No para el coche |
| `status_leds` | Los dos LED RGB de la placa muestran el estado: verde = listo para Start, rojo + color = falta un sensor (azul LiDAR, magenta IMU, amarillo cámara), azul = calibrando, apagado = en ronda, verde parpadeante = terminó |
| `imu_axes` | Herramienta de calibración: con el coche quieto y un empujón hacia adelante imprime los ejes de `imu_adapter` |
| `servo_steer` | Prueba en banco: un ángulo (`Float32`, rad, + izquierda) mueve **solo el servo**, nunca el motor |

La lógica pura (conversiones de servo y motor, límites de aceleración, ejes de la IMU, colores de los LED) está en
`mapping.py` y `status_colors.py` y se prueba sin hardware.

## Drivers de terceros (no están en este repositorio)

El coche necesita tres programas que **no son de este repositorio** y que no se copiaron aquí. Cada uno se
obtiene de su origen.

| Pieza | Para qué sirve | Origen | Licencia | Cómo obtenerla |
|---|---|---|---|---|
| `rrc_lite_driver` y `rrc_lite_msgs` | Driver de la placa RRC Lite (IMU, botón, batería, LEDs, servo de dirección): publica `/rrc/imu_raw`, `/rrc/button`, `/rrc/battery` y recibe `/rrc/acker_servo/set_state`, `/rrc/set_rgb`, `/rrc/set_buzzer` | Derivado de `driver/ros_robot_controller` y `driver/ros_robot_controller_msgs` de [Hiwonder/MentorPi](https://github.com/Hiwonder/MentorPi), commit `fb6d9969` (16 ene 2026) | **Sin licencia declarada** (`TODO: License declaration` en sus `package.xml` y sin archivo LICENSE): por eso no se redistribuye. Se clona de Hiwonder y el script `tools/make_rrc_lite.py` aplica nuestros cambios en tu copia | Ver [Preparar el Pi](#preparar-el-pi) |
| `ldlidar_stl_ros2` | Driver del LiDAR LDROBOT LD19: publica `/scan` | [ldrobotSensorTeam/ldlidar_stl_ros2](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2), commit `bf668a8` (8 may 2023) | MIT (Copyright 2020 SHENZHEN LDROBOT CO., LTD.) | `git clone` + el parche `patches/ldlidar_stl_ros2-build.patch` si no compila |
| `usb_cam` | Cámara USB: publica `/usb_cam/image_raw` | Paquete de ROS `ros-jazzy-usb-cam` | la del paquete (ver `apt show ros-jazzy-usb-cam`) | `sudo apt install ros-jazzy-usb-cam` |

Qué hace falta para cada cosa:

* **Solo conducir el motor y la dirección** (sin sensores): `rrc_lite_driver` (servo) y este paquete.
* **Ver el LiDAR**: `ldlidar_stl_ros2`.
* **Botón de Start, IMU, LED de estado**: `rrc_lite_driver`.
* **Cámara** (color de las líneas): `usb_cam` o cualquier nodo que publique `sensor_msgs/Image` y lo pongas en
  `camera_topic:=`.

Lo que cambia el script `make_rrc_lite.py` respecto a MentorPi (son pocas líneas, nuestras):

1. Nombres: `ros_robot_controller` pasa a `rrc_lite_driver` (nodo `rrc_driver`), `ros_robot_controller_msgs` a
   `rrc_lite_msgs` y los tópicos `~/imu_raw`, `~/button`... a `/rrc/imu_raw`, `/rrc/button`... (los que escuchan
   estos nodos).
2. Un mensaje nuevo, `SetAckerServoState` (`float64 duration`, `uint16 position` en microsegundos), y un
   suscriptor `/rrc/acker_servo/set_state` que mueve el **servo PWM 2** de la placa: el servo de dirección va
   conectado al puerto 2.
3. El puerto serie de la placa es `/dev/ttyACM0` (MentorPi espera un enlace `/dev/rrc` que solo crea la imagen de
   Hiwonder).

El driver del coche del equipo además reabre el puerto serie si la conexión USB se cae durante una corrida; el
script no lo incluye (es código nuestro, pero va mezclado con el de Hiwonder). Si el USB se resetea,
reinicia el launch.

## Preparar el Pi

El coche del equipo es una Raspberry Pi 5 con Ubuntu 24.04 y ROS 2 Jazzy. Estos pasos reproducen su instalación
pero no se han repetido en un Pi limpio: si algo falla, dilo. Con el Pi conectado a internet:

```bash
sudo apt install ros-jazzy-usb-cam ros-jazzy-robot-state-publisher ros-jazzy-xacro ros-jazzy-cv-bridge \
                 python3-serial python3-lgpio python3-opencv python3-numpy
mkdir -p ~/robotek_ws/src && cd ~/robotek_ws/src
git clone <este repositorio> .        # robotek_common, robotek_description, robotek_hardware, robotek_course, my_open...

# 1) Driver de la placa: se clona de Hiwonder y se adapta en tu copia
git clone https://github.com/Hiwonder/MentorPi.git ~/MentorPi
git -C ~/MentorPi checkout fb6d9969
python3 robotek_hardware/tools/make_rrc_lite.py ~/MentorPi ~/robotek_ws/src

# 2) Driver del LiDAR LD19
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
git -C ldlidar_stl_ros2 checkout bf668a8
git -C ldlidar_stl_ros2 apply ../robotek_hardware/patches/ldlidar_stl_ros2-build.patch   # solo si no compila

# 3) Compilar. ROBOTEK_ROBOT=1 hace que robotek_hardware dependa de rrc_lite_msgs
cd ~/robotek_ws && source /opt/ros/jazzy/setup.bash
ROBOTEK_ROBOT=1 colcon build --symlink-install --packages-select rrc_lite_msgs rrc_lite_driver ldlidar_stl_ros2 \
    robotek_common robotek_description robotek_hardware robotek_course my_open
source install/setup.bash
```

Detalles del hardware:

* **Placa RRC Lite**: aparece como `/dev/ttyACM0` (USB `1a86:55d4`). Tu usuario tiene que estar en el grupo
  `dialout` (`sudo usermod -aG dialout $USER` y volver a entrar). Comprueba con `ls -l /dev/serial/by-id/`. Si el
  ModemManager la ocupa, Hiwonder trae una regla de udev que lo evita (`rrc_lite_driver/scripts/99-ttyACM0.rules`;
  cambia `GROUP` por tu grupo antes de copiarla a `/etc/udev/rules.d`).
* **LiDAR LD19**: el `ld19.launch.py` de LDROBOT abre `/dev/ttyUSB0` a 230400 baudios. Si en tu Pi es otro,
  edita `port_name` en ese launch o arranca el driver tú y lanza con `lidar_package:=''`.
* **Motor**: el TB6612 va por GPIO del Pi 5 (`gpiochip` 4; pines PWM 12, AIN1 5, AIN2 6, STBY 13 en
  `config/robot.yaml`). Hace falta `python3-lgpio`. Sin `lgpio` el nodo arranca y avisa «motor disabled».
* **Cámara**: en el Pi 5 hay muchos `/dev/videoN` y el número cambia: usa la ruta estable
  `/dev/v4l/by-id/...` en `video_device:=`. Con `usb_cam` el formato es YUYV 640x480. En el coche del equipo la cámara
  la publica otro nodo, no `usb_cam`: si `usb_cam` no te funciona, lanza el que uses con `camera:=false`.

## Lanzar en el coche

```bash
source ~/robotek_ws/install/setup.bash
ros2 launch robotek_course car.launch.py strategy:=my_open
```

Espera a que los LED de la placa se pongan **verdes** (LiDAR, IMU y cámara reciben datos), pon el coche en la
posición de salida y pulsa el botón una vez: es el Start (`/start`). Pulsa otra vez para parar: el motor queda
en 0 hasta que lances de nuevo. Con las ruedas en el aire se pueden probar los nodos antes de poner el coche en
la pista. `ros2 launch robotek_course car.launch.py --show-args` muestra las opciones (`battery_guard:=false`,
`status_leds:=false`, `camera:=false`, `lidar_package:=''`...).

Si falta algún driver, el launch lo dice y sigue: se puede probar en un portátil sin hardware (los nodos avisan
que falta `rrc_lite_msgs` o `lgpio` y no hacen nada con la placa).

## Calibración

Todos los valores están en [`config/robot.yaml`](config/robot.yaml). Son los del coche del equipo; **mide los
tuyos** antes de confiar en ellos:

* **Servo de dirección** (`base_adapter`): centro (`servo_center_us`), ganancias (`servo_gain_pos`, `servo_gain_neg`),
  sentido (`servo_sign`) y la tabla `wheel_table_*` (ángulo real de la rueda por cada ángulo de servo, medido
  dando círculos). Mientras `servo_measured: false` el nodo avisa en cada arranque: cuando lo midas, ponlo en
  `true`. Prueba el servo solo con `ros2 run robotek_hardware servo_steer` y publicando en `/steer`.
* **Motor** (`motor_pct_per_mps`, `motor_min_pct`, `motor_max_pct`): se midió con la batería baja y solo hasta
  30 % (0,32 m/s). Empieza con velocidades bajas y mide de nuevo con la batería cargada.
* **Distancia entre ejes** (`wheelbase`, 0,1174 m): tiene que coincidir con `WHEELBASE` de
  `robotek_common/vehicle.py`, que usa tu estrategia.
* **Ejes de la IMU** (`imu_adapter`): con el Pi y la placa de pie los ejes cambian. Con el coche quieto lanza
  `ros2 run robotek_hardware imu_axes`, da un empujón hacia adelante, copia el bloque que imprime en
  `robot.yaml` y pon `measured: true`.
* **LiDAR** (`scan_adapter`): `angle_offset` (π para el LD19 montado con el cable atrás) y `reverse` si los
  ángulos de tu driver crecen en sentido horario. Compruébalo en RViz: una pared delante debe verse delante.

## Pruebas sin hardware

```bash
cd src/robotek_hardware && python -m pytest
```

Prueban las conversiones de `mapping.py` y los colores de `status_colors.py`, sin ROS ni placa. Con el entorno de
ROS cargado puede hacer falta `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` para que `pytest` no cargue los plugins de
`launch_testing`.

## Licencia

MIT (Renzo Damian). Los drivers de la tabla de arriba tienen la suya y no están en este repositorio.
`patches/ldlidar_stl_ros2-build.patch` modifica código MIT de LDROBOT y se aplica en tu clon.
