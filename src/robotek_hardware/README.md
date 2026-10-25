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
[Lanzar](#lanzar-en-el-coche)). Para dejar el Pi listo: [Preparar el Pi](#preparar-el-pi-un-solo-comando).

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
| `camera_publisher` | Cámara USB (YUYV 640x480, 15 fps) a `/usb_cam/image_raw`; busca la cámara sola y, si no está o se desconecta, avisa y reintenta (también publica el JPEG `.../compressed` solo si alguien lo pide) |
| `servo_steer` | Prueba en banco: un ángulo (`Float32`, rad, + izquierda) mueve **solo el servo**, nunca el motor |

La lógica pura (conversiones de servo y motor, límites de aceleración, ejes de la IMU, colores de los LED) está en
`mapping.py` y `status_colors.py` y se prueba sin hardware.

## Preparar el Pi (un solo comando)

El coche del equipo es un Raspberry Pi 5 con Ubuntu 24.04 y ROS 2 Jazzy. En un Pi con Ubuntu 24.04 (arm64) y
conexión a internet:

```bash
git clone https://github.com/renzodamgo/robotek-learn.git
cd robotek-learn
bash tools/setup-pi.sh            # instala, configura y compila; pide sudo cuando hace falta
bash tools/setup-pi.sh --check    # después: solo comprueba (placa, LiDAR, cámara, grupos); no cambia nada
```

`tools/setup-pi.sh` se puede repetir (salta lo que ya está hecho) y `--dry-run` muestra los comandos sin ejecutarlos.
Hace, en orden:

1. comprueba que es Ubuntu 24.04;
2. instala ROS 2 Jazzy **base** (sin Gazebo ni RViz: `tools/install-ubuntu.sh --no-gazebo`) y los paquetes apt del coche
   (`ros-jazzy-cv-bridge`, `robot-state-publisher`, `xacro`, `tf2-ros`, `python3-serial`, `python3-lgpio`,
   `python3-opencv`, `v4l-utils`...);
3. instala las reglas de udev `tools/99-robotek.rules` (nombres estables `/dev/rrc_lite` y `/dev/ldlidar`, permisos) y
   te añade a los grupos `dialout`, `video` y `gpio` (cierra la sesión y vuelve a entrar);
4. compila todo: `colcon build --symlink-install`. Tu estrategia `src/my_open` se crea como dice el README de la
   raíz (`ros2 pkg create ...`); si falta, el script lo avisa y compila el resto.

Todo lo que necesita ese comando (dependencias, drivers, reglas) está descrito aquí abajo por si algo falla.

> **Seguridad: la primera vez, el coche sobre soportes (ruedas en el aire).** Ni `setup-pi.sh` ni el launch mueven
> el motor por su cuenta: el motor solo se mueve si llega `/motor_vel`, que `base_adapter` calcula a partir de
> `/cmd_vel_raw` de tu estrategia, y la estrategia espera el Start (la pulsación del botón de la placa). Aun así:
> `base_adapter` limita la aceleración (`accel_limit`, 1 m/s²) y la deceleración (`decel_limit`), el motor no pasa de
> `motor_max_pct` (50 %), un `watchdog` de 0,3 s lo para si dejan de llegar comandos, la dirección se limita a
> ±25° y una segunda pulsación del botón (`/robotek/stop`) lo deja parado hasta lanzar de nuevo. Un
> `/robotek/estop` (`Bool` verdadero) lo corta de golpe.

## Drivers de terceros (incluidos en este repositorio)

El coche necesita cuatro piezas que **no escribió este repositorio**. Todas vienen en `src/`, así que `git clone` +
`colcon build` basta; esto es lo que hay de cada una.

| Pieza | Para qué sirve | Origen | Licencia |
|---|---|---|---|
| `src/ldlidar_stl_ros2` | Driver del LiDAR LDROBOT LD19: publica `/scan` | [ldrobotSensorTeam/ldlidar_stl_ros2](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2), commit `bf668a8` (8 may 2023) | MIT (Copyright 2020 SHENZHEN LDROBOT CO., LTD.). `LICENSE` y avisos intactos; solo los dos arreglos de compilación de `patches/ldlidar_stl_ros2-build.patch`, ya aplicados (ver su `NOTICE-ROBOTEK.md`) |
| `src/rrc_lite_driver` y `src/rrc_lite_msgs` | Driver de la placa RRC Lite (IMU, botón, batería, LEDs, servo de dirección): publica `/rrc/imu_raw`, `/rrc/button`, `/rrc/battery` y recibe `/rrc/acker_servo/set_state`, `/rrc/set_rgb`, `/rrc/set_buzzer` | Basado en `driver/ros_robot_controller` y `driver/ros_robot_controller_msgs` de [Hiwonder/MentorPi](https://github.com/Hiwonder/MentorPi), commit `fb6d9969` (16 ene 2026) | El código original **no declara licencia** (`TODO: License declaration` en sus `package.xml`, sin archivo LICENSE). Se incluye con el permiso de Hiwonder, confirmado por el autor del repositorio; los cambios del equipo Robotek están bajo MIT. Ver el `NOTICE.md` de cada carpeta |
| `camera_publisher` | Cámara USB: publica `/usb_cam/image_raw` | Código del equipo, nodo de este paquete (busca la cámara solo y reintenta si no está) | MIT |
| `usb_cam` (opcional) | La misma cámara con el paquete de ROS | `ros-jazzy-usb-cam` | la del paquete (`apt show ros-jazzy-usb-cam`). No se incluye: `sudo apt install ros-jazzy-usb-cam` y `camera_driver:=usb_cam` |

Cambios del equipo respecto a MentorPi (solo se incluye lo que necesita el coche real):

1. Nombres: `ros_robot_controller` pasa a `rrc_lite_driver` (nodo `rrc_driver`), `ros_robot_controller_msgs` a
   `rrc_lite_msgs` y los tópicos `~/imu_raw`, `~/button`... a `/rrc/imu_raw`, `/rrc/button`... (los que escuchan
   estos nodos).
2. Un mensaje nuevo, `SetAckerServoState` (`float64 duration`, `uint16 position` en microsegundos), y un
   suscriptor `/rrc/acker_servo/set_state` que mueve el **servo PWM 2** de la placa: el servo de dirección va
   conectado al puerto 2.
3. El puerto serie es el parámetro `port` (`ros2 launch rrc_lite_driver controller.launch.py port:=/dev/...`). Vacío
   (por defecto) = `/dev/rrc_lite` (regla de udev) o `/dev/ttyACM0`.
4. Sin la placa enchufada el nodo no se cae con un traceback: avisa y reintenta cada 3 s. Si el USB se cae durante
   una corrida, el puerto se reabre solo.
5. Los desplazamientos de los servos (`config/servo_config.yaml`) se leen del paquete instalado, no de una ruta fija.

Qué hace falta para cada cosa:

* **Solo conducir el motor y la dirección** (sin sensores): `rrc_lite_driver` (servo) y este paquete.
* **Ver el LiDAR**: `ldlidar_stl_ros2`.
* **Botón de Start, IMU, LED de estado**: `rrc_lite_driver`.
* **Cámara** (color de las líneas): `camera_publisher` (o `usb_cam`, o cualquier nodo que publique
  `sensor_msgs/Image` y lo pongas en `camera_topic:=`).

## Hardware: nombres de dispositivo y permisos

* **Placa RRC Lite**: aparece como `/dev/ttyACM0` (USB `1a86:55d4`); la regla de udev le añade el nombre estable
  `/dev/rrc_lite`. Tu usuario tiene que estar en el grupo `dialout`. Comprueba con `ls -l /dev/serial/by-id/`.
* **LiDAR LD19**: adaptador CP210x (USB `10c4:ea60`), aparece como `/dev/ttyUSB0` a 230400 baudios, que es lo que
  abre el `ld19.launch.py` del driver; la regla de udev añade `/dev/ldlidar`. Si en tu Pi es otro puerto, arranca el
  driver tú y lanza con `lidar_package:=''`.
* **Motor**: el TB6612 va por GPIO del Pi 5 (`gpiochip` 4; pines PWM 12, AIN1 5, AIN2 6, STBY 13 en
  `config/robot.yaml`). Hace falta `python3-lgpio` y el grupo `gpio`. Sin `lgpio` o sin acceso a `/dev/gpiochip4` el
  nodo arranca, avisa («GPIO chip 4 not usable» o «lgpio missing») y reintenta, sin mover nada.
* **Cámara**: `camera_publisher` busca la primera `/dev/v4l/by-id/*-video-index0` (en el Pi 5 hay muchos
  `/dev/videoN` y el número cambia: mejor la ruta estable; se fija con `video_device:=` en el launch). YUYV 640x480.
  Sin cámara avisa y reintenta cada segundo.

Las reglas de udev (`tools/99-robotek.rules`) usan solo identificadores USB confirmados: `1a86:55d4` (placa, de la
regla de Hiwonder y de las mediciones del equipo) y `10c4:ea60` (LiDAR, de la regla de LDROBOT). La regla del grupo
`gpio` para `/dev/gpiochip*` es la habitual de Raspberry Pi y no se ha comprobado en un Pi limpio.

## Lanzar en el coche

```bash
cd robotek-learn                   # la carpeta donde clonaste el repositorio
source install/setup.bash
ros2 launch robotek_course car.launch.py strategy:=my_open
```

Espera a que los LED de la placa se pongan **verdes** (LiDAR, IMU y cámara reciben datos), pon el coche en la
posición de salida y pulsa el botón una vez: es el Start (`/start`). Pulsa otra vez para parar: el motor queda
en 0 hasta que lances de nuevo. Con las ruedas en el aire se pueden probar los nodos antes de poner el coche en
la pista. `ros2 launch robotek_course car.launch.py --show-args` muestra las opciones (`battery_guard:=false`,
`status_leds:=false`, `camera:=false`, `camera_driver:=usb_cam`, `video_device:=/dev/v4l/by-id/...`,
`lidar_package:=''`...).

Si falta algún driver o algún aparato, el launch lo dice y sigue: se puede probar en un portátil sin hardware. Cada
nodo avisa con un mensaje claro y reintenta, sin mover nada: la placa («RRC Lite board not found»), el motor
(«GPIO chip 4 not usable»), la cámara («Camara no encontrada»), el LiDAR (el driver del LD19 lo dice y termina; relanza
cuando esté enchufado).

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

Prueban las conversiones de `mapping.py` y los colores de `status_colors.py`, sin placa y casi todas sin ROS (la prueba de `imu_axes` necesita el entorno de ROS cargado y se salta si no está). Con el entorno de
ROS cargado puede hacer falta `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` para que `pytest` no cargue los plugins de
`launch_testing`.

## Licencia

MIT (Renzo Damian) para el código de este paquete, `camera_publisher` incluido y los cambios del equipo en
`rrc_lite_driver` / `rrc_lite_msgs`. `src/ldlidar_stl_ros2/` es una copia del driver de LDROBOT, también MIT (su
`LICENSE` y sus avisos no se tocaron). El código de Hiwonder en `rrc_lite_*` no declara licencia: se incluye con el
permiso de Hiwonder, confirmado por el autor de este repositorio.
