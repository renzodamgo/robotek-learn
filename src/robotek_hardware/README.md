# robotek_hardware

Solo para el **coche real** (Raspberry Pi). Conecta tu estrategia con el hardware del coche de Robotek:
tu nodo `drive` lee `/scan`, `/imu`, `/camera/image` y `/start` y publica `Twist` en `/cmd_vel_raw`, igual que en
el simulador. Estos nodos traducen esos nombres a la placa HiWonder RRC Lite (servo, motor, IMU, botón, LEDs) y al
LiDAR LD19, y al revés. Tu código no cambia entre el simulador y el coche.

```
LiDAR LD19  -> /scan (driver)        -> scan_adapter -> /robotek/scan  (0 rad al frente, +-110 grados) -> tu /scan
IMU de la RRC Lite -> /rrc/imu_raw   -> imu_adapter  -> /robotek/imu   (x adelante, y izquierda, z arriba) -> tu /imu
Cámara USB -> /usb_cam/image_raw                                                                         -> tu /camera/image
Botón de la RRC Lite -> start_button -> /start (primera pulsación), /robotek/stop (las siguientes)
tu /cmd_vel_raw -> base_adapter -> servo (microsegundos, /rrc/acker_servo/set_state) + motor (rps, /rrc/set_motor, puerto M2)
```

Se lanza todo con `ros2 launch robotek_course car.launch.py strategy:=my_open` (ver
[Lanzar](#lanzar-en-el-coche)). Para dejar el Pi listo: [Preparar el Pi](#preparar-el-pi-un-solo-comando).

## Quiero que el coche haga la ronda ya

En la rama `main` está el andamiaje (simulador, nodos del coche, drivers); tu estrategia, el paquete `src/my_open`, la
escribes tú en la guía (git la ignora). La estrategia **terminada** de los pasos 2 a 9 se instala con
`tools/get-reference.sh` (la extrae de la etiqueta `step-9`). Con un Raspberry Pi (Ubuntu 24.04) y el coche **sobre soportes**:

```bash
git clone https://github.com/renzodamgo/robotek-learn.git && cd robotek-learn
bash tools/setup-pi.sh --with-reference        # ROS, dependencias, udev, grupos, estrategia del paso 9 y colcon build
source install/setup.bash && ros2 launch robotek_course open_challenge.launch.py
```

Si el script te añadió a grupos nuevos (`dialout`, `video`, `gpio`), cierra la sesión y vuelve a entrar antes del último comando.
Espera a que los LED de la placa se pongan **verdes** y pulsa el **botón de la placa una vez**: es el Start (`/start`) y el
coche hace las tres vueltas de la ronda Open; una segunda pulsación lo detiene (`/robotek/stop`). Ver los límites de
velocidad más abajo: empieza con `motor_max_rps:=2.5` (~0,57 m/s); el Open v5 del equipo corre con 6,7. Para tu propia estrategia: crea tu paquete como en
el paso 0 de la guía y lanza `car.launch.py strategy:=<tu paquete>`. Sin `src/my_open`, el launch lo dice y sale con
código 1; `bash tools/get-reference.sh --force` reemplaza (con copia de seguridad `src/my_open.bak-FECHA`) tu paquete por el de referencia.

## Nodos

| Nodo | Qué hace |
|---|---|
| `base_adapter` | `/cmd_vel_raw` (`Twist`) a servo de la RRC Lite (µs, tabla de ruedas medida, tope de 25 grados) y motor (rps de la placa en `/rrc/set_motor`, `motor_backend: rrc`). Limita aceleración (`accel_limit`) y deceleración (`decel_limit`), para el motor si dejan de llegar comandos (`watchdog`) y, al parar después de moverse, frena reiniciando la placa (`brake_reset`, ver Motor) |
| `motor_driver` | Solo con `motor_backend: tb6612` (el montaje anterior al 1 nov 2026): puente TB6612FNG por GPIO (`lgpio`), `/motor_vel` de -100 a 100 %. Con el motor en la RRC Lite arranca pero no mueve nada |
| `scan_adapter` | El scan del driver del LD19 (su cero apunta hacia atrás) a `/robotek/scan` con 0 rad al frente. Descarta lo que el coche no ve (más allá de +-110 grados los rayos pegan en el propio coche) |
| `imu_adapter` | IMU de la placa a ejes del coche (x adelante, y izquierda, z arriba). Las estrategias integran el giroscopio z, así que los ejes tienen que estar bien |
| `start_button` | Botón de la placa: primera pulsación publica `/start` (y pita), cualquier otra publica `/robotek/stop` y el coche queda parado hasta lanzar de nuevo |
| `gpio_button` | Pulsador en el GPIO 16 del Pi (la otra pata a GND, pull-up interno) → `/robotek/gpio_button` (Bool: `true` al pulsar, `false` al soltar) y una línea en el log. De momento solo avisa: no arranca ni para la ronda. `gpio_button:=false` lo quita; `gpio_button_pin:=N` cambia el pin |
| `rear_range` | Sensor de distancia VL53L0X en el I2C del Pi (0x29, bus 1) → `/robotek/rear_range` (Range, metros) y `/robotek/rear_range_mm` (Int32, milímetros; `-1` si no ve nada). Arranca con el launch del coche (`rear_range:=false` lo quita); solo avisa. Arranca el sensor con la secuencia de la librería de Pololu (ajustes de ST y calibración de referencia) y mide en continuo (~24 Hz). Aun así, el sensor del coche mide de más y no en proporción: 188 mm a 100 mm y 332 mm a 200 mm (libro y regla, 9 nov 2026); el nodo publica (crudo − `raw_offset_mm`) / `raw_scale` (44 mm y 1,44, la recta por esos dos puntos; con 0 y 1 publica el crudo). Comprobado solo en esos dos puntos: a 200 mm publica 199–200 y, otra vez a 100 mm, 95–99. Si el sensor deja de responder 0,5 s o da 10 errores de I2C seguidos (le pasó en el coche: se reinició y volvió con sus registros de fábrica), el nodo lo vuelve a abrir y repite todo el arranque en menos de 5 s. |
| `battery_guard` | Si la batería de la placa (`/rrc/battery`) baja de 6,8 V durante 5 lecturas seguidas, avisa en rojo parpadeante (`status_leds`). No para el coche |
| `status_leds` | Los dos LED RGB de la placa: verde = listo para el botón, azul = pulsado (la ronda corre, terminó o se paró), rojo parpadeando = batería baja. Solo lee `/robotek/state` y `/robotek/battery_low` |
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
> el motor por su cuenta: el motor solo se mueve si llega `/cmd_vel_raw` de tu estrategia, y la estrategia espera el
> Start (la pulsación del botón de la placa). Aun así: `base_adapter` limita la aceleración (`accel_limit`, 1 m/s²), no
> pide más de `motor_max_rps`, un `watchdog` de 0,3 s lo para si dejan de llegar comandos, la dirección se limita a
> `wheel_limit_deg` (19°) y una segunda pulsación del botón (`/robotek/stop`) lo deja parado hasta lanzar de nuevo. Un
> `/robotek/estop` (`Bool` verdadero) también lo para. Un 0 para el motor (desde el 8 nov 2026, sin mandar el tipo de
> motor: sección Motor). Si aun así sigue girando, apaga la placa con su interruptor.

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
   suscriptor `/rrc/acker_servo/set_state` que mueve el servo PWM de la dirección (`acker_channel`; en el coche del
   equipo, el canal 1 = conector J4).
3. El puerto serie es el parámetro `port` (`ros2 launch rrc_lite_driver controller.launch.py port:=/dev/...`). Vacío
   (por defecto) = `/dev/rrc_lite` (regla de udev) o `/dev/ttyACM0`.
4. Sin la placa enchufada el nodo no se cae con un traceback: avisa y reintenta cada 3 s. Si el USB se cae **durante**
   una corrida, el nodo muere (`TypeError` en `recv_task`) y hay que relanzar: pasó el 4 nov 2026 por ruido eléctrico
   (`disabled by hub (EMI?)` en el kernel).
6. `motor_type` (en `config/controller_params.yaml`): **−1 en el coche del equipo, no se manda** (8 nov 2026; con el 2
   el motor se embalaba, ver Motor). Si fuera ≥ 0, se mandaría al arrancar y cada 3 s con los motores parados.
7. `/rrc/reset` (`std_msgs/Empty`): reinicia la placa (la línea RTS de su chip USB va a RESET), el motor se para, y
   2,5 s después la vuelve a preparar (recepción, tipo de motor si lo hay, motores a 0). Era el freno de `base_adapter`
   (`brake_reset`), apagado desde el 8 nov.
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
* **Motor**: un motor 310 con encoder (JGA27-310R, reductora 20:1, 450 rpm) en el puerto **M2** de la RRC Lite
  (`rrc_motor_port: 2`), sin mandar el tipo de motor (8 nov 2026). Engranajes motor:eje 54:30. Antes iba un motor sin encoder por un
  puente TB6612 en los GPIO del Pi (`motor_backend: tb6612`, `motor_driver`).
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
ros2 launch robotek_course open_challenge.launch.py      # = car.launch.py strategy:=my_open
```

Espera a que los LED de la placa se pongan **verdes** (la estrategia espera el botón), pon el coche en la posición de
salida y pulsa el botón una vez: es el Start (`/start`) y los LED pasan a azul. Pulsa otra vez para parar: el motor
queda en 0 hasta que lances de nuevo. Con las ruedas en el aire se pueden probar los nodos antes de poner el coche en
la pista. `ros2 launch robotek_course car.launch.py --show-args` muestra las opciones (`battery_guard:=false`,
`status_leds:=false`, `motor_max_rps:=2.5`, `strategy_params:=<archivo.yaml>`, `camera:=false`, `camera_driver:=usb_cam`, `video_device:=/dev/v4l/by-id/...`,
`lidar_package:=''`...).

Si falta algún driver o algún aparato, el launch lo dice y sigue: se puede probar en un portátil sin hardware. Cada
nodo avisa con un mensaje claro y reintenta, sin mover nada: la placa («RRC Lite board not found»), la cámara («Camara no encontrada»), el LiDAR (el driver del LD19 lo dice y termina; relanza
cuando esté enchufado).

### Motor: velocidad y freno

La estrategia de referencia pide `CRUISE_SPEED = 1.28` m/s (`drive.py` lo publica en `/cmd_vel_raw`: `linear.x` en m/s y
`angular.z = v·tan(dirección)/0,1174` en rad/s). `base_adapter` lo convierte así:

* **Motor**: `rps = v·motor_rps_per_mps`, con el signo `rrc_motor_sign` y recortado a `motor_max_rps`.
  - Configuración desde el 8 nov 2026: motor 310 (JGA27-310R 20:1) en **M2**, **sin mandar el tipo de motor**
    (`motor_type: -1`), engranajes motor:eje 54:30, `motor_rps_per_mps` **4,4**, `rrc_motor_sign` −1 (`rrc_driver`
    invierte el signo y la placa avanza con rps positivos).
  - Medido con el LiDAR: 0,15 / 0,30 / 0,50 m/s pedidos dan 0,16 / 0,28 / 0,45 con el factor 4,0, así que el factor
    pasó a 4,4.
  - Con 30:54 (el engranaje pequeño en el motor), el factor es 13 y la máxima ~0,65 m/s.
  - El Open del equipo corre con `motor_max_rps:=6.7`: no recorta su 1,28 m/s; ~0,87 m/s de media en la ronda.
* **Parada**: un 0 normal. El coche se desliza 8–35 cm (54:30) y se queda quieto.
  - `brake_reset` (reiniciar la placa por `/rrc/reset`) y `brake_rps` están apagados.
  - Eran parches del 1–4 nov, cuando se mandaba el **tipo 2**. El firmware de la RRC Lite (código de Hiwonder,
    `motors_param.h`) tiene para el JGA27 ganancias PID **negativas**, y el encoder de nuestro motor cuenta al revés
    que el de Hiwonder. Con el tipo 2, el control empujaba en el sentido equivocado: PWM a tope con cualquier consigna,
    no paraba con 0 y a veces iba hacia atrás.
  - Sin mandar el tipo, la placa usa el suyo de fábrica y controla bien.
  - Detalle y medidas: WRO-Strategies `docs/robot-real-pi/obstacle-runs/20261108/`.
* **Poca velocidad** (54:30): el motor tiene poca fuerza; con todo el giro, desde parado, no arranca por debajo de
  ~1 rps (~0,23 m/s). Tres opciones del adaptador, apagadas por defecto (el Open no cambia), como argumentos de
  `car.launch.py`:
  - `motor_min_rps`: toda orden de moverse manda al menos estos rps. Con 1,1–1,3 arranca siempre, pero lo lento va más
    rápido de lo pedido (0,06 m/s → ~0,25 m/s).
  - `motor_kick_rps` / `motor_kick_s`: arranque a más rps (p. ej. 2 durante 0,2 s), solo si el coche llevaba parado al
    menos 0,5 s.
  - `motor_pulse_rps` / `motor_pulse_period_s`: por debajo de `motor_pulse_rps`, alterna ese valor y 0 para que la
    media sea lo pedido. En el suelo, en recto, con arranque 2 rps y pulsos 0,66 rps: 0,06 / 0,10 / 0,20 m/s pedidos
    dan 0,046 / 0,108 / 0,233 m/s. Con todo el giro, ni pulsos de 1 rps cada 2 s movieron el coche (8 nov 2026).
* **Aceleración**: `accel_limit` 1 m/s² (arranque suave). Al parar sigue `decel_limit` (0,6 m/s²).
* **Dirección**: el ángulo de rueda del Twist pasa por la tabla `wheel_table_*` al servo (µs) y se limita a `wheel_limit_deg`, 19°. Medida el 1 nov 2026
  (`servo_measured: true`). Las estrategias piden menos: `STEER_LIMIT` 17,6° (Open) y `steer_limit` 17,5° en el
  archivo del Obstacle en el coche. En cuanto la estrategia publica (aunque esté parada esperando el Start) el servo va
  al centro.
  - **Fallo intermitente, sin resolver:** a veces la placa recibe todo el giro y el coche sigue recto (giroscopio ~0).
    Pasó en dos rondas del Open del 4 nov, en dos del 8 nov y en una del Obstacle. CPU y frecuencias normales; el servo
    funciona cuando se prueba solo.
* **Reinicios de la placa**: el giroscopio se corta ~2,5 s («Board data back after … s»), sobre todo con el coche
  atascado empujando. `rrc_driver` le reenvía los offsets del servo; sin tipo de motor, no hay nada más que reenviar.
* **Primera vez**: con las ruedas en el aire y `motor_max_rps:=2`. Comprueba que el motor gira hacia adelante, que la
  dirección sigue al servo y que la segunda pulsación del botón lo para.

### El camino del botón al motor

`botón de la placa → rrc_driver (/rrc/button) → start_button → /start (Bool, QoS por defecto) → nodo strategy (drive.py)`.
`strategy` imprime `Start` y su estado (`/robotek/state`, transient local, lo leen `status_leds`) pasa de `WAIT_START` a
`CALIBRATE` y `DETECT_DIRECTION`. Los sensores llegan por las remapeos del launch (`/scan` → `/robotek/scan`, `/imu` →
`/robotek/imu`, `/camera/image` → `/usb_cam/image_raw`) y el mando vuelve por `/cmd_vel_raw → base_adapter → servo
(/rrc/acker_servo/set_state) + /rrc/set_motor → rrc_driver`. La segunda pulsación publica `/robotek/stop`: `strategy` pasa a
`OPERATOR_STOP` y `base_adapter` deja el motor en 0. El nodo usa el reloj del nodo (en el coche
`use_sim_time` es falso, es reloj de pared). Ojo: en el Pi el temporizador de `CONTROL_RATE` = 50 Hz va a ~35–40 Hz; si integras el giroscopio con 1/50 s fijo el rumbo sale corto (el Open v5 usa el tiempo real entre ciclos). Si el LiDAR (0,35 s)
o el giroscopio (0,2 s) dejan de llegar, el log dice `A sensor went quiet: braking until it is back` y la velocidad es 0.

## Calibración

Todos los valores están en [`config/robot.yaml`](config/robot.yaml). Son los del coche del equipo; **mide los
tuyos** antes de confiar en ellos:

* **Servo de dirección** (`base_adapter`): centro (`servo_center_us`), ganancias (`servo_gain_pos`, `servo_gain_neg`),
  sentido (`servo_sign`) y la tabla `wheel_table_*` (ángulo real de la rueda por cada ángulo de servo, medido
  dando círculos). Mientras `servo_measured: false` el nodo avisa en cada arranque: cuando lo midas, ponlo en
  `true`. Prueba el servo solo con `ros2 run robotek_hardware servo_steer` y publicando en `/steer`.
* **Motor** (`motor_backend: rrc`, `rrc_motor_port`, `rrc_motor_sign`, `motor_rps_per_mps`, `motor_max_rps`, `brake_*`)
  y `motor_type` en `src/rrc_lite_driver/config/controller_params.yaml` (−1: no mandar el tipo; con el 310 y el tipo 2,
  el control de la placa va al revés). Si cambias de puerto o de tipo, vuelve a comprobar el sentido. Mide la velocidad con el LiDAR (distancia a una pared antes y después) con pulsos cortos.
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
