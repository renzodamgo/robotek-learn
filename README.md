# robotek-learn

Un coche, un campo y una guía: aprende a programar la estrategia del **reto Open de la WRO 2026 (Future Engineers)**
con ROS 2 Jazzy y Gazebo, escribiendo tú el código paso a paso. Lo que escribes es un nodo de ROS que lee el LiDAR, el
giroscopio y la cámara de un coche simulado con las medidas y los retrasos del coche real, da tres vueltas al campo y
se detiene en la sección de salida. Al final, el mismo nodo corre en el coche de verdad (Raspberry Pi).

La guía completa, con gráficos interactivos y un simulador en el navegador, está en
**https://renzodamian.com/learn/open-strategy**. Este repositorio es el andamiaje.

## Empezar

```bash
git clone https://github.com/renzodamgo/robotek-learn.git
cd robotek-learn
bash tools/install-ubuntu.sh        # Ubuntu 24.04 o WSL2; en macOS ver la guía (RoboStack)
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robotek_course sim.launch.py          # el coche en Gazebo
```

Después crea tu paquete y sigue los pasos de la guía:

```bash
cd src
ros2 pkg create --build-type ament_python my_open --node-name drive \
  --license MIT --maintainer-name "Tu Nombre" --maintainer-email tu@correo.com \
  --description "Mi estrategia del reto Open" \
  --dependencies rclpy geometry_msgs sensor_msgs std_msgs visualization_msgs cv_bridge robotek_common robotek_course
```

## Qué hay aquí

| Carpeta | Para qué |
|---|---|
| `src/robotek_description` | El coche (URDF y mallas) |
| `src/robotek_gazebo` | El campo, el tapete y el puente con ROS 2 |
| `src/robotek_common` | Medidas del coche y odometría con LiDAR |
| `src/robotek_evaluation` | Solo simulación: juez de reglas, modelo del servo y del motor, contactos, Open lab |
| `src/robotek_course` | Lanzadores del simulador y de una ronda, el árbitro, `score`, la clase base de la estrategia |
| `src/robotek_hardware` | Los nodos del coche real (Raspberry Pi) |
| `src/robotek_open` | El Open v5, la estrategia del equipo (copia de WRO-Strategies con los valores del coche) |
| `src/ldlidar_stl_ros2` | Driver del LiDAR LD19 (copia del de LDROBOT, MIT) |
| `src/rrc_lite_driver`, `src/rrc_lite_msgs` | Driver de la placa RRC Lite (basado en Hiwonder MentorPi, con permiso) |
| `tools/setup-pi.sh` | Deja un Raspberry Pi listo para el coche real (un solo comando) |
| `tools/pi-monitor` | App de la barra de menús del Mac: batería, temperatura, CPU, Wi-Fi y ROS del Pi del coche, con historial de caídas |
| `course/tests` | Pruebas de cada paso (`pytest`, sin ROS ni Gazebo) |
| `course/verify_step.py` | Comprobación de cada paso en Gazebo |

Tu código vive en `src/my_open` (git lo ignora en `main`). La rama **`solutions`** tiene un commit y una etiqueta
`step-N` por paso para que compares:

```bash
bash tools/compare-step.sh 2 3     # qué cambia la referencia entre dos pasos
bash tools/compare-step.sh 3        # tu árbol contra la referencia del paso 3
```

## Una ronda

```bash
ros2 launch robotek_course round.launch.py strategy:=my_open gui:=true direction:=ccw
ros2 run robotek_course score                # puntúa de nuevo la última ronda
```

Las rondas se guardan en `~/robotek_runs/`. Los puntos son una **estimación de simulación** con las reglas
publicadas, no una puntuación oficial.

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
velocidad en `src/robotek_hardware/README.md` (con el motor 310, `motor_max_rps:=2.5`). Para tu propia estrategia: crea tu paquete como en
el paso 0 de la guía y lanza `car.launch.py strategy:=<tu paquete>`. Sin `src/my_open`, el launch lo dice y sale con
código 1; `bash tools/get-reference.sh --force` reemplaza (con copia de seguridad `src/my_open.bak-FECHA`) tu paquete por el de referencia.

## Coche real (Raspberry Pi)

En el Raspberry Pi del coche (Ubuntu 24.04, arm64, con internet) basta clonar el repositorio y ejecutar un script:

```bash
git clone https://github.com/renzodamgo/robotek-learn.git
cd robotek-learn
bash tools/setup-pi.sh            # ROS 2 Jazzy base, dependencias, reglas de udev, grupos y colcon build
```

Crea tu paquete `my_open` en `src` (el comando `ros2 pkg create` de arriba) o instala el de referencia con
`bash tools/get-reference.sh` (o `setup-pi.sh --with-reference`), y vuelve a ejecutar el script (salta lo que ya está hecho). Después de cerrar la sesión y volver a entrar (por los grupos nuevos):

```bash
bash tools/setup-pi.sh --check    # placa, LiDAR, cámara, grupos: solo comprueba, no cambia nada
source install/setup.bash
ros2 launch robotek_course open_challenge.launch.py     # = car.launch.py strategy:=my_open
```

### Probar más rápido

`ros2 launch robotek_course open_challenge.launch.py button:=false` arranca solo 12 s después de lanzar
(`start_delay:=...`), sin pulsar el botón; la primera pulsación del botón ya es la parada. Ctrl+C también para el motor.

### Correr el Open v5 (la estrategia del equipo)

El Open v5 está en `src/robotek_open` y corre con el mismo `car.launch.py`, así que usa la calibración actual del coche
(`src/robotek_hardware/config/robot.yaml`): la dirección medida el 1 nov 2026 (centro 1403 µs, ±19°) y el motor 310 en
M1 con el tipo 2 de la placa (`src/rrc_lite_driver/config/controller_params.yaml`). El 4 nov 2026 hizo rondas completas
de 3 vueltas en ~30 s en el campo del equipo. En el Pi:

```bash
cd ~/robotek-learn && git pull && colcon build --symlink-install && source install/setup.bash
bash tools/car-clean.sh                                   # nada de pruebas anteriores
ros2 launch robotek_course car.launch.py strategy:=robotek_open executable:=open_v5 motor_max_rps:=2.5 \
    strategy_params:=$HOME/.config/robotek/open_v5_car.yaml
```

`~/.config/robotek/open_v5_car.yaml` son los ajustes del coche (el simulador usa los valores por defecto de `Params`
en `v5/mission.py`):

```yaml
strategy:
  ros__parameters:
    inner_gap: 0.35            # m a la isla en las rectas
    wall_heading_gain: 0.3     # corrige el rumbo con las paredes (el giroscopio marca ~5 % menos)
    finish_speed: 0.4          # m/s del último tramo
    finish_lead: 0.25          # m: pide parar antes, lo que se pasa el freno
    # laps: 1                  # solo para probar la llegada y el freno
```

Con la cámara IMX708 en el flex (`camera_driver:=picam`), el detector usa otros colores: esta cámara ve el muro negro
azulado (V 40-60), el naranja con H ≈ 7 (la USB lo veía rosado) y el rojo de un pilar con H ≈ 177. En
`~/.config/robotek/open_v5_car_picam.yaml`, los mismos ajustes de arriba más estos (medidos el 4 nov 2026):

```yaml
    vision_blue_h_min: 95
    vision_blue_v_min: 60      # el pie del muro sale azul oscuro (V ≈ 45)
    vision_orange_h_min: 2     # naranja de verdad, no el rojo de un pilar (H ≈ 177)
    vision_orange_h_max: 25
    vision_orange_v_min: 60
    vision_wall_v_max: 70      # el muro llega a V ≈ 60 con esta cámara
    vision_line_contrast: 30   # líneas finas más oscuras que el suelo (desde la salida están a ~1,5 m)
    vision_line_tint: 8        # y su tinte respecto al suelo: azul o naranja
```

Desde la salida, las líneas de la esquina están a ~1,5 m. A 640x360 ocupan 1 px y casi no tienen color, así que
`camera_driver:=picam` usa por defecto 1280x720 a 10 fps. La cámara solo funciona hasta que se elige el sentido
(`picam_active_states:=auto` en el Open): después de Start se para `rpicam-vid` y no gasta batería ni CPU.

Espera el botón de la placa. Usa solo el LiDAR, el giroscopio y la cámara, que decide el sentido (el LiDAR lo
comprueba antes de la primera esquina). La telemetría queda en `~/runs/<fecha>-<hora>-open-v5/`. Prueba sin botón y con
todo grabado: `bash tools/open-debug-run.sh motor_max_rps:=2.5 strategy_params:=$HOME/.config/robotek/open_v5_car.yaml`.

* **Velocidad:** con el tipo 2, ~0,45–0,5 m/s a 2,5 rps y ~0,75 m/s a 4–6 rps (el tope del motor). Corre con
  `motor_max_rps:=2.5`.
* **Freno:** con el motor 310 la placa no se queda en 0. `base_adapter` (`brake_reset: true`) reinicia la placa por
  `/rrc/reset` al parar después de moverse; el coche se para ~17 cm después. Pendiente: revisar el encoder de M1.
* **Luces:** verde = listo para el botón; azul = pulsado; rojo parpadeando = batería baja.

### Que arranque solo al encender el Pi

```bash
bash tools/car-autostart.sh on motor_max_rps:=2.5 strategy_params:=$HOME/.config/robotek/open_v5_car.yaml
bash tools/car-autostart.sh status                # ¿está corriendo?   ... log: su salida
bash tools/car-autostart.sh stop                  # pararlo ahora (p. ej. para lanzar otra cosa a mano)
bash tools/car-autostart.sh start                 # otra ronda: el Open termina en DONE y hay que relanzarlo
bash tools/car-autostart.sh off                   # que no arranque más al encender
```

Es un servicio de usuario de systemd (`robotek-car.service`, sin sudo). Al encender el Pi espera el Wi-Fi (hasta 60 s)
y lanza `car.launch.py` con esos argumentos; la estrategia **espera el botón de la placa**. `button:=false` no se acepta
aquí: el coche saldría solo a los 12 s de cada arranque. `on` otra vez cambia los argumentos
(`~/.config/robotek/autostart-args`). Cada arranque y cada `stop` pasan antes por `tools/car-clean.sh`, que para todos
los nodos del coche que hayan quedado de un launch anterior y deja los motores a 0 (el 4 nov 2026, nodos sobrantes
pusieron dos estrategias a la vez y frenaron el Open hasta chocar): úsalo también a mano antes de cualquier prueba.
Cada ronda queda en `~/runs/<fecha>-<hora>-autostart/`: `launch.log`, `kernel.log` (cortes del USB de la placa) y un
rosbag ligero que empieza al pulsar Start. El driver reenvía el tipo de motor cada 3 s mientras el coche está parado:
el interruptor apaga la placa y no el Pi, y la placa vuelve al tipo por defecto (sin fuerza). Tras un `git pull` y
`colcon build`, el código nuevo entra en el siguiente arranque (o con `stop` y `start`).

### Conducir el coche con el teclado (para colocarlo en la salida)

En el coche, todo menos la estrategia:

```bash
ros2 launch robotek_course car.launch.py teleop:=true                  # hasta ~0,2 m/s
ros2 launch robotek_course car.launch.py teleop:=true motor_max_rps:=3   # otro tope para conducir (el de robot.yaml es 7 rps)
```

En tu ordenador, en la misma Wi-Fi que el coche, una ventana como un juego de carreras: mantén y combina
**↑ ↓ ← →** (o **W A S D**); al soltar todo se para y centra la dirección; si la ventana pierde el foco o se cierra,
el coche se para. Los números son la velocidad (m/s) y los grados de giro.

* **Mac** (RoboStack):

  ```bash
  ROBOSTACK_ENV=~/robotek-ros/jazzy bash tools/keyboard_racing.sh 0.4 15
  ```

* **Windows**: desde una terminal con ROS 2 Jazzy cargado (RoboStack con `pixi shell`, o los binarios oficiales con
  `call C:\dev\ros2_jazzy\local_setup.bat`):

  ```powershell
  powershell -ExecutionPolicy Bypass -File tools\keyboard_racing.ps1 -Speed 0.4 -Steer 15
  ```

  La primera vez, permite a Python en el cortafuegos de Windows (redes privadas). Si el coche tiene otra IP:
  `-PiIp 192.168.8.xxx`.

Las dos usan la red del coche (Cyclone DDS, `ROS_DOMAIN_ID` 1, descubrimiento en la subred, el Pi como par fijo). La
velocidad real la limita `motor_max_rps` del launch, no la ventana. Desde una terminal del Pi, sin ventana:
`ros2 run robotek_course keyboard_drive --ros-args -p hold:=true` (una tecla por movimiento: `u i o / j k l / m , .`).

### Ver qué hace

* `debug:=true` en el launch: la misma terminal informa de los estados, el sentido, cada esquina y vuelta y las líneas
  cruzadas.
* `python3 tools/open_monitor.py` en otra terminal del Pi: dos veces por segundo el estado, lo que ve la cámara (y las
  líneas que cruza), el LiDAR, el giroscopio y la dirección y velocidad que manda la estrategia; guarda imágenes de la
  decisión del sentido.
* `get-reference.sh` aplica los arreglos del coche de `tools/patches/` (con `--no-patch`, la referencia exacta del
  curso).

**Después de compilar en el Pi, `sync` (o `sudo poweroff`) antes de cortar la corriente**: un corte en mitad de la
escritura deja archivos vacíos (le pasó al ejecutable de la estrategia).

**La primera vez, con el coche sobre soportes (ruedas en el aire).** Ni el script ni el launch mueven el motor por su
cuenta: tu estrategia espera el Start (el botón de la placa) y el motor tiene límites de velocidad, aceleración y un
watchdog. El detalle (nodos, calibración, nombres de los dispositivos) está en
[`src/robotek_hardware/README.md`](src/robotek_hardware/README.md). El driver de la placa RRC Lite (basado en código de
Hiwonder, incluido con su permiso) está en `src/rrc_lite_*`; ver [CREDITS.md](CREDITS.md). Páginas de la guía:

* [Paso 10: el coche real](https://renzodamian.com/learn/open-strategy/paso-10-el-coche-real/)
* [Paso 11: solución de problemas](https://renzodamian.com/learn/open-strategy/paso-11-solucion-de-problemas/)
* [Paso 12: afinar](https://renzodamian.com/learn/open-strategy/paso-12-afinar/)

## El reto Obstacle

La segunda guía, **https://renzodamian.com/learn/obstacle-strategy**, construye la estrategia del reto Obstacle: tres
vueltas pasando las señales rojas por la derecha y las verdes por la izquierda, y al final aparcar en paralelo. Tu
paquete es `src/my_obstacle` (git lo ignora en `main`); la referencia está en la rama **`solutions-obstacle`**, con
una etiqueta `obstacle-step-N` por paso.

```bash
cd src
ros2 pkg create --build-type ament_python my_obstacle --node-name drive \
  --license MIT --maintainer-name "Tu Nombre" --maintainer-email tu@correo.com \
  --description "Mi estrategia del reto Obstacle" \
  --dependencies rclpy geometry_msgs sensor_msgs std_msgs cv_bridge robotek_common robotek_course
```

Una ronda en Gazebo: `seed` elige la disposición de las señales con el procedimiento de las reglas (la misma semilla
es el mismo campo) y `parking` pone el aparcamiento en la sección de salida.

```bash
ros2 launch robotek_course obstacle_round.launch.py strategy:=my_obstacle seed:=4 direction:=ccw parking:=true
ros2 run robotek_course score_obstacle                       # puntúa de nuevo la última ronda
python3 course/obstacle/verify_step.py 7                     # el paso 7 en Gazebo, sin ventana: PASS o FAIL
```

El laboratorio da la misma ronda en segundos, sin Gazebo (el campo, el coche real y el juez del Obstacle):

```bash
python3 -m robotek_evaluation.obstacle_lab --strategy my_obstacle.strategy:Strategy --seed 2 --direction cw
python3 -m robotek_evaluation.obstacle_lab --strategy my_obstacle.strategy:Strategy --layouts 0-4 --seeds 0-4 --early
python3 -m robotek_evaluation.obstacle_lab --strategy my_obstacle.strategy:Strategy --parking --layouts 0-9
```

| Pieza | Qué es |
|---|---|
| `robotek_course/obstacle_base.py` | La clase base de tu estrategia: escaneos, giroscopio, señales vistas (`on_signs`), `update` |
| `robotek_course/obstacle_vision.py` | Ver las señales: grupos del LiDAR, manchas rojas y verdes de la cámara, y su asociación |
| `robotek_evaluation/obstacle_field.py` | El campo por las reglas: las 36 cartas, el aparcamiento, el mundo de Gazebo |
| `robotek_evaluation/obstacle_judge.py` | El juez: vueltas, lados, señales movidas, parada y aparcamiento (40, o 55 con el aparcamiento) |
| `robotek_evaluation/obstacle_lab.py` | El laboratorio rápido, una ronda o lotes en paralelo |
| `robotek_course/launch/obstacle_round.launch.py` | Una ronda: campo, simulador, tu nodo y el árbitro (`obstacle_round_recorder`) |

Los puntos son estimaciones de simulación, no puntuación oficial; los +7 por arrancar dentro del aparcamiento no se
evalúan.

## Licencia

MIT, ver [LICENSE](LICENSE). Créditos y procedencia de los archivos en [CREDITS.md](CREDITS.md).

## Quick start: sala de preguntas en vivo (`/reto`)

Un cuestionario tipo Kahoot para clase: 5 preguntas por paso, 30 segundos cada una, puntos por acierto y rapidez, y
ranking en vivo. El servidor corre en tu computadora; los alumnos solo necesitan un navegador.

### Tú (profesor): arrancar el servidor

1. Instala [Bun](https://bun.sh) (una sola vez): `curl -fsSL https://bun.sh/install | bash`
2. Desde la carpeta del repositorio:

   ```bash
   bun reto/server.ts
   ```

3. En la terminal aparecen dos direcciones. Copia las dos:
   * **La tuya (anfitrión):** `http://localhost:3000/host?key=XXXXXX`. La clave `XXXXXX` cambia cada vez que arrancas
     el servidor y solo tú la ves. No se la des a los alumnos.
   * **La de los alumnos:** `http://192.168.x.x:3000/` (el número de tu computadora en la red).
4. Abre la dirección de anfitrión en tu navegador, marca los pasos que quieres jugar (del 1 al 9) y pulsa **Crear
   sala**. Verás un **código de 4 dígitos** muy grande. Proyéctalo o díctalo.
5. Cuando todos estén en la lista, pulsa **Empezar**. Con **Siguiente** avanzas de pregunta y de paso.

### Alumnos: cómo entrar

1. Conéctate al **mismo Wi-Fi** que el profesor.
2. Abre en el navegador del celular o la laptop la **dirección de alumnos** que te dé el profesor
   (por ejemplo `http://192.168.1.25:3000/`).
3. Escribe el **código de 4 dígitos** y un **apodo** (hasta 16 letras, sin tu nombre completo) y pulsa **Entrar**.
4. Espera a que el profesor empiece. En cada pregunta tienes 30 segundos: toca la respuesta (A, B, C o D). Cuanto más
   rápido aciertes, más puntos. Arriba siempre ves tu total y tu puesto.
5. Si se te cierra la página, vuelve a entrar con el **mismo código y apodo** y recuperas tus puntos.

### Si algo no funciona

* **Los alumnos no abren la dirección:** muchos Wi-Fi de colegio aíslan los dispositivos. Activa el punto de acceso
  (hotspot) de tu celular, conecta ahí tu computadora y los alumnos, y usa la dirección nueva que imprime el servidor.
* **Alumnos a distancia:** usa un túnel, por ejemplo `cloudflared tunnel --url http://localhost:3000`, y comparte el
  enlace `https://...trycloudflare.com` que aparece. No hace falta cuenta.
* **Reinicias el servidor:** la sala se pierde (todo está en memoria). Crea una nueva.
* Detalles, formato de las preguntas y privacidad: [`reto/README.md`](reto/README.md).
