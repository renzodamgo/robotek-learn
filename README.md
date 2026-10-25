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
| `src/ldlidar_stl_ros2` | Driver del LiDAR LD19 (copia del de LDROBOT, MIT) |
| `src/rrc_lite_driver`, `src/rrc_lite_msgs` | Driver de la placa RRC Lite (basado en Hiwonder MentorPi, con permiso) |
| `tools/setup-pi.sh` | Deja un Raspberry Pi listo para el coche real (un solo comando) |
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
velocidad y por qué la primera vez conviene `motor_max_pct:=30`, más abajo. Para tu propia estrategia: crea tu paquete como en
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

**La primera vez, con el coche sobre soportes (ruedas en el aire).** Ni el script ni el launch mueven el motor por su
cuenta: tu estrategia espera el Start (el botón de la placa) y el motor tiene límites de velocidad, aceleración y un
watchdog. El detalle (nodos, calibración, nombres de los dispositivos) está en
[`src/robotek_hardware/README.md`](src/robotek_hardware/README.md). El driver de la placa RRC Lite (basado en código de
Hiwonder, incluido con su permiso) está en `src/rrc_lite_*`; ver [CREDITS.md](CREDITS.md). Páginas de la guía:

* [Paso 10: el coche real](https://renzodamian.com/learn/open-strategy/paso-10-el-coche-real/)
* [Paso 11: solución de problemas](https://renzodamian.com/learn/open-strategy/paso-11-solucion-de-problemas/)
* [Paso 12: afinar](https://renzodamian.com/learn/open-strategy/paso-12-afinar/)

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
