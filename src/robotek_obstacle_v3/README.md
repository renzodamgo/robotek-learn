# robotek_obstacle_v3: Obstacle v3 como solución de referencia

La estrategia **Obstacle v3** del entrenador (WRO-Strategies, por defecto desde el 3 nov 2026) en el formato del
curso: una clase `Strategy` sobre `robotek_course.obstacle_base.ObstacleStrategyBase`, como la de `my_obstacle`.
No es un paso de la guía: es una estrategia terminada para comparar, mirar el código y probar en el coche.

Solo usa sensores honestos: LiDAR, giroscopio (IMU) y cámara. Sin encoder ni pose real.

## Qué hace

1. **SETUP**: calibra el giroscopio y lee el sentido de la ronda (CCW o CW) con los muros.
2. **RUN**: sigue el centro del carril y, por cada señal, se pone a un lado (roja a la derecha, verde a la
   izquierda). Recuerda cada señal por su asiento, así que desde la vuelta 2 ya conoce todas. Antes de cada esquina
   elige la línea (recta, arco, recta) con más holgura a las señales.
3. Tras tres vueltas, **PARK** (si el campo tiene el aparcamiento): aparca en paralelo en el lote. Si no, para en la
   recta de salida. El estado final es `DONE`.

## Archivos

| archivo | qué es |
| --- | --- |
| `strategy.py` | la `Strategy` del curso: conecta la clase base con la misión |
| `drive.py` | el nodo ROS (`drive`): /scan, /imu, /camera/image, /start → /cmd_vel_raw |
| `mission.py` | la misión: SETUP → RUN → PARK → STOP |
| `corner.py`, `corner_table.py` | la línea de cada esquina (`corner_table` se precalculó en WRO-Strategies) |
| `park.py`, `parking_search.py` | el aparcamiento en paralelo planificado (`park_mode: 0`, por defecto) |
| `park_fixed.py` | el aparcamiento fijo (`park_mode: 1`): cada paso termina en el giroscopio o el LiDAR |
| `localizer.py`, `field.py`, `scene.py` | ubicarse con los muros, la geometría del campo y del carril |
| `perception.py` | los grupos del LiDAR que pueden ser señales |
| `car.py` | las medidas del coche con las que se ajustó la v3 (ver abajo) |
| `settings.py` | los 35 parámetros de la misión (también parámetros ROS del nodo) |

La lógica es la de WRO-Strategies sin cambios, copiada del commit `5cf3771` (solo cambian los `import`). Lo que hacía su nodo con los sensores lo
hace aquí `ObstacleStrategyBase` (odometría del LiDAR, giro integrado, Start). La `Strategy` añade tres cosas para
decidir igual que el nodo original:

- la distancia solo cuenta cuando el localizador ya está listo;
- con un sensor viejo (más de 0.35 s), el coche se queda quieto (`WAIT_SENSOR`);
- a los 180 s de Start, `FAULT`.

## Probarla

En el laboratorio (sin Gazebo, unos segundos por ronda):

```bash
python3 -m robotek_evaluation.obstacle_lab --strategy robotek_obstacle_v3.strategy:Strategy --parking --layouts 0-4
```

En Gazebo, una ronda con el aparcamiento:

```bash
ros2 launch robotek_course obstacle_round.launch.py strategy:=robotek_obstacle_v3 seed:=0 direction:=ccw
```

Las pruebas:

```bash
python3 -m pytest src/robotek_obstacle_v3/test
```

## En el coche

Compilar y cargar el entorno en la Raspberry Pi (una vez por cambio):

```bash
cd ~/robotek-learn && colcon build --symlink-install --packages-select robotek_obstacle_v3 && source install/setup.bash
```

Lanzar en el coche (el botón de la placa da Start y otra pulsación lo para):

```bash
bash tools/car-autostart.sh stop && bash tools/car-clean.sh      # el arranque automático tiene la placa ocupada
ros2 launch robotek_course car.launch.py strategy:=robotek_obstacle_v3 camera_driver:=picam
```

Con esos dos argumentos, `car.launch.py` pone los valores de la mejor ronda en el coche (8 nov 2026, 23:08: **DONE a los
85,0 s** del Start, sin aparcamiento):
- **`motor_max_rps`:** 4.
- **`strategy_params`:** `config/car_picam.yaml` de este paquete, con la cámara IMX708 y `steer_limit` 0,3054 (17,5°).
- **`start_delay`:** 20 s con `button:=false`.

Cualquiera de los tres se cambia pasándolo al lanzar, por ejemplo `strategy_params:=otro.yaml` o `strategy_params:=''`
(sin archivo).

Sin botón: añadir `button:=false`; arranca a los 20 s. La estrategia tarda 13–17 s en estar lista; si el Start llega
antes se pierde y se queda en WAIT_START. Entonces, desde otra terminal del Pi:
`ros2 topic pub --once -w 2 /start std_msgs/msg/Bool "data: true"`.

**Dónde colocarlo:** centrado en el carril y a ~1,55–1,60 m de la pared de delante. Con un pilar en la recta, la v3 sale
con todo el giro; desde 0,4–0,5 m más atrás va más rápido, el arco se abre y choca (8 nov 2026). Para seguir los estados en la terminal: `debug:=true`. El
nodo publica su estado en `/robotek/state`. `/obstacle/pose` lleva x, y, rumbo y progreso. Para ver qué detecta, en
el Mac (WRO-Strategies): `PI_IP=192.168.8.153 bash tools/macos_rviz_real.sh docs/robot-real-pi/robot_obstacle.rviz`.
Muestra la imagen con los pilares (caja gruesa si el LiDAR lo confirma), las señales y la dirección. La imagen sale
solo si alguien la mira y como mucho a 5 Hz: a 25 fps dejaba sin cámara a la estrategia.

Para grabar la ronda, en otra terminal del coche:

```bash
ros2 bag record --max-bag-duration 5 -o ~/runs/$(date +%Y%m%d-%H%M%S)-obstacle-v3/bag /cmd_vel_raw /rrc/set_motor \
    /robotek/scan /robotek/imu /rrc/battery /robotek/state /start /obstacle/pose /obstacle/signs \
    /obstacle/debug_image/compressed /rrc/acker_servo/set_state
```

### Salir del aparcamiento al empezar (`start_lot:=true`, 9 nov 2026)

El coche espera dentro del aparcamiento, pegado al muro exterior y a ~1 cm del tope de atrás. Luces: verde
parpadeando = listo, esperando el botón; azul = pulsado, la ronda corre. Arranca con el botón de la placa o con el
pulsador nuevo del GPIO 16 (`/robotek/gpio_button`).

1. **Dirección:** el lado del LiDAR que tiene el muro cerca (o nada, porque el LiDAR no ve a menos de 12 cm). Muro a
   la derecha = antihorario; a la izquierda = horario.
2. **Salida** (`lot_exit.py`): pulsos con todo el giro, en `lot_exit_steps` (escritos para antihorario; en horario,
   en espejo). Cada paso es `<L|R|C><+|-><segundos>`: L izquierda a tope (lejos del muro), R derecha a tope, C ruedas
   al centro (servo ~1403 µs); + adelante, − atrás.
   Por defecto `L+0.5 R-0.5 L+2.0 R+0.5 R-1.0 L+0.5` a 0,70 m/s (`lot_exit_speed`), los valores del entrenador tras
   las primeras pruebas. Antes de cada pulso gira las ruedas parado (0,5 s); en los pasos hacia atrás el sensor de
   atrás frena en seco a < 20 mm. Al final, ruedas al centro.
3. **Vueltas** de la v3 sin aparcar al final y sin retroceder al empezar.

Durante la salida (estado `LOT_EXIT`) el
motor va con golpe de arranque y pulsos de 1,8 rps (`low_speed_states` de base_adapter): así el coche avanzó 5–6 cm/s
pidiendo 0,06 m/s; además, sin las rampas de aceleración y frenado (`no_ramp_states`): cada pulso arranca de golpe y
frena a 0 en seco. En el resto de la ronda, la velocidad y las rampas normales.

```bash
bash tools/car-autostart.sh stop && bash tools/car-clean.sh
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ROS_DOMAIN_ID=1 \
  ros2 launch robotek_course car.launch.py strategy:=robotek_obstacle_v3 camera_driver:=picam start_lot:=true \
  motor_min_rps:=1.5 lot_exit_speed:=0.70 lot_exit_steps:="L+0.5 R-0.5 L+2.0 R+0.5 R-1.0 L+0.5"
```

Grabar desde el Mac (WRO-Strategies): `bash tools/car_bag_mac.sh salida-lote`.

En el coche (9 nov 2026, 3 intentos, grabaciones en el Mac `~/rondas-coche/bags/20261109-*-salida-lote*`):
- a 0,12 m/s casi no se movió (cada pulso daba uno o dos golpes de motor; batería a 6,7–6,9 V);
- el segundo no empezó: la batería bajó de 6,26 a 5,1 V esperando y el Pi se apagó;
- con batería cargada (7,69 V), 0,25 m/s y `motor_min_rps:=1.5`: salió (rumbo 2,5° → 11,4° → 22,6° → 23,1°) y siguió en
  vueltas; en el pulso de atrás (entonces 0,5 s) la trasera casi tocó el muro exterior. Por eso ese pulso es ahora
  de 0,3 s: el sensor de atrás mira hacia atrás y no ve el muro de al lado. Sin juez ni puntos medidos.

Límite conocido: en el modelo de contorno del coche, un solo arco adelante a tope choca con el tope de delante a los
~11 cm (17°).

### En el coche real (8 nov 2026): primera ronda completa

- **Primera ronda que termina:** DONE a los 118,7 s del Start, con tres vueltas y la parada. Fue con el motor en M2
  sin tipo, engranajes 54:30, `motor_rps_per_mps` 4,4 y sin el freno de pared. El usuario: «funcionó perfecto».
  - Sin bag ni juez: no hay puntuación medida.
  - Notas: WRO-Strategies `docs/robot-real-pi/obstacle-runs/20261108/`.
- **Motor:** hasta el 8 nov no podía ir despacio. Se le mandaba el tipo de motor 2 a la placa, y el motor iba siempre
  a tope y no paraba con 0. Ver la sección Motor de `src/robotek_hardware/README.md`. Ahora la placa sigue la
  velocidad desde ~0,02 m/s.
  - Ya no hace falta `motor_min_rps`: déjalo en 0.
- **Fuerza a poca velocidad:** con 54:30 (el engranaje grande en el motor), a 0,15–0,20 m/s (0,7–0,9 rps), cerca de
  los pilares o en las curvas cerradas, al motor a veces le falta fuerza. Para el Obstacle se pueden usar los
  engranajes 30:54: 3,24 veces más fuerza en las ruedas y máxima ~0,65 m/s.
  - Entonces `motor_rps_per_mps` 13 en `robot.yaml` y `motor_max_rps:=10`.
- **Freno de pared** de `base_adapter`: **apagado** (`wall_stop_m: 0` en `robot.yaml`, 8 nov 2026).
  - Bajaba la velocidad cuando el LiDAR veía algo a menos de 0,30 m delante, lo que pasa todo el rato junto a pilares
    y paredes. El coche acababa parado.
  - Gazebo nunca lo tuvo.
- **Salida con un pilar en la recta:** la v3 pide un cambio de carril de 0,30 m sin rampa. La dirección se va a
  tope, el coche se pasa y puede tocar la isla. Pasó el 4 nov; ver WRO-Strategies
  `docs/robot-real-pi/obstacle-runs/20261104/`. Sigue pendiente de arreglar en la estrategia.
- **Dirección:** a veces, en el Open, el servo recibe todo el giro y el coche no gira. Si pasa en el Obstacle, es lo
  mismo; está pendiente.
- **Segunda ronda completa** (8 nov, 18:17): DONE a ~138 s del Start.
- **La mejor** (8 nov, 23:08): **DONE a los 85,0 s**, con el giro a 17,5° y lo demás como la de las 18:17. Sin esperas de
  sensores ni reinicios de la placa. Son los valores por defecto de `car.launch.py` para la v3.
- **Límite de giro:** la v3 limita el giro con `steer_limit` (0,307 rad = 17,6° por defecto) y `robot.yaml` permite
  ±19°. En el coche, `config/car_picam.yaml` lo deja en **0,3054 (17,5°)** desde el 8 nov, para no forzar el servo
  contra su tope. Subirlo cambia las líneas de las esquinas: validarlo antes en el laboratorio y en Gazebo.

### Aparcamiento en el coche (8 nov 2026)

`car.launch.py` no tiene un argumento `parking`: se activa en el archivo de parámetros. En el coche están:
- `~/.config/robotek/obstacle_v3_car_picam_park.yaml`: el de la cámara más `parking: true`, `lot_laps: 0` y
  `park_margin: 0.03`.
- `~/.config/robotek/obstacle_v3_car_picam_park_only.yaml`: lo mismo con `park_after: 0`, que aparca nada más arrancar,
  sin vueltas. Sirve para probar solo la maniobra, con el coche junto al aparcamiento.

`lot_laps` (1 por defecto): con 0, las vueltas son las de una ronda sin aparcamiento y el lote solo se usa para
aparcar. Con 1, las vueltas se apartan del lote y la salida cambia.

Lo que pasó (detalle y bolsas: WRO-Strategies `docs/robot-real-pi/obstacle-runs/20261108/`, «Rondas con aparcamiento de
la tarde»):
- **Las vueltas sí salen:** tres vueltas en 90 y 115 s con `lot_laps: 0`, y la v3 pasa a PARK.
- **El aparcamiento planificado no:** pide 0,06 m/s (0,26 rps) con todo el giro y el motor no mueve el coche, ni con
  los pulsos del adaptador (0,66 o 1 rps); acaba en FAULT.
- **Con `motor_min_rps` 1,1–1,3** sí hace las maniobras, pero se pasa en sus ~14 movimientos cortos y se atasca contra
  un tope por detrás, dos veces en el mismo punto.
- **Falta medir el lote del campo:** la v3 usa el del reglamento (0,35 × 0,20 m, límite lejano a 0,48 m del centro de
  la recta de salida).
- Por eso existe el aparcamiento fijo de la sección siguiente.

### Aparcamiento fijo (`park_mode: 1`, 8 nov 2026)

El aparcamiento planificado (`park.py`) no funciona en el coche real del 8 nov:
- con todo el giro, el motor no mueve el coche a 0,06 m/s;
- con `motor_min_rps` va a ~0,25 m/s y se pasa en cada uno de sus ~14 movimientos cortos.

`park_fixed.py` no planifica. Hace estos pasos:
1. Se pone a `park_fixed_ahead` (0,145 m) por delante del límite lejano.
2. Marcha atrás con todo el giro hasta `park_fixed_entry` (0,76 rad, giroscopio).
3. Marcha atrás recta hasta que el contorno queda cerca de la pared o de un límite (LiDAR).
4. Endereza adelante y atrás con todo el giro.
5. Se centra.

**Lote:** su tamaño y su posición son parámetros (`park_lot_length` 0,3495, `park_lot_depth` 0,20 y `park_lot_end` 0,48 m,
los del reglamento): hay que poner los del campo medido.

**Resultados en el laboratorio de WRO-Strategies** (`--car real_v3`, solo aparcar, 200 rondas):
- el fijo aparca bien 195 veces y el planificado, 35;
- con un piso de 0,25 m/s (como `motor_min_rps` 1,1) no aparca ninguno de los dos;
- a 1 cm de error en el lote baja a 128–184.

Comando de la prueba en el coche: WRO-Strategies `docs/robot-real-pi/obstacle-runs/20261108/README.md`, sección
«Aparcamiento fijo».

### La cámara IMX708 (así está el coche desde el 4 nov 2026)

Los valores por defecto de `drive.py` son los de la cámara USB (medida el 8 oct 2026) y los colores de la cámara
simulada. Con la IMX708, `car.launch.py` carga `config/car_picam.yaml` (`strategy_params` por defecto con
`camera_driver:=picam`).

Con `camera_driver:=picam`, el Obstacle usa la cámara toda la ronda a 640x360 y 30 fps (en el coche llegan ~25 fps;
`drive` usa ~42 % de un núcleo). `config/car_picam.yaml`, medido el 4 nov 2026 con tres
pilares a ~0,9 m comparados con los grupos del LiDAR (error de rumbo 0,8° rms):

```yaml
strategy:
  ros__parameters:
    camera_hfov: 1.78        # rad, ~102°: es la IMX708 Wide (rpicam la lista solo como imx708)
    camera_pitch: 0.14       # rad hacia abajo, por la base de los pilares (cámara a 0,142 m del suelo)
    camera_yaw: 0.054        # rad, + = mira a la izquierda del cero del LiDAR
    red_hsv: "170,6,90,80"   # pilar H 177-1, S 107-138, V 117-132; v_min 80 deja fuera la línea naranja (V ~50)
    green_hsv: "40,85,70,60" # pilar H 50-59, S 97-126, V 102-122
```

Formato de los colores: `"h_min,h_max,s_min,v_min"` (si h_min > h_max, el rango da la vuelta por 0, como el rojo).
Con los valores de la cámara USB, la IMX708 no veía el pilar rojo de la derecha y ponía el verde del centro sobre
un objeto a 2,1 m; con estos, los tres pilares caen en su grupo del LiDAR. Medido solo con la luz de la sala, de
frente y a ~0,9 m: falta comprobarlo de lado, de lejos y en marcha.

## Las medidas del coche (`car.py`)

`car.py` es una copia de `robotek_common/vehicle.py` de WRO-Strategies (medidas del STEP del 26 oct 2026). El
`vehicle.py` de robotek-learn tiene las medidas del calibre del 21 oct, que difieren hasta en 1.4 mm (distancia entre
ejes: .1174 aquí y .11883 en WRO-Strategies). `corner_table.py` se calculó con las de WRO-Strategies. Si la v3 usara
las de aquí, las holguras de las esquinas cambiarían en el cuarto decimal y dejarían de coincidir con la tabla (la
prueba `test_v3_corner_table.py` lo detecta). Solo `drive.py` usa `robotek_common.vehicle.WHEELBASE`, para convertir
el giro en /cmd_vel_raw tal como lo deshace `base_adapter`.

## Resultados

Ver [RESULTADOS.md](RESULTADOS.md): laboratorio y Gazebo, comparados con la v3 de WRO-Strategies.
