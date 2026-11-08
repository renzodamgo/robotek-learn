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
| `park.py`, `parking_search.py` | el aparcamiento en paralelo |
| `localizer.py`, `field.py`, `scene.py` | ubicarse con los muros, la geometría del campo y del carril |
| `perception.py` | los grupos del LiDAR que pueden ser señales |
| `car.py` | las medidas del coche con las que se ajustó la v3 (ver abajo) |
| `settings.py` | los 26 parámetros de la misión (también parámetros ROS del nodo) |

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

Lanzar en el coche (así corre desde el 8 nov 2026; el botón de la placa da Start y otra pulsación lo para):

```bash
bash tools/car-autostart.sh stop && bash tools/car-clean.sh      # el arranque automático tiene la placa ocupada
ros2 launch robotek_course car.launch.py strategy:=robotek_obstacle_v3 camera_driver:=picam motor_max_rps:=3.4 \
    strategy_params:=$HOME/.config/robotek/obstacle_v3_car_picam.yaml
```

Sin botón (arranca solo a los 12 s): añadir `button:=false`. Para seguir los estados en la terminal: `debug:=true`. El
nodo publica su estado en `/robotek/state`. `/obstacle/pose` lleva x, y, rumbo y progreso. Para ver qué detecta, en
el Mac (WRO-Strategies): `PI_IP=192.168.8.153 bash tools/macos_rviz_real.sh docs/robot-real-pi/robot_obstacle.rviz`.
Muestra la imagen con los pilares (caja gruesa si el LiDAR lo confirma), las señales y la dirección.

Para grabar la ronda, en otra terminal del coche:

```bash
ros2 bag record --max-bag-duration 5 -o ~/runs/$(date +%Y%m%d-%H%M%S)-obstacle-v3/bag /cmd_vel_raw /rrc/set_motor \
    /robotek/scan /robotek/imu /rrc/battery /robotek/state /start /obstacle/pose /obstacle/signs \
    /obstacle/debug_image/compressed /rrc/acker_servo/set_state
```

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
- **Aparcamiento:** `car.launch.py` todavía no tiene un argumento `parking`, así que en el coche la v3 no aparca: para
  en la recta de salida tras tres vueltas.
- **Límite de giro:** la v3 limita el giro a 0.307 rad (17.6°, `steer_limit`) y `robot.yaml` permite ±19°. Subir
  `steer_limit` cambia las líneas de las esquinas, así que habría que validarlo antes en el laboratorio y en Gazebo.

### La cámara IMX708 (así está el coche desde el 4 nov 2026)

Los valores por defecto de `drive.py` son los de la cámara USB (medida el 8 oct 2026) y los colores de la cámara
simulada; con la IMX708 se usa `strategy_params:=$HOME/.config/robotek/obstacle_v3_car_picam.yaml`.

Con `camera_driver:=picam`, el Obstacle usa la cámara toda la ronda a 640x360 y 30 fps (en el coche llegan ~25 fps;
`drive` usa ~42 % de un núcleo). `~/.config/robotek/obstacle_v3_car_picam.yaml`, medido el 4 nov 2026 con tres
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
