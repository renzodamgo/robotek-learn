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

Lanzar (el botón de la placa da Start; otra pulsación lo para):

```bash
ros2 launch robotek_course car.launch.py strategy:=robotek_obstacle_v3
```

Para seguir los estados en esa terminal, añadir `debug:=true`. El nodo publica su estado en `/robotek/state` y lo
escribe en la terminal. `/obstacle/pose` lleva x, y, rumbo y progreso. **Nunca se ha probado en el coche real.**
La primera vez, probarlo sobre tacos y con el motor limitado (`motor_max_rps:=2.5`; el motor 310 no siempre se para con 0: ver el freno en `src/robotek_hardware/README.md`).

Antes de la primera ronda real, revisar esto:

- **Aparcamiento**: `car.launch.py` todavía no tiene un argumento `parking` (Gazebo sí se lo pasa al nodo), así que
  en el coche la v3 no aparca: para en la recta de salida tras tres vueltas.
- **Dirección**: la v3 limita el giro a 0.307 rad (17.6°, `steer_limit`). `robot.yaml` permite ±19°. La v3 funciona
  dentro de ese límite pero no usa los 1.4° que sobran. Subir `steer_limit` cambia las líneas de las esquinas y el
  aparcamiento, así que habría que validarlo antes en el laboratorio y en Gazebo.
- **Velocidad**: la v3 pide 0.35 m/s en recta, 0.30 junto a las señales y 0.06 al aparcar. `base_adapter` convierte
  la velocidad con `motor_rps_per_mps` 4.8, una tabla medida solo cerca de 1 rps. Hay que comprobar con el LiDAR
  que 0.06 m/s mueve el coche (el motor nuevo no se movía a 1.3 rps o menos).
- **Freno de muro** de `base_adapter` (`wall_slow_m` 0.30, `wall_stop_m` 0.10): reduce la velocidad si el LiDAR ve algo
  delante a menos de 0.30 m. Puede frenar a la v3 en las esquinas y al aparcar. Gazebo no tiene ese freno.
- **Colores**: los rangos HSV de `robotek_course.obstacle_vision` se ajustaron con la cámara simulada. En el coche
  hay que ajustarlos de nuevo.

## Las medidas del coche (`car.py`)

`car.py` es una copia de `robotek_common/vehicle.py` de WRO-Strategies (medidas del STEP del 26 oct 2026). El
`vehicle.py` de robotek-learn tiene las medidas del calibre del 21 oct, que difieren hasta en 1.4 mm (distancia entre
ejes: .1174 aquí y .11883 en WRO-Strategies). `corner_table.py` se calculó con las de WRO-Strategies. Si la v3 usara
las de aquí, las holguras de las esquinas cambiarían en el cuarto decimal y dejarían de coincidir con la tabla (la
prueba `test_v3_corner_table.py` lo detecta). Solo `drive.py` usa `robotek_common.vehicle.WHEELBASE`, para convertir
el giro en /cmd_vel_raw tal como lo deshace `base_adapter`.

## Resultados

Ver [RESULTADOS.md](RESULTADOS.md): laboratorio y Gazebo, comparados con la v3 de WRO-Strategies.
