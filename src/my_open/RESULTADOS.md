# Resultados de la solución de referencia

Estimaciones de simulación con las reglas publicadas, no puntuación oficial. Los tiempos van desde Start hasta la
parada, en segundos simulados.

## Gazebo (`ros2 launch robotek_course round.launch.py`, coche real simulado)

| Sentido | Puntos | Tiempo | Vueltas (s) | Margen mínimo a los muros | Válida |
|---|---:|---:|---|---:|:---:|
| ccw | 30 de 30 | 35,56 | 11,4 / 11,0 / 11,0 | 17,7 cm | sí |
| cw | 30 de 30 | 35,53 | 11,4 / 11,0 / 11,0 | 18,0 cm | sí |

## Open lab (12 rondas, 2 s cada una)

`python3 -m robotek_evaluation.open_lab --strategy my_open.strategy:Strategy`

| Campo | ccw | cw |
|---|---:|---:|
| square100 | 30 · 33,78 s | 30 · 33,78 s |
| square180 | 30 · 51,38 s | 30 · 51,28 s |
| rect100x180 | 30 · 41,98 s | 30 · 41,98 s |
| rect180x100 | 30 · 43,18 s | 30 · 43,18 s |
| shift140 | 30 · 42,08 s | 30 · 42,78 s |
| east140x100 | 30 · 37,78 s | 30 · 38,68 s |

12 de 12 rondas válidas, tiempo medio 41,82 s. El Open lab es un filtro rápido (modelo del coche en Python); la
autoridad es Gazebo.

## Tu tabla

Anota aquí tus rondas (`ros2 run robotek_course score`): puntos primero, tiempo desde Start después.

| Fecha | Cambio | Sentido | Puntos | Tiempo | Válida |
|---|---|---|---:|---:|:---:|
| | | | | | |
