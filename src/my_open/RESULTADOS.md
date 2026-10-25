# Resultados de la solución de referencia

Estimaciones de simulación con las reglas publicadas, no puntuación oficial. Los tiempos van desde Start hasta la
parada, en segundos simulados.

## Gazebo (`ros2 launch robotek_course round.launch.py`, coche real simulado)

| Sentido | Puntos | Tiempo | Vueltas (s) | Margen mínimo a los muros | Válida |
|---|---:|---:|---|---:|:---:|
| ccw | 30 de 30 | 34,80 | 10,9 / 10,6 / 11,0 | 11,9 cm | sí |
| cw | 30 de 30 | 34,90 | 10,9 / 10,6 / 11,0 | 11,1 cm | sí |

## Open lab (12 rondas, 2 s cada una)

`python3 -m robotek_evaluation.open_lab --strategy my_open.strategy:Strategy`

| Campo | ccw | cw |
|---|---:|---:|
| square100 | 30 · 32,58 s | 30 · 32,58 s |
| square180 | 30 · 50,18 s | 30 · 50,28 s |
| rect100x180 | 30 · 40,58 s | 30 · 40,58 s |
| rect180x100 | 30 · 42,28 s | 30 · 42,28 s |
| shift140 | 30 · 40,58 s | 30 · 42,08 s |
| east140x100 | 30 · 36,78 s | 30 · 38,28 s |

12 de 12 rondas válidas, tiempo medio 40,75 s. El Open lab es un filtro rápido (modelo del coche en Python); la
autoridad es Gazebo.

## Tu tabla

Anota aquí tus rondas (`ros2 run robotek_course score`): puntos primero, tiempo desde Start después.

| Fecha | Cambio | Sentido | Puntos | Tiempo | Válida |
|---|---|---|---:|---:|:---:|
| | | | | | |
