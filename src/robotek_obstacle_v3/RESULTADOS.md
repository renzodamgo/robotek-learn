# Resultados de robotek_obstacle_v3 (3 nov 2026)

Son puntos estimados de simulación, no puntuación oficial. Todas las rondas tienen el aparcamiento, las semillas de
ruido 0–2 y los campos 0–4 en los dos sentidos (30 rondas por fila).

## 1. ¿Decide igual que la v3 de WRO-Strategies?

Sí. En el laboratorio de robotek-learn corrí, con los mismos campos y semillas, esta `Strategy` y la `Mission` original
de WRO-Strategies, conectada como su nodo (`v3/node.py`). Las **30 trayectorias son idénticas fila a fila** (la pose
cada 0.1 s de toda la ronda), con los mismos puntos y el mismo tiempo. El arnés está fuera del repositorio porque
necesita el código de WRO-Strategies.

## 2. Laboratorio: robotek-learn frente a WRO-Strategies

| laboratorio | válidas | puntos medios | tiempo medio de las válidas |
| --- | --- | --- | --- |
| robotek-learn (`obstacle_lab`, con su `vehicle.py`) | 23/30 | 47.4 | 143.6 s |
| robotek-learn con las medidas de WRO-Strategies (el arnés del punto 1) | 24/30 | 48.0 | — |
| WRO-Strategies (`tools/obstacle_lab_suite.py --strategy v3 --parking`) | 21/30 | 46.2 | 143.2 s |

```bash
python3 -m robotek_evaluation.obstacle_lab --strategy robotek_obstacle_v3.strategy:Strategy --parking --layouts 0-4 --seeds 0-2
```

Los mismos campos fallan igual en los dos laboratorios:

- **campo 2, CCW**: choca con la señal verde 3 a los 28 s (3 puntos), en todas las semillas y en los dos laboratorios;
- **campo 3, CW**: aparca pero pasa de 180 s (37 puntos, `rule_maximum`), en todas las semillas y en los dos laboratorios.

Las diferencias están en el aparcamiento:

- **campo 1, CW**: robotek-learn toca un límite del lote en 1 de 3 semillas; WRO-Strategies, en 2 de 3;
- **campo 2, CW**: solo WRO-Strategies toca el límite, en 1 de 3 semillas.

El laboratorio de WRO-Strategies modela más cosas: la oclusión 3D de la cámara (lo que se ve por encima y
alrededor de las otras cajas) y usa otra secuencia de ruido. Las medidas del simulador también cambian (el `vehicle.py` de robotek-learn). La v3 decide igual,
así que estas diferencias vienen de los simuladores.

## 3. Gazebo con GUI

`obstacle_round.launch.py strategy:=robotek_obstacle_v3 seed:=1 direction:=ccw parking:=true`, en el Mac, 1x, con la
cámara, el LiDAR y los actuadores del coche real.

| ronda | puntos | tiempo desde Start | vueltas | aparcamiento | válida |
| --- | --- | --- | --- | --- | --- |
| `20261103T225820-v3-port-gui-reg1-ccw-park` | 55/55 | 129.26 s | 28.9 / 26.3 / 26.6 s | paralelo completo | sí (margen 3.7 cm) |

Con el mismo campo, el laboratorio da 130.0 s. La ronda grabada de la v3 en WRO-Strategies (reg1-ccw) hizo 55 puntos
en 129.02 s.

En Gazebo hubo 9 pausas breves `WAIT_SENSOR`, de 20 a 150 ms: algún sensor llegó con más de 0.35 s, como hace el nodo
original. Un intento anterior (`20261103T225427-v3-port-gui-reg1-ccw`) hizo 40 puntos y no aparcó. Era un fallo de
`drive.py`: el parámetro `park=0` por defecto anulaba `parking:=true`. Ya está corregido y lo cubre `test_drive.py`.
En ese intento también se cerró la ventana de Gazebo (segfault de la GUI) y la ronda siguió en el servidor.
