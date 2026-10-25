# Resultados de la solución de referencia (`my_obstacle`)

Medido el 25 de octubre de 2026. Todos los puntos son **estimaciones de simulación**, no
puntuación oficial: el juez del curso aplica las reglas de 2026 sobre la pose real del coche
(secciones, vueltas, lados de cada señal, señales movidas, parada y aparcamiento).

## Laboratorio del curso

`python3 -m robotek_evaluation.obstacle_lab --strategy my_obstacle.strategy:Strategy ...`, coche
medido (servo, motor, LiDAR con ruido y pérdidas, cámara de 66,7° girada 4,4°), semilla de ruido 0.
Sin aparcamiento con `--layouts 0-49 --early`; con aparcamiento `--layouts 0-9 --parking --tmax 200`.

Con aparcamiento el resultado puede cambiar en una o dos rondas de una tanda a otra: la búsqueda
del aparcamiento tiene un tope de tiempo de reloj, y con la máquina cargada (12 rondas en paralelo)
encuentra menos. En otra tanda, en el laboratorio del equipo, salieron 9/20.

| Conjunto | Rondas | Válidas | Puntos de una válida | Tiempo medio de las válidas |
|---|---:|---:|---:|---:|
| Tres vueltas, campos 0–49 × CCW/CW | 100 | **88/100** | 40 | 104,5 s |
| Tres vueltas + aparcar, campos 0–9 × CCW/CW | 20 | **11/20** | 55 | 152,8 s |

## Gazebo

Una ronda de la estrategia del equipo de la que sale esta solución (el mismo código), con el
coche real simulado, sin ventana y a doble velocidad: campo 4 en CCW, **40 puntos, válida, 115,6 s**.
Con el aparcamiento las tres vueltas salieron bien (37 puntos) pero el aparcamiento no terminó
antes de los 180 s: la búsqueda que corrige el plan bloquea el bucle de control unos segundos.

## Lo que todavía falla (y por qué)

- **La esquina verde interior → roja exterior** (o al revés): la línea recta-arco-recta solo deja
  3,4 cm de margen con un seguimiento perfecto; el seguimiento real se los come. Es la causa de
  la mayoría de los contactos.
- **En CW la cámara ve poco del lado de la isla** (28,9°, porque está girada 4,4° a la
  izquierda): las señales interiores justo después de una esquina se ven tarde. «Mirar» (retroceder
  girando la nariz 15°) lo arregla casi siempre, no siempre.
- **Aparcar tarda** 35–70 s y a veces queda torcido o a medias (7 puntos en vez de 15).
- **La salida** con la primera señal en el asiento del final y a 0,6 m: el coche no puede
  retroceder (el asiento de detrás puede estar ocupado) y no siempre llega al lado correcto.

## Cómo se verificó el port

La solución es la estrategia v2 del equipo partida en los módulos de la guía. Corriendo las dos
en el mismo laboratorio (el del repositorio del equipo), campo por campo, salen **idénticas**:
100/100 campos sin aparcamiento y 20/20 con aparcamiento, mismos puntos y mismo tiempo.
