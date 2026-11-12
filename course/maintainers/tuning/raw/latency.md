### Control latency (Pi/ROS between the strategy and the actuators)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| latency 0 ms | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| latency 20 ms | 12/12 | 30.0 | 42.06 | 33.88 | 9.2 |  |
| latency 50 ms | 12/12 | 30.0 | 42.48 | 34.48 | 9.5 |  |
| latency 100 ms | 12/12 | 30.0 | 43.12 | 35.28 | 4.9 |  |
| latency 200 ms | 5/12 | 21.5 | 44.00 | - | 0.0 | wall in TURN x4; stopped short in DONE x1 |
