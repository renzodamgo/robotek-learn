### Control latency (Pi/ROS between the strategy and the actuators)

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| latency 0 ms | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| latency 20 ms | 12/12 | 30.0 | 41.14 | 33.08 | 6.9 |  |
| latency 50 ms | 12/12 | 30.0 | 41.45 | 33.38 | 7.6 |  |
| latency 100 ms | 12/12 | 30.0 | 41.89 | 33.88 | 3.0 |  |
| latency 200 ms | 5/12 | 14.7 | 39.64 | 33.88 | 0.0 | wall in TURN x7 |
