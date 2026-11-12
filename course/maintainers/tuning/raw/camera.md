### Camera: slower to see the direction

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| camera ready after 0.3 s | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| camera ready after 1 s | 12/12 | 30.0 | 42.51 | 34.38 | 9.6 |  |
| camera ready after 3 s | 12/12 | 30.0 | 44.59 | 36.28 | 10.0 |  |
| camera ready after 8 s | 12/12 | 30.0 | 49.50 | 41.38 | 10.2 |  |
| camera ready after 14 s | 12/12 | 30.0 | 55.48 | 47.28 | 10.3 |  |
| camera ready after 16 s | 0/12 | 0.0 | - | - | 17.6 | stopped short in DIRECTION_FAULT x12 |
| camera 6 fps | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| camera 3 fps | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
| camera 1 fps | 12/12 | 30.0 | 41.82 | 33.78 | 10.1 |  |
