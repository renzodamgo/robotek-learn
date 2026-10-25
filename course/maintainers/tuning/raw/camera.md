### Camera: slower to see the direction

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| camera ready after 0.3 s | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| camera ready after 1 s | 12/12 | 30.0 | 41.52 | 33.28 | 7.4 |  |
| camera ready after 3 s | 12/12 | 30.0 | 43.45 | 35.38 | 9.5 |  |
| camera ready after 8 s | 12/12 | 30.0 | 48.39 | 40.28 | 9.5 |  |
| camera ready after 14 s | 12/12 | 30.0 | 54.41 | 46.28 | 8.0 |  |
| camera ready after 16 s | 0/12 | 0.0 | - | - | 17.6 | stopped short in DIRECTION_FAULT x12 |
| camera 6 fps | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| camera 3 fps | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
| camera 1 fps | 12/12 | 30.0 | 40.75 | 32.58 | 9.5 |  |
