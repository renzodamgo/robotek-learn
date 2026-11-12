### Baseline vs the faster candidates when something else is off

Seeds 0,1,2, 36 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| baseline, motor x0.85 | 36/36 | 30.0 | 48.38 | 38.68 | 11.5 |  |
| fast, motor x0.85 | 18/36 | 28.5 | 47.77 | - | 11.6 | stopped short in DONE x18 |
| safer, motor x0.85 | 27/36 | 29.2 | 47.23 | - | 11.0 | stopped short in DONE x9 |
| baseline, motor x1.15 | 29/36 | 29.4 | 37.74 | 29.98 | 7.9 | stopped short in DONE x7 |
| fast, motor x1.15 | 18/36 | 28.5 | 36.77 | - | 4.0 | stopped short in DONE x18 |
| safer, motor x1.15 | 18/36 | 28.5 | 38.00 | - | 5.3 | stopped short in DONE x18 |
| baseline, servo delay x2 | 36/36 | 30.0 | 42.54 | 34.38 | 6.9 |  |
| fast, servo delay x2 | 18/36 | 28.5 | 42.12 | - | 4.4 | stopped short in DONE x18 |
| safer, servo delay x2 | 18/36 | 28.5 | 43.56 | - | 6.6 | stopped short in DONE x18 |
| baseline, servo rate 0.6 | 36/36 | 30.0 | 43.39 | 35.46 | 2.9 |  |
| fast, servo rate 0.6 | 17/36 | 27.2 | 42.58 | - | 0.0 | wall in TURN x2; stopped short in DONE x17 |
| safer, servo rate 0.6 | 32/36 | 29.7 | 41.86 | 34.14 | 1.5 | stopped short in DONE x4 |
| baseline, LiDAR dropout 12% | 36/36 | 30.0 | 41.83 | 33.78 | 10.0 |  |
| fast, LiDAR dropout 12% | 18/36 | 28.5 | 41.44 | - | 6.7 | stopped short in DONE x18 |
