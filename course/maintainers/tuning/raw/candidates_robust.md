### Baseline vs search result when something else is off

Seeds 0, 12 rounds per row.

| Configuration | ok | pts | t (s) | sq100 (s) | clr (cm) | failures |
|---|---:|---:|---:|---:|---:|---|
| baseline, motor x0.85 | 12/12 | 30.0 | 47.03 | 37.18 | 12.1 |  |
| search result, motor x0.85 | 12/12 | 30.0 | 44.78 | 35.88 | 12.2 |  |
| safer, motor x0.85 | 12/12 | 30.0 | 45.28 | 36.28 | 12.2 |  |
| baseline, motor x1.15 | 12/12 | 30.0 | 35.99 | 28.58 | 7.0 |  |
| search result, motor x1.15 | 7/12 | 28.8 | 36.74 | - | 7.0 | stopped short in DONE x5 |
| safer, motor x1.15 | 12/12 | 30.0 | 34.83 | 27.98 | 7.0 |  |
| baseline, servo delay x2 | 12/12 | 30.0 | 41.54 | 33.28 | 5.9 |  |
| search result, servo delay x2 | 12/12 | 30.0 | 39.62 | 32.18 | 5.9 |  |
| safer, servo delay x2 | 12/12 | 30.0 | 40.05 | 32.38 | 5.9 |  |
| baseline, servo rate 0.6 | 12/12 | 30.0 | 42.24 | 34.18 | 1.7 |  |
| search result, servo rate 0.6 | 12/12 | 30.0 | 40.31 | 32.98 | 1.7 |  |
| safer, servo rate 0.6 | 12/12 | 30.0 | 40.74 | 33.18 | 1.7 |  |
| baseline, LiDAR dropout 12% | 12/12 | 30.0 | 40.67 | 32.58 | 9.1 |  |
| search result, LiDAR dropout 12% | 12/12 | 30.0 | 38.85 | 31.58 | 9.1 |  |
