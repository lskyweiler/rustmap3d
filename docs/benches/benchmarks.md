### rustmap3d-ecef2lla-batch-vs-single

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2lla_batch` | 353.49 µs | 309.29 µs | 266.25 µs | 87.34 µs | 1192 | 10.5x |
| `test_ecef2lla_loop` | 3.71 ms | 3.69 ms | 3.55 ms | 90.68 µs | 261 | 1.0x |

### rustmap3d-lla2ecef-batch-vs-single

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_lla2ecef_batch` | 417.08 µs | 405.42 µs | 219.12 µs | 100.75 µs | 1669 | 6.9x |
| `test_lla2ecef_loop` | 2.88 ms | 2.86 ms | 2.76 ms | 101.14 µs | 350 | 1.0x |

### rustmap3d-ecef2aer-batch-vs-single

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2aer_batch` | 454.47 µs | 449.63 µs | 249.12 µs | 113.48 µs | 1535 | 8.4x |
| `test_ecef2aer_loop` | 3.84 ms | 3.79 ms | 3.61 ms | 194.20 µs | 262 | 1.0x |

### ecef2lla-batch-vs-pymap3d

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2lla_batch_rustmap3d` | 390.34 µs | 332.69 µs | 260.08 µs | 131.78 µs | 2562 | 1.5x |
| `test_ecef2geodetic_vectorized_pymap3d` | 568.65 µs | 554.75 µs | 506.83 µs | 54.51 µs | 542 | 1.0x |

### lla2ecef-batch-vs-pymap3d

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_geodetic2ecef_vectorized_pymap3d` | 226.65 µs | 223.42 µs | 208.88 µs | 13.80 µs | 2632 | 1.6x |
| `test_lla2ecef_batch_rustmap3d` | 360.13 µs | 336.87 µs | 188.71 µs | 108.48 µs | 1826 | 1.0x |

### ecef2aer-batch-vs-pymap3d

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2aer_vectorized_pymap3d` | 251.94 µs | 232.58 µs | 205.25 µs | 43.34 µs | 2070 | 1.4x |
| `test_ecef2aer_batch_rustmap3d` | 343.08 µs | 305.13 µs | 241.71 µs | 85.98 µs | 2113 | 1.0x |

### rustmap3d-ecef2lla-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2lla_batch_parallel` | 356.59 µs | 310.85 µs | 257.71 µs | 89.92 µs | 2264 | 1.9x |
| `test_ecef2lla_batch_serial` | 665.09 µs | 647.13 µs | 596.75 µs | 65.65 µs | 899 | 1.0x |

### rustmap3d-lla2ecef-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_lla2ecef_batch_serial` | 203.84 µs | 204.46 µs | 184.67 µs | 7.98 µs | 4513 | 1.7x |
| `test_lla2ecef_batch_parallel` | 353.95 µs | 332.75 µs | 237.12 µs | 88.51 µs | 1675 | 1.0x |

### rustmap3d-ecef2aer-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2aer_batch_parallel` | 357.42 µs | 319.08 µs | 240.50 µs | 95.50 µs | 1997 | 1.1x |
| `test_ecef2aer_batch_serial` | 396.26 µs | 395.83 µs | 364.42 µs | 15.88 µs | 1886 | 1.0x |

### single-ecef2lla

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2lla_rustmap3d` | 1.30 µs | 1.29 µs | 1.25 µs | 36.52 ns | 10 | 238.2x |
| `test_ecef2geodetic_pymap3d` | 309.71 µs | 308.48 µs | 305.67 µs | 5.44 µs | 10 | 1.0x |

### single-lla2ecef

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_lla2ecef_rustmap3d` | 571.77 ns | 568.40 ns | 555.54 ns | 14.97 ns | 10 | 43.7x |
| `test_geodetic2ecef_pymap3d` | 24.98 µs | 24.91 µs | 24.70 µs | 298.46 ns | 10 | 1.0x |

### single-vincenty-direct

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_vincenty_direct_rustmap3d` | 1.99 µs | 1.97 µs | 1.95 µs | 48.07 ns | 10 | 236.3x |
| `test_vreckon_pymap3d` | 469.75 µs | 469.18 µs | 466.76 µs | 2.79 µs | 10 | 1.0x |

### single-vincenty-inverse

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_vincenty_inverse_rustmap3d` | 3.41 µs | 3.40 µs | 3.34 µs | 56.96 ns | 10 | 279.4x |
| `test_vdist_pymap3d` | 952.09 µs | 945.23 µs | 939.52 µs | 13.82 µs | 10 | 1.0x |
