### rustmap3d-ecef2lla-batch-vs-single

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2lla_batch` | 335.91 µs | 294.13 µs | 241.00 µs | 85.61 µs | 1153 | 11.0x |
| `test_ecef2lla_loop` | 3.69 ms | 3.69 ms | 3.55 ms | 58.44 µs | 264 | 1.0x |

### rustmap3d-lla2ecef-batch-vs-single

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_lla2ecef_batch` | 179.03 µs | 179.40 µs | 167.58 µs | 7.54 µs | 4988 | 16.1x |
| `test_lla2ecef_loop` | 2.89 ms | 2.89 ms | 2.72 ms | 70.79 µs | 325 | 1.0x |

### rustmap3d-ecef2aer-batch-vs-single

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2aer_batch` | 357.12 µs | 310.46 µs | 238.54 µs | 112.88 µs | 1744 | 10.3x |
| `test_ecef2aer_loop` | 3.66 ms | 3.63 ms | 3.41 ms | 146.20 µs | 282 | 1.0x |

### ecef2lla-batch-vs-pymap3d

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2lla_batch_rustmap3d` | 334.80 µs | 293.38 µs | 181.83 µs | 96.60 µs | 3229 | 1.6x |
| `test_ecef2geodetic_vectorized_pymap3d` | 541.86 µs | 532.69 µs | 504.67 µs | 38.43 µs | 978 | 1.0x |

### lla2ecef-batch-vs-pymap3d

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_lla2ecef_batch_rustmap3d` | 175.46 µs | 173.29 µs | 166.71 µs | 7.03 µs | 4877 | 1.3x |
| `test_geodetic2ecef_vectorized_pymap3d` | 234.15 µs | 230.50 µs | 216.46 µs | 13.87 µs | 2526 | 1.0x |

### ecef2aer-batch-vs-pymap3d

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2aer_vectorized_pymap3d` | 247.20 µs | 238.50 µs | 204.29 µs | 34.16 µs | 2050 | 1.3x |
| `test_ecef2aer_batch_rustmap3d` | 331.24 µs | 289.98 µs | 243.87 µs | 86.60 µs | 1794 | 1.0x |

### rustmap3d-ecef2lla-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2lla_batch_parallel` | 332.41 µs | 293.31 µs | 245.25 µs | 84.99 µs | 1798 | 1.8x |
| `test_ecef2lla_batch_serial` | 592.77 µs | 580.38 µs | 537.58 µs | 61.15 µs | 941 | 1.0x |

### rustmap3d-lla2ecef-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_lla2ecef_batch_serial` | 183.00 µs | 182.37 µs | 167.42 µs | 7.32 µs | 5110 | 1.9x |
| `test_lla2ecef_batch_parallel` | 343.28 µs | 326.46 µs | 228.83 µs | 82.85 µs | 2018 | 1.0x |

### rustmap3d-ecef2aer-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2aer_batch_parallel` | 343.53 µs | 303.83 µs | 254.71 µs | 85.03 µs | 1562 | 2.1x |
| `test_ecef2aer_batch_serial` | 717.02 µs | 718.21 µs | 679.29 µs | 16.65 µs | 1367 | 1.0x |

### single-ecef2lla

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2lla_rustmap3d` | 1.36 µs | 1.36 µs | 1.32 µs | 25.62 ns | 10 | 239.2x |
| `test_ecef2geodetic_pymap3d` | 325.49 µs | 327.12 µs | 309.99 µs | 10.98 µs | 10 | 1.0x |

### single-lla2ecef

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_lla2ecef_rustmap3d` | 581.93 ns | 579.52 ns | 556.46 ns | 16.36 ns | 10 | 43.4x |
| `test_geodetic2ecef_pymap3d` | 25.23 µs | 25.23 µs | 24.93 µs | 179.70 ns | 10 | 1.0x |

### single-vincenty-direct

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_vincenty_direct_rustmap3d` | 1.98 µs | 1.97 µs | 1.93 µs | 47.32 ns | 10 | 239.7x |
| `test_vreckon_pymap3d` | 474.54 µs | 472.75 µs | 467.04 µs | 6.17 µs | 10 | 1.0x |

### single-vincenty-inverse

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_vincenty_inverse_rustmap3d` | 3.40 µs | 3.37 µs | 3.32 µs | 77.49 ns | 10 | 278.4x |
| `test_vdist_pymap3d` | 946.27 µs | 944.15 µs | 940.36 µs | 4.92 µs | 10 | 1.0x |
