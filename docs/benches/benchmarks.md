### rustmap3d-ecef2lla-batch-vs-single

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2lla_batch` | 390.52 µs | 374.50 µs | 249.88 µs | 106.48 µs | 841 | 9.7x |
| `test_ecef2lla_loop` | 3.78 ms | 3.76 ms | 3.58 ms | 96.77 µs | 257 | 1.0x |

### rustmap3d-lla2ecef-batch-vs-single

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_lla2ecef_batch` | 357.75 µs | 348.63 µs | 165.00 µs | 90.18 µs | 2207 | 8.2x |
| `test_lla2ecef_loop` | 2.94 ms | 2.94 ms | 2.76 ms | 83.01 µs | 314 | 1.0x |

### rustmap3d-ecef2aer-batch-vs-single

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2aer_batch` | 356.05 µs | 319.00 µs | 243.37 µs | 96.97 µs | 1711 | 10.2x |
| `test_ecef2aer_loop` | 3.62 ms | 3.62 ms | 3.43 ms | 79.31 µs | 273 | 1.0x |

### ecef2lla-batch-vs-pymap3d

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2lla_batch_rustmap3d` | 375.83 µs | 347.77 µs | 183.04 µs | 104.93 µs | 1982 | 1.5x |
| `test_ecef2geodetic_vectorized_pymap3d` | 564.15 µs | 553.54 µs | 512.17 µs | 43.42 µs | 958 | 1.0x |

### lla2ecef-batch-vs-pymap3d

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_geodetic2ecef_vectorized_pymap3d` | 232.46 µs | 230.00 µs | 213.21 µs | 15.71 µs | 2484 | 1.7x |
| `test_lla2ecef_batch_rustmap3d` | 396.08 µs | 367.79 µs | 178.08 µs | 333.48 µs | 1909 | 1.0x |

### ecef2aer-batch-vs-pymap3d

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2aer_vectorized_pymap3d` | 288.38 µs | 276.21 µs | 231.67 µs | 50.83 µs | 2078 | 1.3x |
| `test_ecef2aer_batch_rustmap3d` | 370.61 µs | 338.12 µs | 245.46 µs | 107.10 µs | 1048 | 1.0x |

### rustmap3d-ecef2lla-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2lla_batch_parallel` | 360.82 µs | 318.50 µs | 245.92 µs | 97.64 µs | 1029 | 2.1x |
| `test_ecef2lla_batch_serial` | 755.35 µs | 722.37 µs | 663.54 µs | 122.98 µs | 852 | 1.0x |

### rustmap3d-lla2ecef-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_lla2ecef_batch_serial` | 199.15 µs | 198.71 µs | 180.17 µs | 7.58 µs | 4477 | 1.9x |
| `test_lla2ecef_batch_parallel` | 369.44 µs | 359.79 µs | 225.54 µs | 92.76 µs | 2546 | 1.0x |

### rustmap3d-ecef2aer-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2aer_batch_parallel` | 422.97 µs | 407.81 µs | 235.25 µs | 130.76 µs | 1722 | 1.8x |
| `test_ecef2aer_batch_serial` | 759.59 µs | 751.58 µs | 706.75 µs | 45.25 µs | 1302 | 1.0x |

### single-ecef2lla

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2lla_rustmap3d` | 1.37 µs | 1.38 µs | 1.29 µs | 32.60 ns | 10 | 235.3x |
| `test_ecef2geodetic_pymap3d` | 321.57 µs | 321.26 µs | 316.43 µs | 4.04 µs | 10 | 1.0x |

### single-lla2ecef

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_lla2ecef_rustmap3d` | 765.99 ns | 717.27 ns | 697.00 ns | 110.49 ns | 10 | 34.8x |
| `test_geodetic2ecef_pymap3d` | 26.67 µs | 26.31 µs | 25.94 µs | 1.15 µs | 10 | 1.0x |

### single-vincenty-direct

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_vincenty_direct_rustmap3d` | 2.08 µs | 2.06 µs | 1.99 µs | 69.76 ns | 10 | 234.6x |
| `test_vreckon_pymap3d` | 486.82 µs | 488.98 µs | 478.08 µs | 6.26 µs | 10 | 1.0x |

### single-vincenty-inverse

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_vincenty_inverse_rustmap3d` | 3.42 µs | 3.42 µs | 3.33 µs | 87.38 ns | 10 | 293.6x |
| `test_vdist_pymap3d` | 1.01 ms | 983.22 µs | 964.21 µs | 53.51 µs | 10 | 1.0x |
