### rustmap3d-ecef2lla-batch-vs-single

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2lla_batch` | 359.87 µs | 319.21 µs | 243.13 µs | 95.88 µs | 1491 | 10.7x |
| `test_ecef2lla_loop` | 3.84 ms | 3.74 ms | 3.59 ms | 282.19 µs | 259 | 1.0x |

### rustmap3d-lla2ecef-batch-vs-single

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_lla2ecef_batch` | 361.94 µs | 352.90 µs | 216.67 µs | 92.98 µs | 1616 | 8.0x |
| `test_lla2ecef_loop` | 2.89 ms | 2.88 ms | 2.72 ms | 89.76 µs | 343 | 1.0x |

### rustmap3d-ecef2aer-batch-vs-single

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2aer_batch` | 445.60 µs | 429.37 µs | 241.50 µs | 141.15 µs | 1927 | 8.3x |
| `test_ecef2aer_loop` | 3.70 ms | 3.71 ms | 3.41 ms | 155.27 µs | 289 | 1.0x |

### ecef2lla-batch-vs-pymap3d

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2lla_batch_rustmap3d` | 383.50 µs | 367.75 µs | 238.25 µs | 107.00 µs | 2797 | 1.5x |
| `test_ecef2geodetic_vectorized_pymap3d` | 560.36 µs | 549.44 µs | 510.50 µs | 46.15 µs | 572 | 1.0x |

### lla2ecef-batch-vs-pymap3d

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_geodetic2ecef_vectorized_pymap3d` | 257.02 µs | 249.96 µs | 223.33 µs | 21.38 µs | 2577 | 1.5x |
| `test_lla2ecef_batch_rustmap3d` | 388.32 µs | 378.33 µs | 225.58 µs | 92.68 µs | 2174 | 1.0x |

### ecef2aer-batch-vs-pymap3d

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2aer_vectorized_pymap3d` | 308.34 µs | 306.25 µs | 240.42 µs | 38.99 µs | 1969 | 1.5x |
| `test_ecef2aer_batch_rustmap3d` | 473.61 µs | 460.96 µs | 225.37 µs | 95.15 µs | 1703 | 1.0x |

### rustmap3d-ecef2lla-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2lla_batch_parallel` | 403.27 µs | 379.46 µs | 247.33 µs | 136.15 µs | 1562 | 1.9x |
| `test_ecef2lla_batch_serial` | 766.44 µs | 749.46 µs | 647.17 µs | 85.77 µs | 798 | 1.0x |

### rustmap3d-lla2ecef-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_lla2ecef_batch_serial` | 200.19 µs | 199.96 µs | 180.46 µs | 7.63 µs | 4549 | 1.8x |
| `test_lla2ecef_batch_parallel` | 364.13 µs | 355.50 µs | 223.46 µs | 93.92 µs | 2519 | 1.0x |

### rustmap3d-ecef2aer-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2aer_batch_parallel` | 363.58 µs | 325.04 µs | 236.83 µs | 102.85 µs | 2049 | 2.1x |
| `test_ecef2aer_batch_serial` | 746.75 µs | 747.10 µs | 705.96 µs | 14.05 µs | 1304 | 1.0x |

### single-ecef2lla

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2lla_rustmap3d` | 1.36 µs | 1.35 µs | 1.31 µs | 30.54 ns | 10 | 233.6x |
| `test_ecef2geodetic_pymap3d` | 316.64 µs | 316.75 µs | 311.00 µs | 3.81 µs | 10 | 1.0x |

### single-lla2ecef

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_lla2ecef_rustmap3d` | 598.84 ns | 600.83 ns | 575.29 ns | 11.52 ns | 10 | 42.4x |
| `test_geodetic2ecef_pymap3d` | 25.41 µs | 25.35 µs | 25.00 µs | 320.06 ns | 10 | 1.0x |

### single-vincenty-direct

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_vincenty_direct_rustmap3d` | 2.02 µs | 2.00 µs | 1.95 µs | 72.03 ns | 10 | 241.2x |
| `test_vreckon_pymap3d` | 487.94 µs | 486.76 µs | 476.85 µs | 7.96 µs | 10 | 1.0x |

### single-vincenty-inverse

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_vincenty_inverse_rustmap3d` | 3.42 µs | 3.41 µs | 3.33 µs | 89.37 ns | 10 | 284.4x |
| `test_vdist_pymap3d` | 973.59 µs | 969.79 µs | 964.33 µs | 10.96 µs | 10 | 1.0x |
