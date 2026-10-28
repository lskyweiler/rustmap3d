### single

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_lla2ecef_rustmap3d` | 600.88 ns | 603.21 ns | 581.50 ns | 9.09 ns | 10 | 567.8x |
| `test_ecef2lla_rustmap3d` | 1.86 µs | 1.83 µs | 1.70 µs | 110.79 ns | 10 | 183.4x |
| `test_geodetic2ecef_pymap3d` | 26.14 µs | 26.15 µs | 25.61 µs | 334.18 ns | 10 | 13.1x |
| `test_ecef2geodetic_pymap3d` | 341.18 µs | 332.21 µs | 318.92 µs | 22.11 µs | 10 | 1.0x |

### batch_vs_pymap3d

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_lla2ecef_batch_rustmap3d` | 200.73 µs | 192.58 µs | 180.25 µs | 72.28 µs | 3836 | 4.0x |
| `test_geodetic2ecef_vectorized_pymap3d` | 236.05 µs | 229.46 µs | 209.17 µs | 38.21 µs | 2565 | 3.4x |
| `test_ecef2aer_vectorized_pymap3d` | 314.29 µs | 299.29 µs | 242.04 µs | 43.70 µs | 1900 | 2.6x |
| `test_ecef2aer_batch_rustmap3d` | 374.12 µs | 365.96 µs | 325.42 µs | 32.60 µs | 1827 | 2.2x |
| `test_ecef2geodetic_vectorized_pymap3d` | 613.36 µs | 586.44 µs | 520.21 µs | 104.90 µs | 964 | 1.3x |
| `test_ecef2lla_batch_rustmap3d` | 812.86 µs | 754.02 µs | 614.79 µs | 144.79 µs | 674 | 1.0x |

### batch_vs_single

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_lla2ecef_batch` | 394.61 µs | 387.08 µs | 217.42 µs | 91.72 µs | 2355 | 10.1x |
| `test_ecef2aer_batch` | 402.99 µs | 397.33 µs | 239.38 µs | 97.42 µs | 1757 | 9.8x |
| `test_ecef2lla_batch` | 416.45 µs | 403.25 µs | 268.42 µs | 113.51 µs | 1268 | 9.5x |
| `test_lla2ecef_loop` | 2.97 ms | 2.98 ms | 2.75 ms | 68.74 µs | 330 | 1.3x |
| `test_ecef2aer_loop` | 3.74 ms | 3.70 ms | 3.49 ms | 260.15 µs | 269 | 1.1x |
| `test_ecef2lla_loop` | 3.97 ms | 3.93 ms | 3.66 ms | 571.36 µs | 251 | 1.0x |

### batch_parallel

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_lla2ecef_batch_serial` | 196.98 µs | 195.71 µs | 181.04 µs | 7.90 µs | 4442 | 3.6x |
| `test_ecef2aer_batch_serial` | 373.95 µs | 368.06 µs | 303.00 µs | 41.76 µs | 1746 | 1.9x |
| `test_lla2ecef_batch_parallel` | 423.82 µs | 415.33 µs | 227.04 µs | 99.09 µs | 2310 | 1.7x |
| `test_ecef2lla_batch_parallel` | 428.01 µs | 420.58 µs | 220.83 µs | 106.41 µs | 1964 | 1.7x |
| `test_ecef2aer_batch_parallel` | 442.38 µs | 431.62 µs | 242.42 µs | 107.57 µs | 2856 | 1.6x |
| `test_ecef2lla_batch_serial` | 717.96 µs | 689.96 µs | 570.96 µs | 108.39 µs | 741 | 1.0x |

### vincenty

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_vincenty_direct_rustmap3d` | 2.14 µs | 2.13 µs | 2.11 µs | 31.01 ns | 10 | 456.0x |
| `test_vincenty_inverse_rustmap3d` | 3.61 µs | 3.60 µs | 3.56 µs | 58.92 ns | 10 | 270.5x |
| `test_vreckon_pymap3d` | 496.32 µs | 496.86 µs | 475.65 µs | 11.31 µs | 10 | 2.0x |
| `test_vdist_pymap3d` | 977.79 µs | 981.54 µs | 949.47 µs | 16.81 µs | 10 | 1.0x |
