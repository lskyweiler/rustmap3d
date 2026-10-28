### single-point-ecef2lla

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2lla_rustmap3d` | 1.52 µs | 1.52 µs | 1.48 µs | 31.33 ns | 10 | 214.8x |
| `test_ecef2geodetic_pymap3d` | 326.20 µs | 321.15 µs | 318.17 µs | 9.91 µs | 10 | 1.0x |

### single-point-lla2ecef

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_lla2ecef_rustmap3d` | 596.44 ns | 593.46 ns | 571.17 ns | 19.63 ns | 10 | 43.3x |
| `test_geodetic2ecef_pymap3d` | 25.84 µs | 25.76 µs | 25.50 µs | 312.28 ns | 10 | 1.0x |

### single-point-vincenty-direct

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_vincenty_direct_rustmap3d` | 1.98 µs | 1.98 µs | 1.93 µs | 39.01 ns | 10 | 252.4x |
| `test_vreckon_pymap3d` | 499.84 µs | 497.48 µs | 489.10 µs | 9.00 µs | 10 | 1.0x |

### single-point-vincenty-inverse

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_vincenty_inverse_rustmap3d` | 3.57 µs | 3.60 µs | 3.42 µs | 73.69 ns | 10 | 273.5x |
| `test_vdist_pymap3d` | 975.73 µs | 974.69 µs | 961.35 µs | 11.70 µs | 10 | 1.0x |

### rustmap3d-ecef2lla-batch-vs-single

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2lla_batch` | 371.87 µs | 313.96 µs | 245.08 µs | 148.37 µs | 725 | 10.5x |
| `test_ecef2lla_loop` | 3.90 ms | 3.87 ms | 3.63 ms | 192.40 µs | 255 | 1.0x |

### rustmap3d-lla2ecef-batch-vs-single

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_lla2ecef_batch` | 180.14 µs | 178.67 µs | 168.00 µs | 8.77 µs | 4663 | 16.4x |
| `test_lla2ecef_loop` | 2.95 ms | 2.95 ms | 2.78 ms | 81.00 µs | 344 | 1.0x |

### rustmap3d-ecef2aer-batch-vs-single

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2aer_batch` | 362.10 µs | 323.54 µs | 243.33 µs | 105.81 µs | 2375 | 10.0x |
| `test_ecef2aer_loop` | 3.62 ms | 3.62 ms | 3.40 ms | 80.26 µs | 285 | 1.0x |

### ecef2lla-batch-vs-pymap3d

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2lla_batch_rustmap3d` | 389.76 µs | 363.60 µs | 249.71 µs | 119.79 µs | 2096 | 1.5x |
| `test_ecef2geodetic_vectorized_pymap3d` | 585.46 µs | 560.33 µs | 512.29 µs | 104.71 µs | 817 | 1.0x |

### lla2ecef-batch-vs-pymap3d

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_lla2ecef_batch_rustmap3d` | 179.25 µs | 178.62 µs | 166.17 µs | 7.64 µs | 4502 | 1.4x |
| `test_geodetic2ecef_vectorized_pymap3d` | 242.30 µs | 237.77 µs | 214.83 µs | 21.53 µs | 2658 | 1.0x |

### ecef2aer-batch-vs-pymap3d

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_ecef2aer_vectorized_pymap3d` | 280.15 µs | 272.04 µs | 200.37 µs | 41.62 µs | 2016 | 1.3x |
| `test_ecef2aer_batch_rustmap3d` | 352.89 µs | 313.92 µs | 241.79 µs | 98.35 µs | 1535 | 1.0x |

### rustmap3d-ecef2lla-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_batch_par_vs_serial[ecef2lla-parallel]` | 386.14 µs | 362.79 µs | 176.17 µs | 120.55 µs | 2103 | 1.6x |
| `test_batch_par_vs_serial[ecef2lla-serial]` | 618.69 µs | 599.13 µs | 529.12 µs | 79.26 µs | 926 | 1.0x |

### rustmap3d-lla2ecef-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_batch_par_vs_serial[lla2ecef-serial]` | 181.78 µs | 181.17 µs | 167.25 µs | 8.28 µs | 5245 | 2.0x |
| `test_batch_par_vs_serial[lla2ecef-parallel]` | 369.74 µs | 359.60 µs | 210.75 µs | 88.71 µs | 730 | 1.0x |

### rustmap3d-ecef2aer-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_batch_par_vs_serial[ecef2aer-parallel]` | 371.98 µs | 351.67 µs | 181.92 µs | 104.39 µs | 2097 | 1.9x |
| `test_batch_par_vs_serial[ecef2aer-serial]` | 721.48 µs | 721.92 µs | 678.12 µs | 18.91 µs | 1362 | 1.0x |

### rustmap3d-ecef2enu-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_batch_par_vs_serial[ecef2enu-serial]` | 224.29 µs | 222.04 µs | 204.29 µs | 17.88 µs | 4639 | 2.0x |
| `test_batch_par_vs_serial[ecef2enu-parallel]` | 449.79 µs | 441.50 µs | 221.58 µs | 96.32 µs | 2444 | 1.0x |

### rustmap3d-ecef2ned-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_batch_par_vs_serial[ecef2ned-serial]` | 315.52 µs | 313.96 µs | 284.13 µs | 11.61 µs | 2949 | 1.2x |
| `test_batch_par_vs_serial[ecef2ned-parallel]` | 388.95 µs | 377.67 µs | 233.42 µs | 110.87 µs | 2161 | 1.0x |

### rustmap3d-enu2ecef-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_batch_par_vs_serial[enu2ecef-serial]` | 218.28 µs | 215.13 µs | 198.87 µs | 19.85 µs | 4146 | 1.7x |
| `test_batch_par_vs_serial[enu2ecef-parallel]` | 377.34 µs | 368.54 µs | 226.25 µs | 95.99 µs | 2470 | 1.0x |

### rustmap3d-enu2lla-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_batch_par_vs_serial[enu2lla-parallel]` | 401.59 µs | 352.21 µs | 277.29 µs | 136.99 µs | 1471 | 3.4x |
| `test_batch_par_vs_serial[enu2lla-serial]` | 1.35 ms | 1.35 ms | 1.27 ms | 30.73 µs | 713 | 1.0x |

### rustmap3d-enu2aer-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_batch_par_vs_serial[enu2aer-serial]` | 250.95 µs | 247.37 µs | 203.08 µs | 27.61 µs | 2394 | 1.4x |
| `test_batch_par_vs_serial[enu2aer-parallel]` | 357.48 µs | 344.46 µs | 223.13 µs | 92.24 µs | 2179 | 1.0x |

### rustmap3d-aer2ecef-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_batch_par_vs_serial[aer2ecef-serial]` | 340.88 µs | 340.21 µs | 313.62 µs | 9.39 µs | 2998 | 1.1x |
| `test_batch_par_vs_serial[aer2ecef-parallel]` | 364.74 µs | 351.33 µs | 228.63 µs | 103.50 µs | 2535 | 1.0x |

### rustmap3d-aer2enu-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_batch_par_vs_serial[aer2enu-serial]` | 165.50 µs | 164.00 µs | 147.00 µs | 9.06 µs | 4845 | 2.2x |
| `test_batch_par_vs_serial[aer2enu-parallel]` | 369.62 µs | 355.06 µs | 199.33 µs | 111.19 µs | 2182 | 1.0x |

### rustmap3d-aer2ned-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_batch_par_vs_serial[aer2ned-serial]` | 161.35 µs | 159.46 µs | 145.58 µs | 10.23 µs | 4885 | 2.8x |
| `test_batch_par_vs_serial[aer2ned-parallel]` | 456.29 µs | 444.00 µs | 225.88 µs | 101.85 µs | 2106 | 1.0x |

### rustmap3d-lla2enu-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_batch_par_vs_serial[lla2enu-parallel]` | 355.52 µs | 340.58 µs | 223.46 µs | 94.05 µs | 2302 | 1.1x |
| `test_batch_par_vs_serial[lla2enu-serial]` | 396.67 µs | 395.79 µs | 369.46 µs | 9.11 µs | 2576 | 1.0x |

### rustmap3d-lla2ned-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_batch_par_vs_serial[lla2ned-parallel]` | 375.34 µs | 359.85 µs | 160.54 µs | 117.77 µs | 1724 | 1.1x |
| `test_batch_par_vs_serial[lla2ned-serial]` | 395.69 µs | 394.79 µs | 367.62 µs | 9.19 µs | 2440 | 1.0x |

### rustmap3d-ned2ecef-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_batch_par_vs_serial[ned2ecef-serial]` | 296.20 µs | 294.33 µs | 271.46 µs | 8.96 µs | 3507 | 1.2x |
| `test_batch_par_vs_serial[ned2ecef-parallel]` | 366.58 µs | 353.88 µs | 232.29 µs | 99.19 µs | 2090 | 1.0x |

### rustmap3d-ned2lla-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_batch_par_vs_serial[ned2lla-parallel]` | 371.82 µs | 321.44 µs | 282.63 µs | 97.55 µs | 1950 | 4.0x |
| `test_batch_par_vs_serial[ned2lla-serial]` | 1.48 ms | 1.48 ms | 1.39 ms | 31.31 µs | 654 | 1.0x |

### rustmap3d-ned2aer-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_batch_par_vs_serial[ned2aer-serial]` | 260.84 µs | 256.08 µs | 203.83 µs | 29.25 µs | 2380 | 1.4x |
| `test_batch_par_vs_serial[ned2aer-parallel]` | 371.86 µs | 356.85 µs | 218.29 µs | 111.00 µs | 1820 | 1.0x |

### rustmap3d-aer2ecef_uvw-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_batch_par_vs_serial[aer2ecef_uvw-serial]` | 271.55 µs | 269.71 µs | 248.62 µs | 9.63 µs | 3951 | 1.4x |
| `test_batch_par_vs_serial[aer2ecef_uvw-parallel]` | 382.86 µs | 372.54 µs | 208.21 µs | 97.36 µs | 1837 | 1.0x |

### rustmap3d-enu2ecef_uvw-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_batch_par_vs_serial[enu2ecef_uvw-serial]` | 152.90 µs | 151.42 µs | 139.92 µs | 6.19 µs | 6583 | 2.3x |
| `test_batch_par_vs_serial[enu2ecef_uvw-parallel]` | 351.83 µs | 343.17 µs | 213.00 µs | 88.15 µs | 2979 | 1.0x |

### rustmap3d-ned2ecef_uvw-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_batch_par_vs_serial[ned2ecef_uvw-serial]` | 241.70 µs | 238.79 µs | 220.63 µs | 17.10 µs | 4337 | 1.5x |
| `test_batch_par_vs_serial[ned2ecef_uvw-parallel]` | 357.25 µs | 342.75 µs | 225.71 µs | 95.99 µs | 2210 | 1.0x |

### rustmap3d-ecef_uvw2aer-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_batch_par_vs_serial[ecef_uvw2aer-parallel]` | 352.82 µs | 327.83 µs | 235.46 µs | 95.36 µs | 1767 | 1.6x |
| `test_batch_par_vs_serial[ecef_uvw2aer-serial]` | 556.95 µs | 551.13 µs | 494.62 µs | 26.50 µs | 1439 | 1.0x |

### rustmap3d-ecef_uvw2enu-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_batch_par_vs_serial[ecef_uvw2enu-serial]` | 152.19 µs | 151.50 µs | 140.87 µs | 4.87 µs | 6921 | 2.3x |
| `test_batch_par_vs_serial[ecef_uvw2enu-parallel]` | 351.15 µs | 340.96 µs | 223.42 µs | 87.10 µs | 2169 | 1.0x |

### rustmap3d-ecef_uvw2ned-batch-par-vs-serial

| Benchmark | Mean | Median | Min | StdDev | Rounds | vs. slowest |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `test_batch_par_vs_serial[ecef_uvw2ned-serial]` | 227.87 µs | 227.00 µs | 211.67 µs | 6.69 µs | 4646 | 1.6x |
| `test_batch_par_vs_serial[ecef_uvw2ned-parallel]` | 355.90 µs | 345.71 µs | 230.71 µs | 92.13 µs | 1969 | 1.0x |
