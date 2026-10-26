use criterion::{criterion_group, criterion_main, Criterion};
use map3d;
use std::hint::black_box;

fn bench_geo(c: &mut Criterion) {
    c.bench_function(
        "ecef2lla_ferrari",
        |b: &mut criterion::Bencher<criterion::measurement::WallTime>| {
            b.iter(|| {
                let point = map3d::rand_ecef();
                map3d::ecef2lla_ferarri(black_box(&point));
            })
        },
    );
    c.bench_function(
        "ecef2lla_map3d",
        |b: &mut criterion::Bencher<criterion::measurement::WallTime>| {
            b.iter(|| {
                let point = map3d::rand_ecef();
                map3d::ecef2lla_map3d(black_box(&point));
            })
        },
    );
    c.bench_function(
        "lla2ecef",
        |b: &mut criterion::Bencher<criterion::measurement::WallTime>| {
            b.iter(|| {
                let point = map3d::rand_lla();
                map3d::lla2ecef(black_box(&point));
            })
        },
    );
    c.bench_function(
        "ecef2enu_quat",
        |b: &mut criterion::Bencher<criterion::measurement::WallTime>| {
            b.iter(|| {
                let point = map3d::rand_lla();
                map3d::ecef2enu_quat(black_box((point.x, point.y)));
            })
        },
    );
    c.bench_function(
        "enu2ecef_quat",
        |b: &mut criterion::Bencher<criterion::measurement::WallTime>| {
            b.iter(|| {
                let point = map3d::rand_lla();
                map3d::enu2ecef_quat(black_box((point.x, point.y)));
            })
        },
    );
    c.bench_function(
        "ecef2enu",
        |b: &mut criterion::Bencher<criterion::measurement::WallTime>| {
            b.iter(|| {
                let ref_point = map3d::rand_lla();
                let point = map3d::rand_ecef();
                map3d::ecef_uvw2enu(&point, &ref_point);
            })
        },
    );
    c.bench_function(
        "enu2ecef",
        |b: &mut criterion::Bencher<criterion::measurement::WallTime>| {
            b.iter(|| {
                let ref_point = map3d::rand_lla();
                let point = map3d::rand_ecef();
                map3d::enu2ecef_uvw(&point, &ref_point);
            })
        },
    );
    c.bench_function(
        "ecef2rae",
        |b: &mut criterion::Bencher<criterion::measurement::WallTime>| {
            b.iter(|| {
                let ref_point = map3d::rand_lla();
                let point = map3d::rand_ecef();
                map3d::ecef_uvw2aer(&point, &ref_point);
            })
        },
    );
    c.bench_function(
        "rae2ecef",
        |b: &mut criterion::Bencher<criterion::measurement::WallTime>| {
            b.iter(|| {
                let ref_point = map3d::rand_lla();
                let point = map3d::rand_ecef();
                map3d::aer2ecef_uvw(&point, &ref_point);
            })
        },
    );
}

const BATCH_N: usize = 10_000;

/// Batched transforms against a loop of single-point calls over the same `BATCH_N` points.
fn bench_batch(c: &mut Criterion) {
    let ecefs: Vec<glam::DVec3> = (0..BATCH_N).map(|_| map3d::rand_ecef()).collect();
    let llas: Vec<glam::DVec3> = (0..BATCH_N).map(|_| map3d::rand_lla()).collect();
    let lla_ref = glam::DVec3::new(38.9, -77.0, 120.0);
    let mut out = vec![glam::DVec3::ZERO; BATCH_N];

    let mut group = c.benchmark_group("batch_10k");
    group.throughput(criterion::Throughput::Elements(BATCH_N as u64));
    group.bench_function("ecef2lla_loop", |b| {
        b.iter(|| {
            for (o, v) in out.iter_mut().zip(&ecefs) {
                *o = map3d::ecef2lla(black_box(v));
            }
        })
    });
    group.bench_function("ecef2lla_batch_into", |b| {
        b.iter(|| map3d::ecef2lla_batch_into(black_box(&ecefs), &mut out).unwrap())
    });
    group.bench_function("lla2ecef_loop", |b| {
        b.iter(|| {
            for (o, v) in out.iter_mut().zip(&llas) {
                *o = map3d::lla2ecef(black_box(v));
            }
        })
    });
    group.bench_function("lla2ecef_batch_into", |b| {
        b.iter(|| map3d::lla2ecef_batch_into(black_box(&llas), &mut out).unwrap())
    });
    group.bench_function("ecef2aer_loop", |b| {
        b.iter(|| {
            for (o, v) in out.iter_mut().zip(&ecefs) {
                *o = map3d::ecef2aer(black_box(v), black_box(lla_ref));
            }
        })
    });
    group.bench_function("ecef2aer_batch_into", |b| {
        b.iter(|| {
            map3d::ecef2aer_batch_into(black_box(&ecefs), black_box(lla_ref), &mut out).unwrap()
        })
    });
    group.finish();
}

criterion_group!(benches, bench_geo, bench_batch);
criterion_main!(benches);
