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

criterion_group!(benches, bench_geo);
criterion_main!(benches);
