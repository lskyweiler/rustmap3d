use criterion::{black_box, criterion_group, criterion_main, Criterion};

use map3d;
extern crate nalgebra_glm as glm;

fn bench_geo(c: &mut Criterion) {
    c.bench_function(
        "ecef2lla_ferrari",
        |b: &mut criterion::Bencher<criterion::measurement::WallTime>| {
            b.iter(|| {
                let point = map3d::geo::rand_ecef(None, None, None, None, None, None);
                map3d::geo::ecef2lla_ferarri(black_box(&point));
            })
        },
    );
    c.bench_function(
        "ecef2lla_map3d",
        |b: &mut criterion::Bencher<criterion::measurement::WallTime>| {
            b.iter(|| {
                let point = map3d::geo::rand_ecef(None, None, None, None, None, None);
                map3d::geo::ecef2lla_map3d(black_box(&point));
            })
        },
    );
    c.bench_function(
        "lla2ecef",
        |b: &mut criterion::Bencher<criterion::measurement::WallTime>| {
            b.iter(|| {
                let point = map3d::geo::rand_lla(None, None, None, None, None, None);
                map3d::geo::lla2ecef(black_box(&point));
            })
        },
    );
    c.bench_function(
        "ecef2enu_quat",
        |b: &mut criterion::Bencher<criterion::measurement::WallTime>| {
            b.iter(|| {
                let point = map3d::geo::rand_lla(None, None, None, None, None, None);
                map3d::geo::ecef2enu_quat(black_box(point.x), black_box(point.y));
            })
        },
    );
    c.bench_function(
        "enu2ecef_quat",
        |b: &mut criterion::Bencher<criterion::measurement::WallTime>| {
            b.iter(|| {
                let point = map3d::geo::rand_lla(None, None, None, None, None, None);
                map3d::geo::enu2ecef_quat(black_box(point.x), black_box(point.y));
            })
        },
    );
    c.bench_function(
        "ecef2enu",
        |b: &mut criterion::Bencher<criterion::measurement::WallTime>| {
            b.iter(|| {
                let ref_point = map3d::geo::rand_lla(None, None, None, None, None, None);
                let point = map3d::geo::rand_ecef(None, None, None, None, None, None);
                map3d::geo::ecef2enu(&point, &ref_point.xy());
            })
        },
    );
    c.bench_function(
        "enu2ecef",
        |b: &mut criterion::Bencher<criterion::measurement::WallTime>| {
            b.iter(|| {
                let ref_point = map3d::geo::rand_lla(None, None, None, None, None, None);
                let point = map3d::geo::rand_ecef(None, None, None, None, None, None);
                map3d::geo::enu2ecef(&point, &ref_point.xy());
            })
        },
    );
    c.bench_function(
        "ecef2rae",
        |b: &mut criterion::Bencher<criterion::measurement::WallTime>| {
            b.iter(|| {
                let ref_point = map3d::geo::rand_lla(None, None, None, None, None, None);
                let point = map3d::geo::rand_ecef(None, None, None, None, None, None);
                map3d::geo::ecef2rae(&point, &ref_point.xy());
            })
        },
    );
    c.bench_function(
        "rae2ecef",
        |b: &mut criterion::Bencher<criterion::measurement::WallTime>| {
            b.iter(|| {
                let ref_point = map3d::geo::rand_lla(None, None, None, None, None, None);
                let point = map3d::geo::rand_ecef(None, None, None, None, None, None);
                map3d::geo::rae2ecef(&point, &ref_point);
            })
        },
    );
}

criterion_group!(benches, bench_geo);
criterion_main!(benches);
