#!/usr/bin/env python3
"""Generate and validate smooth, watertight Go2-W ramp meshes."""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import math
import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class Lane:
    angle_deg: int
    y_m: float


@dataclass
class RampGeometry:
    lane: Lane
    transition_length_m: float
    points: list[tuple[float, float]]
    facet_angles_rad: list[float]
    section_counts: tuple[int, int, int, int]
    vertices: list[tuple[float, float, float]]
    faces: list[tuple[int, int, int]]
    triangle_count: int
    maximum_adjacent_slope_change_deg: float


LANES = (
    Lane(15, -6.0),
    Lane(30, -2.0),
    Lane(35, 2.0),
    Lane(45, 6.0),
)
ENTRY_X_M = 1.5
RAMP_WIDTH_M = 2.5
CONSTANT_SLOPE_LENGTH_M = 5.0
TOP_PLATFORM_LENGTH_M = 2.0
MESH_THICKNESS_M = 0.12
APPROACH_MARKER_CENTER_X_M = -1.25
APPROACH_MARKER_LENGTH_M = 5.5
EXPECTED_FRICTION = (1.0, 1.0)


class Checks:
    def __init__(self) -> None:
        self.count = 0
        self.failures: list[str] = []

    def check(self, condition: bool, label: str) -> None:
        self.count += 1
        if not condition:
            self.failures.append(label)


def smoothstep5(u: float) -> float:
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


def smoothstep5_derivative(u: float) -> float:
    return 30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4


def transition_length(
    angle_rad: float,
    design_speed_mps: float,
    maximum_normal_accel_mps2: float,
    minimum_transition_length_m: float,
) -> float:
    acceleration_length = (
        design_speed_mps**2 * angle_rad * 1.875 / maximum_normal_accel_mps2
    )
    return max(minimum_transition_length_m, acceleration_length)


def append_arc_section(
    points: list[tuple[float, float]],
    facet_angles: list[float],
    length_m: float,
    resolution_m: float,
    angle_at_u,
) -> int:
    count = max(1, math.ceil(length_m / resolution_m))
    ds = length_m / count
    x, z = points[-1]
    for index in range(count):
        u_mid = (index + 0.5) / count
        theta = angle_at_u(u_mid)
        x += math.cos(theta) * ds
        z += math.sin(theta) * ds
        points.append((x, z))
        facet_angles.append(theta)
    return count


def append_straight_section(
    points: list[tuple[float, float]],
    facet_angles: list[float],
    length_m: float,
    resolution_m: float,
    theta: float,
) -> int:
    return append_arc_section(
        points, facet_angles, length_m, resolution_m, lambda _u: theta
    )


def extrude_watertight_mesh(
    points: list[tuple[float, float]], width_m: float, lateral_resolution_m: float
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    lateral_count = max(1, math.ceil(width_m / lateral_resolution_m))
    lateral_step = width_m / lateral_count
    y_values = [
        -0.5 * width_m + index * lateral_step
        for index in range(lateral_count + 1)
    ]
    row_size = len(y_values)

    top_vertices = [(x, y, z) for x, z in points for y in y_values]
    bottom_vertices = [(x, y, -MESH_THICKNESS_M) for x, _z in points for y in y_values]
    vertices = top_vertices + bottom_vertices
    top_count = len(top_vertices)

    def top(i: int, j: int) -> int:
        return i * row_size + j

    def bottom(i: int, j: int) -> int:
        return top_count + i * row_size + j

    faces: list[tuple[int, int, int]] = []
    longitudinal_count = len(points) - 1

    for i in range(longitudinal_count):
        for j in range(lateral_count):
            a, b = top(i, j), top(i + 1, j)
            c, d = top(i + 1, j + 1), top(i, j + 1)
            faces.extend(((a, b, c), (a, c, d)))

            a, b = bottom(i, j), bottom(i + 1, j)
            c, d = bottom(i + 1, j + 1), bottom(i, j + 1)
            faces.extend(((a, d, c), (a, c, b)))

        top_left, next_top_left = top(i, 0), top(i + 1, 0)
        bottom_left, next_bottom_left = bottom(i, 0), bottom(i + 1, 0)
        faces.extend(
            (
                (top_left, bottom_left, next_bottom_left),
                (top_left, next_bottom_left, next_top_left),
            )
        )

        top_right, next_top_right = top(i, lateral_count), top(i + 1, lateral_count)
        bottom_right, next_bottom_right = (
            bottom(i, lateral_count),
            bottom(i + 1, lateral_count),
        )
        faces.extend(
            (
                (top_right, next_top_right, next_bottom_right),
                (top_right, next_bottom_right, bottom_right),
            )
        )

    last = len(points) - 1
    for j in range(lateral_count):
        faces.extend(
            (
                (top(0, j), top(0, j + 1), bottom(0, j + 1)),
                (top(0, j), bottom(0, j + 1), bottom(0, j)),
                (top(last, j), bottom(last, j), bottom(last, j + 1)),
                (top(last, j), bottom(last, j + 1), top(last, j + 1)),
            )
        )

    return vertices, faces


def generate_ramp(
    lane: Lane,
    design_speed_mps: float,
    maximum_normal_accel_mps2: float,
    minimum_transition_length_m: float,
    longitudinal_resolution_m: float,
    lateral_resolution_m: float,
) -> RampGeometry:
    target = math.radians(lane.angle_deg)
    length = transition_length(
        target,
        design_speed_mps,
        maximum_normal_accel_mps2,
        minimum_transition_length_m,
    )
    points = [(0.0, 0.0)]
    facet_angles: list[float] = []
    entry_count = append_arc_section(
        points,
        facet_angles,
        length,
        longitudinal_resolution_m,
        lambda u: target * smoothstep5(u),
    )
    constant_count = append_straight_section(
        points,
        facet_angles,
        CONSTANT_SLOPE_LENGTH_M,
        longitudinal_resolution_m,
        target,
    )
    crest_count = append_arc_section(
        points,
        facet_angles,
        length,
        longitudinal_resolution_m,
        lambda u: target * (1.0 - smoothstep5(u)),
    )
    platform_count = append_straight_section(
        points,
        facet_angles,
        TOP_PLATFORM_LENGTH_M,
        longitudinal_resolution_m,
        0.0,
    )
    vertices, faces = extrude_watertight_mesh(
        points, RAMP_WIDTH_M, lateral_resolution_m
    )
    adjacent_change = max(
        abs(math.degrees(second - first))
        for first, second in zip(facet_angles, facet_angles[1:])
    )
    return RampGeometry(
        lane=lane,
        transition_length_m=length,
        points=points,
        facet_angles_rad=facet_angles,
        section_counts=(entry_count, constant_count, crest_count, platform_count),
        vertices=vertices,
        faces=faces,
        triangle_count=len(faces),
        maximum_adjacent_slope_change_deg=adjacent_change,
    )


def triangle_cross(
    vertices: list[tuple[float, float, float]], face: tuple[int, int, int]
) -> tuple[float, float, float]:
    a, b, c = (vertices[index] for index in face)
    ab = (b[0] - a[0], b[1] - a[1], b[2] - a[2])
    ac = (c[0] - a[0], c[1] - a[1], c[2] - a[2])
    return (
        ab[1] * ac[2] - ab[2] * ac[1],
        ab[2] * ac[0] - ab[0] * ac[2],
        ab[0] * ac[1] - ab[1] * ac[0],
    )


def validate_geometry(
    geometry: RampGeometry,
    checks: Checks,
    design_speed_mps: float,
    maximum_normal_accel_mps2: float,
    minimum_transition_length_m: float,
) -> None:
    angle = math.radians(geometry.lane.angle_deg)
    label = f"{geometry.lane.angle_deg}deg"
    entry_count, constant_count, crest_count, platform_count = geometry.section_counts
    entry_angles = geometry.facet_angles_rad[:entry_count]
    constant_angles = geometry.facet_angles_rad[
        entry_count : entry_count + constant_count
    ]
    crest_start = entry_count + constant_count
    crest_angles = geometry.facet_angles_rad[crest_start : crest_start + crest_count]
    platform_angles = geometry.facet_angles_rad[-platform_count:]

    endpoint_tolerance = 2.0e-7
    checks.check(abs(entry_angles[0]) < endpoint_tolerance, f"{label}: entry start slope")
    checks.check(abs(entry_angles[-1] - angle) < endpoint_tolerance, f"{label}: entry end slope")
    checks.check(abs(crest_angles[0] - angle) < endpoint_tolerance, f"{label}: crest start slope")
    checks.check(abs(crest_angles[-1]) < endpoint_tolerance, f"{label}: crest end slope")
    checks.check(smoothstep5_derivative(0.0) == 0.0, f"{label}: entry start curvature")
    checks.check(smoothstep5_derivative(1.0) == 0.0, f"{label}: entry end curvature")
    checks.check(smoothstep5_derivative(0.0) == 0.0, f"{label}: crest start curvature")
    checks.check(smoothstep5_derivative(1.0) == 0.0, f"{label}: crest end curvature")
    checks.check(
        all(b[0] > a[0] for a, b in zip(geometry.points, geometry.points[1:])),
        f"{label}: monotonic x",
    )
    checks.check(
        all(b[1] >= a[1] for a, b in zip(geometry.points, geometry.points[1:])),
        f"{label}: monotonic height",
    )
    checks.check(
        all(abs(value - angle) < 1.0e-14 for value in constant_angles),
        f"{label}: exact constant slope",
    )
    checks.check(
        all(abs(value) < 1.0e-14 for value in platform_angles),
        f"{label}: horizontal platform",
    )
    required_length = transition_length(
        angle,
        design_speed_mps,
        maximum_normal_accel_mps2,
        minimum_transition_length_m,
    )
    checks.check(
        geometry.transition_length_m + 1.0e-12 >= required_length,
        f"{label}: transition acceleration length",
    )

    edge_counts: Counter[tuple[int, int]] = Counter()
    minimum_area_squared = math.inf
    for face in geometry.faces:
        for first, second in ((face[0], face[1]), (face[1], face[2]), (face[2], face[0])):
            edge_counts[tuple(sorted((first, second)))] += 1
        cross = triangle_cross(geometry.vertices, face)
        minimum_area_squared = min(minimum_area_squared, sum(value * value for value in cross))
    checks.check(all(count == 2 for count in edge_counts.values()), f"{label}: watertight mesh")
    checks.check(minimum_area_squared > 1.0e-20, f"{label}: no zero-area triangles")
    checks.check(
        all(math.isfinite(value) for vertex in geometry.vertices for value in vertex),
        f"{label}: finite vertices",
    )
    checks.check(
        all(
            b[0] > a[0] and b[1] >= a[1]
            for a, b in zip(geometry.points, geometry.points[1:])
        )
        and RAMP_WIDTH_M > 0.0
        and MESH_THICKNESS_M > 0.0,
        f"{label}: no profile/extrusion self-intersection",
    )

    top_vertex_count = len(geometry.vertices) // 2
    top_faces = [
        face for face in geometry.faces if all(index < top_vertex_count for index in face)
    ]
    checks.check(
        bool(top_faces)
        and all(triangle_cross(geometry.vertices, face)[2] > 0.0 for face in top_faces),
        f"{label}: top normals upward",
    )


def write_obj(path: Path, geometry: RampGeometry, force: bool) -> None:
    if path.exists() and not force:
        raise FileExistsError(f"refusing to overwrite {path}; pass --force")
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        mode="w", encoding="utf-8", dir=path.parent, delete=False
    ) as stream:
        temporary_path = Path(stream.name)
        stream.write(
            f"# Deterministic smooth Go2-W {geometry.lane.angle_deg} degree ramp\n"
        )
        for x, y, z in geometry.vertices:
            stream.write(f"v {x:.9f} {y:.9f} {z:.9f}\n")
        for face in geometry.faces:
            cross = triangle_cross(geometry.vertices, face)
            magnitude = math.sqrt(sum(value * value for value in cross))
            normal = tuple(value / magnitude for value in cross)
            stream.write(f"vn {normal[0]:.9f} {normal[1]:.9f} {normal[2]:.9f}\n")
        for normal_index, (a, b, c) in enumerate(geometry.faces, start=1):
            stream.write(
                f"f {a + 1}//{normal_index} {b + 1}//{normal_index} "
                f"{c + 1}//{normal_index}\n"
            )
    temporary_path.chmod(0o644)
    os.replace(temporary_path, path)


def mesh_uri(angle_deg: int) -> str:
    return (
        "package://quadruped_bringup/worlds/"
        f"go2w_curved_ramp_{angle_deg}.obj"
    )


def validate_world(
    world_path: Path,
    mesh_directory: Path,
    geometries: list[RampGeometry],
    checks: Checks,
) -> None:
    try:
        root = ET.parse(world_path).getroot()
    except (ET.ParseError, OSError) as error:
        checks.check(False, f"world: SDF parse ({error})")
        return
    checks.check(root.tag == "sdf" and root.find("world") is not None, "world: SDF parses")
    world = root.find("world")
    assert world is not None
    models = {model.get("name"): model for model in world.findall("model")}

    checks.check(
        models.get("ground_plane") is not None
        and models["ground_plane"].findtext("link/collision/geometry/plane/normal") == "0 0 1"
        and models["ground_plane"].findtext("link/collision/geometry/plane/size") == "100 100",
        "world: ground plane unchanged",
    )
    checks.check(
        all(f"ramp_{lane.angle_deg}_deg" in models for lane in LANES),
        "world: all four target angles present",
    )
    checks.check(
        not any(name and name.startswith("platform_") for name in models),
        "world: old platform primitives removed",
    )
    checks.check(
        all(
            model.find("link/collision/geometry/box") is None
            for name, model in models.items()
            if name and name.startswith("ramp_")
        ),
        "world: sharp ramp primitives removed",
    )
    checks.check(
        all(abs(first.y_m - second.y_m) >= RAMP_WIDTH_M for first, second in zip(LANES, LANES[1:])),
        "world: adjacent lanes separated",
    )
    checks.check(
        -3.0 < ENTRY_X_M and LANES[0].y_m == -6.0,
        "world: default robot spawn remains on flat run-up",
    )

    for geometry in geometries:
        lane = geometry.lane
        label = f"{lane.angle_deg}deg"
        model = models.get(f"ramp_{lane.angle_deg}_deg")
        checks.check(model is not None, f"{label}: assembly model present")
        if model is None:
            continue
        pose = [float(value) for value in (model.findtext("pose") or "").split()]
        expected_uri = mesh_uri(lane.angle_deg)
        collision_uri = model.findtext("link/collision/geometry/mesh/uri")
        visual_uri = model.findtext("link/visual/geometry/mesh/uri")
        checks.check(
            len(pose) == 6
            and abs(pose[0] - ENTRY_X_M) < 1.0e-12
            and abs(pose[1] - lane.y_m) < 1.0e-12
            and abs(pose[2]) < 1.0e-12,
            f"{label}: lane pose",
        )
        checks.check(collision_uri == visual_uri == expected_uri, f"{label}: visual/collision alignment")
        checks.check(
            (mesh_directory / f"go2w_curved_ramp_{lane.angle_deg}.obj").is_file(),
            f"{label}: mesh URI resolves",
        )
        mu = float(model.findtext("link/collision/surface/friction/ode/mu", "nan"))
        mu2 = float(model.findtext("link/collision/surface/friction/ode/mu2", "nan"))
        checks.check((mu, mu2) == EXPECTED_FRICTION, f"{label}: friction preserved")
        marker = models.get(f"approach_marker_{lane.angle_deg}_deg")
        marker_pose = [float(value) for value in (marker.findtext("pose") if marker is not None else "").split()]
        marker_size = [
            float(value)
            for value in (
                marker.findtext("link/visual/geometry/box/size") if marker is not None else ""
            ).split()
        ]
        checks.check(
            len(marker_pose) == 6
            and len(marker_size) == 3
            and abs(marker_pose[1] - lane.y_m) < 1.0e-12
            and abs(marker_pose[0] + 0.5 * marker_size[0] - ENTRY_X_M) < 1.0e-12,
            f"{label}: approach marker alignment",
        )
        checks.check(RAMP_WIDTH_M >= 2.5, f"{label}: width preserved")
        checks.check(
            abs(max(vertex[2] for vertex in geometry.vertices) - geometry.points[-1][1]) < 1.0e-12,
            f"{label}: platform height matches path",
        )


def parse_args() -> argparse.Namespace:
    package_root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--world-output",
        type=Path,
        default=package_root / "worlds" / "go2w_slope_test.sdf",
        help="authoritative world SDF to validate against the generated meshes",
    )
    parser.add_argument(
        "--mesh-output-directory",
        type=Path,
        default=package_root / "worlds" / "meshes",
    )
    parser.add_argument("--design-speed-mps", type=float, default=10.0)
    parser.add_argument("--maximum-normal-accel-mps2", type=float, default=9.81)
    parser.add_argument("--minimum-transition-length-m", type=float, default=2.0)
    parser.add_argument("--longitudinal-resolution-m", type=float, default=0.025)
    parser.add_argument("--lateral-resolution-m", type=float, default=0.10)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    positive_values = (
        args.design_speed_mps,
        args.maximum_normal_accel_mps2,
        args.minimum_transition_length_m,
        args.longitudinal_resolution_m,
        args.lateral_resolution_m,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in positive_values):
        raise ValueError("all geometry-design parameters must be finite and positive")
    if args.longitudinal_resolution_m > 0.025:
        raise ValueError("longitudinal resolution must be no coarser than 0.025 m")
    if args.lateral_resolution_m > 0.10:
        raise ValueError("lateral resolution must be no coarser than 0.10 m")

    checks = Checks()
    geometries: list[RampGeometry] = []
    for lane in LANES:
        geometry = generate_ramp(
            lane,
            args.design_speed_mps,
            args.maximum_normal_accel_mps2,
            args.minimum_transition_length_m,
            args.longitudinal_resolution_m,
            args.lateral_resolution_m,
        )
        validate_geometry(
            geometry,
            checks,
            args.design_speed_mps,
            args.maximum_normal_accel_mps2,
            args.minimum_transition_length_m,
        )
        output = args.mesh_output_directory / f"go2w_curved_ramp_{lane.angle_deg}.obj"
        write_obj(output, geometry, args.force)
        geometries.append(geometry)

    validate_world(args.world_output, args.mesh_output_directory, geometries, checks)

    print(
        "angle  entry_m  constant_m  crest_m  height_m  kappa_max_1pm  "
        "normal_accel_mps2  max_adjacent_deg  triangles"
    )
    for geometry in geometries:
        target = math.radians(geometry.lane.angle_deg)
        curvature = target * 1.875 / geometry.transition_length_m
        normal_acceleration = args.design_speed_mps**2 * curvature
        print(
            f"{geometry.lane.angle_deg:>5}  "
            f"{geometry.transition_length_m:>7.4f}  "
            f"{CONSTANT_SLOPE_LENGTH_M:>10.4f}  "
            f"{geometry.transition_length_m:>7.4f}  "
            f"{geometry.points[-1][1]:>8.4f}  "
            f"{curvature:>14.6f}  "
            f"{normal_acceleration:>17.6f}  "
            f"{geometry.maximum_adjacent_slope_change_deg:>16.6f}  "
            f"{geometry.triangle_count:>9}"
        )
    print(f"static_checks={checks.count} failures={len(checks.failures)}")
    for failure in checks.failures:
        print(f"FAIL: {failure}")
    return 1 if checks.failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
