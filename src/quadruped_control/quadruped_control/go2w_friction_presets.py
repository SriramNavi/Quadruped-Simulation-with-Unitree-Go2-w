"""Validate Go2-W friction presets and render isolated runtime descriptions."""

from __future__ import annotations

import argparse
import math
import os
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Mapping

import yaml


REQUIRED_PARAMETERS = (
    'wheel_mu_longitudinal',
    'wheel_mu_lateral',
    'terrain_mu_longitudinal',
    'terrain_mu_lateral',
    'wheel_slip_longitudinal',
    'wheel_slip_lateral',
)
METADATA_PARAMETERS = (
    'wheel_contact_stiffness',
    'wheel_contact_damping',
)
ALL_PARAMETERS = REQUIRED_PARAMETERS + METADATA_PARAMETERS
WHEEL_LINKS = ('FL_foot', 'FR_foot', 'RL_foot', 'RR_foot')


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise ValueError(f'duplicate YAML key: {key!r}')
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


@dataclass(frozen=True)
class ResolvedFriction:
    """A validated preset after all explicit overrides are applied."""

    requested_preset: str
    effective_label: str
    values: dict[str, float]
    overrides: dict[str, float]


@dataclass(frozen=True)
class RuntimeDescriptions:
    """Paths and parameters owned by one isolated launch runtime."""

    directory: Path
    robot_sdf: Path
    world_sdf: Path
    effective_parameters: Path
    friction: ResolvedFriction

    def cleanup(self) -> None:
        """Remove only this launch's uniquely owned runtime directory."""
        shutil.rmtree(self.directory, ignore_errors=True)


def default_preset_path() -> Path:
    """Resolve the installed preset file, with a source-tree fallback."""
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory('unitree_go2w_description'))
        return share / 'config' / 'go2w_friction_presets.yaml'
    except (ImportError, LookupError):
        source_path = (
            Path(__file__).resolve().parents[2]
            / 'unitree_go2w_description'
            / 'config'
            / 'go2w_friction_presets.yaml'
        )
        if source_path.is_file():
            return source_path
        raise RuntimeError('cannot locate go2w_friction_presets.yaml')


def load_configuration(path: os.PathLike | str) -> dict:
    """Load a preset without modifying it or accepting duplicate keys."""
    with Path(path).open(encoding='utf-8') as stream:
        configuration = yaml.load(stream, Loader=_UniqueKeyLoader)
    errors = validate_configuration(configuration)
    if errors:
        raise ValueError('; '.join(errors))
    return configuration


def _numeric(name: str, value) -> float:
    if isinstance(value, bool):
        raise ValueError(f'{name} must be numeric')
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{name} must be numeric') from error
    if not math.isfinite(number):
        raise ValueError(f'{name} must be finite')
    if number < 0.0:
        raise ValueError(f'{name} must be non-negative')
    return number


def validate_configuration(configuration) -> list[str]:
    """Return all structural and numeric preset errors."""
    errors = []
    if not isinstance(configuration, dict):
        return ['top-level YAML value must be a mapping']
    defaults = configuration.get('defaults', {})
    presets = configuration.get('presets')
    if not isinstance(defaults, dict):
        errors.append('defaults must be a mapping')
        defaults = {}
    if not isinstance(presets, dict) or not presets:
        return errors + ['presets must be a non-empty mapping']
    required_names = {
        'dry_concrete', 'smooth_tile', 'wet_tile',
        'low_friction', 'zero_friction',
    }
    missing_names = sorted(required_names - set(presets))
    if missing_names:
        errors.append(f'missing required presets: {", ".join(missing_names)}')
    for preset_name, preset in presets.items():
        if not isinstance(preset_name, str) or not preset_name:
            errors.append(f'invalid preset name: {preset_name!r}')
            continue
        if not isinstance(preset, dict):
            errors.append(f'{preset_name} must be a mapping')
            continue
        for name in REQUIRED_PARAMETERS:
            if name not in preset:
                errors.append(f'{preset_name}.{name} is required')
                continue
            try:
                _numeric(f'{preset_name}.{name}', preset[name])
            except ValueError as error:
                errors.append(str(error))
        for name in METADATA_PARAMETERS:
            if name in preset or name in defaults:
                try:
                    _numeric(
                        f'{preset_name}.{name}',
                        preset.get(name, defaults.get(name)),
                    )
                except ValueError as error:
                    errors.append(str(error))
    zero = presets.get('zero_friction')
    if isinstance(zero, dict):
        for name in REQUIRED_PARAMETERS:
            if zero.get(name) != 0 and zero.get(name) != 0.0:
                errors.append(f'zero_friction.{name} must be explicitly zero')
    return errors


def resolve_friction(
    configuration: dict,
    preset_name: str,
    overrides: Mapping[str, str | float | None] | None = None,
) -> ResolvedFriction:
    """Resolve preset defaults and explicit, including zero, overrides."""
    presets = configuration['presets']
    if preset_name not in presets:
        available = ', '.join(sorted(presets))
        raise ValueError(
            f'unknown friction_preset={preset_name!r}; available: {available}'
        )
    values = dict(configuration.get('defaults', {}))
    values.update(presets[preset_name])
    applied = {}
    for name, raw_value in (overrides or {}).items():
        if name not in ALL_PARAMETERS:
            raise ValueError(f'unknown friction override: {name}')
        empty_string = isinstance(raw_value, str) and raw_value.strip() == ''
        if raw_value is None or empty_string:
            continue
        applied[name] = _numeric(name, raw_value)
        values[name] = applied[name]
    missing = [name for name in ALL_PARAMETERS if name not in values]
    if missing:
        raise ValueError(f'missing friction parameters: {", ".join(missing)}')
    resolved = {name: _numeric(name, values[name]) for name in ALL_PARAMETERS}
    label = preset_name if not applied else f'{preset_name}+overrides'
    return ResolvedFriction(preset_name, label, resolved, applied)


def _set_text(parent: ET.Element, name: str, value: float) -> None:
    element = parent.find(name)
    if element is None:
        element = ET.SubElement(parent, name)
    element.text = str(value)


def generate_runtime_descriptions(
    *,
    friction: ResolvedFriction,
    source_world: os.PathLike | str,
    control_xacro: os.PathLike | str,
    source_urdf: os.PathLike | str,
    controllers_file: os.PathLike | str,
    ramp_angle_deg: float = 15.0,
) -> RuntimeDescriptions:
    """Render robot/world SDF and metadata into one unique /tmp directory."""
    safe_preset = re.sub(r'[^A-Za-z0-9_.-]+', '_', friction.requested_preset)
    timestamp = datetime.now().astimezone()
    prefix = (
        f'go2w_friction_{safe_preset}_{os.getpid()}_'
        f'{timestamp:%Y%m%dT%H%M%S}_'
    )
    directory = Path(tempfile.mkdtemp(prefix=prefix))
    robot_urdf = directory / 'robot.urdf'
    robot_sdf = directory / 'robot.sdf'
    world_sdf = directory / 'world.sdf'
    snapshot_path = directory / 'effective_friction.yaml'
    values = friction.values
    try:
        command = [
            'xacro', str(control_xacro),
            f'base_urdf_file:={source_urdf}',
            f'controllers_file:={controllers_file}',
            f'wheel_mu_longitudinal:={values["wheel_mu_longitudinal"]}',
            f'wheel_mu_lateral:={values["wheel_mu_lateral"]}',
            f'wheel_slip_longitudinal:={values["wheel_slip_longitudinal"]}',
            f'wheel_slip_lateral:={values["wheel_slip_lateral"]}',
        ]
        rendered_urdf = subprocess.run(
            command, check=True, text=True, capture_output=True,
        ).stdout
        robot_urdf.write_text(rendered_urdf, encoding='utf-8')
        rendered_sdf = subprocess.run(
            ['gz', 'sdf', '-p', str(robot_urdf)],
            check=True, text=True, capture_output=True,
        ).stdout
        robot_root = ET.fromstring(rendered_sdf)
        for link_name in WHEEL_LINKS:
            collision = robot_root.find(
                f".//link[@name='{link_name}']/collision"
            )
            if collision is None:
                raise ValueError(
                    f'generated robot has no collision for {link_name}'
                )
            ode = collision.find('./surface/friction/ode')
            if ode is None:
                raise ValueError(
                    f'generated robot has no friction surface for {link_name}'
                )
            _set_text(ode, 'mu', values['wheel_mu_longitudinal'])
            _set_text(ode, 'mu2', values['wheel_mu_lateral'])
            _set_text(ode, 'slip1', values['wheel_slip_longitudinal'])
            _set_text(ode, 'slip2', values['wheel_slip_lateral'])
        ET.ElementTree(robot_root).write(
            robot_sdf, encoding='utf-8', xml_declaration=True,
        )
        robot_urdf.unlink()

        world_tree = ET.parse(source_world)
        surfaces = world_tree.findall('.//collision/surface/friction/ode')
        if not surfaces:
            raise ValueError(
                'selected world contains no terrain friction surfaces'
            )
        for ode in surfaces:
            _set_text(ode, 'mu', values['terrain_mu_longitudinal'])
            _set_text(ode, 'mu2', values['terrain_mu_lateral'])
        world_tree.write(world_sdf, encoding='utf-8', xml_declaration=True)

        snapshot = {
            'requested_preset': friction.requested_preset,
            'effective_preset_label': friction.effective_label,
            'overrides': friction.overrides,
            'effective_parameters': values,
            'wheel_collision_radius_m': 0.086,
            'wheel_collision_width_m': 0.052,
            'physics_engine': 'DART',
            'ramp_angle_deg': float(ramp_angle_deg),
            'generated_robot_sdf': str(robot_sdf),
            'generated_world_sdf': str(world_sdf),
            'generated_at': timestamp.isoformat(),
            'run_identifier': directory.name,
        }
        snapshot_path.write_text(
            yaml.safe_dump(snapshot, sort_keys=False), encoding='utf-8',
        )
        return RuntimeDescriptions(
            directory, robot_sdf, world_sdf, snapshot_path, friction,
        )
    except Exception:
        shutil.rmtree(directory, ignore_errors=True)
        raise


def _print_preset(configuration: dict, name: str) -> None:
    resolved = resolve_friction(configuration, name)
    print(f'preset: {name}')
    for parameter in ALL_PARAMETERS:
        print(f'{parameter}: {resolved.values[parameter]}')


def main(argv: list[str] | None = None) -> int:
    """Run the go2w_friction_presets command-line utility."""
    parser = argparse.ArgumentParser(prog='go2w_friction_presets')
    subparsers = parser.add_subparsers(dest='command', required=True)
    subparsers.add_parser('list', help='list available preset names')
    show = subparsers.add_parser('show', help='show one resolved preset')
    show.add_argument('preset')
    subparsers.add_parser('validate', help='validate the preset file')
    subparsers.add_parser('path', help='print the resolved preset file path')
    arguments = parser.parse_args(argv)
    path = default_preset_path()
    if arguments.command == 'path':
        print(path)
        return 0
    try:
        configuration = load_configuration(path)
        if arguments.command == 'list':
            print('\n'.join(configuration['presets']))
        elif arguments.command == 'show':
            _print_preset(configuration, arguments.preset)
        else:
            print(f'PASS: {path}')
        return 0
    except (OSError, RuntimeError, ValueError, yaml.YAMLError) as error:
        print(f'FAIL: {error}')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
