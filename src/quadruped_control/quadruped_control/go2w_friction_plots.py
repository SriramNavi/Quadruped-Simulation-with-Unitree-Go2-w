"""Plot and compare friction-aware Go2-W slope experiment CSV files."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402


WHEELS = ('fl', 'fr', 'rl', 'rr')
REQUIRED_COLUMNS = {
    'elapsed_time_sec', 'state', 'base_x_m', 'linear_vx_mps',
    'imu_pitch_deg', 'average_longitudinal_slip_ratio',
    'distance_travelled_m',
    *(f'wheel_{wheel}_circumferential_velocity_mps' for wheel in WHEELS),
    *(f'wheel_{wheel}_slip_ratio' for wheel in WHEELS),
}


def load_run(path: Path) -> pd.DataFrame:
    """Load one compatible CSV and coerce malformed samples to missing values."""
    frame = pd.read_csv(path, low_memory=False)
    missing = sorted(REQUIRED_COLUMNS - set(frame.columns))
    if missing:
        raise ValueError(f'{path}: incompatible CSV schema; missing {missing}')
    numeric = [
        column for column in frame.columns
        if column.endswith(('_sec', '_m', '_mps', '_mps2', '_radps', '_deg', '_ratio'))
        or column in {'ramp_angle_deg'}
    ]
    for column in numeric:
        frame[column] = pd.to_numeric(frame[column], errors='coerce')
    frame = frame.dropna(subset=['elapsed_time_sec']).sort_values('elapsed_time_sec')
    if frame.empty:
        raise ValueError(f'{path}: no valid timestamped rows')
    frame.attrs['path'] = path
    return frame


def run_label(frame: pd.DataFrame) -> str:
    """Return a stable legend label from recorded metadata."""
    if 'friction_preset' in frame:
        values = frame['friction_preset'].dropna().astype(str)
        if not values.empty and values.iloc[0]:
            return values.iloc[0]
    return frame.attrs['path'].stem


def available_column(frame: pd.DataFrame, primary: str, fallback: str) -> str:
    """Prefer a populated sensor column and otherwise use ground truth."""
    if primary in frame and not frame[primary].dropna().empty:
        return primary
    return fallback


def plot_columns(axis, frame, columns: Iterable[str], ylabel: str) -> None:
    """Plot each available nonempty column without hiding missing channels."""
    plotted = False
    for column in columns:
        if column not in frame:
            continue
        valid = frame[['elapsed_time_sec', column]].dropna()
        if valid.empty:
            continue
        axis.plot(valid['elapsed_time_sec'], valid[column], label=column)
        plotted = True
    axis.set_xlabel('Elapsed time (s)')
    axis.set_ylabel(ylabel)
    axis.grid(True, alpha=0.3)
    if plotted:
        axis.legend(fontsize='small')
    else:
        axis.text(0.5, 0.5, 'No valid data', ha='center', va='center', transform=axis.transAxes)


def single_run(csv_path: Path, output_directory: Path) -> Path:
    """Generate the required eight-panel diagnostic plot for one run."""
    frame = load_run(csv_path)
    output_directory.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(4, 2, figsize=(16, 18), constrained_layout=True)
    orientation_columns = tuple(
        available_column(frame, f'imu_{axis}_deg', f'{axis}_deg')
        for axis in ('roll', 'pitch', 'yaw')
    )
    angular_columns = tuple(
        available_column(
            frame, f'imu_angular_{axis}_radps', f'angular_v{axis}_radps',
        )
        for axis in ('x', 'y', 'z')
    )
    panels = (
        (orientation_columns, 'Orientation (deg)'),
        (angular_columns, 'Angular velocity (rad/s)'),
        (
            ('imu_accel_x_mps2', 'imu_accel_y_mps2', 'imu_accel_z_mps2'),
            'Linear acceleration (m/s²)',
        ),
        (
            tuple(f'wheel_{wheel}_angular_velocity_radps' for wheel in WHEELS),
            'Wheel angular velocity (rad/s)',
        ),
        (
            tuple(
                f'wheel_{wheel}_circumferential_velocity_mps'
                for wheel in WHEELS
            ),
            'Wheel circumferential velocity (m/s)',
        ),
        (('linear_vx_mps', 'cmd_linear_x_mps'), 'Forward velocity (m/s)'),
        (
            tuple(f'wheel_{wheel}_slip_ratio' for wheel in WHEELS),
            'Longitudinal slip ratio (-)',
        ),
        (('average_longitudinal_slip_ratio',), 'Average longitudinal slip ratio (-)'),
    )
    for axis, (columns, ylabel) in zip(axes.flat, panels):
        plot_columns(axis, frame, columns, ylabel)
    figure.suptitle(f'Go2-W slope contact run: {run_label(frame)}', fontsize=16)
    output = output_directory / f'{csv_path.stem}_friction_diagnostics.png'
    figure.savefig(output, dpi=160)
    plt.close(figure)
    return output


def summarize(frame: pd.DataFrame) -> dict[str, object]:
    """Compute reproducible ramp and completion metrics from recorded state."""
    angle = float(frame['ramp_angle_deg'].dropna().iloc[0])
    success_start = (
        float(frame['platform_success_start_x_m'].dropna().iloc[0])
        if 'platform_success_start_x_m' in frame
        and not frame['platform_success_start_x_m'].dropna().empty else np.inf
    )
    ramp = frame[(frame['base_x_m'] >= 1.5) & (frame['base_x_m'] <= success_start)]
    pitch_column = available_column(frame, 'imu_pitch_deg', 'pitch_deg')
    pitch = ramp[pitch_column].dropna()
    pitch_error = (pitch.abs() - angle).abs()
    angular_columns = [
        available_column(
            frame, f'imu_angular_{axis}_radps', f'angular_v{axis}_radps',
        )
        for axis in ('x', 'y', 'z')
    ]
    angular = ramp[angular_columns].dropna() if angular_columns else pd.DataFrame()
    angular_rms = (
        float(np.sqrt(np.mean(np.square(angular.to_numpy()))))
        if not angular.empty else np.nan
    )
    slip_columns = [
        f'wheel_{wheel}_slip_ratio' for wheel in WHEELS
        if f'wheel_{wheel}_slip_ratio' in ramp
    ]
    slips = (
        ramp[slip_columns].to_numpy(dtype=float)
        if slip_columns else np.empty((0, 0))
    )
    finite_slips = np.abs(slips[np.isfinite(slips)])
    states = frame['state'].fillna('').astype(str)
    successful = bool((states == 'SUCCESS').any())
    completion_rows = frame.loc[states == 'SUCCESS', 'elapsed_time_sec']
    x = frame['base_x_m'].to_numpy(dtype=float)
    running_max = np.maximum.accumulate(np.where(np.isfinite(x), x, -np.inf))
    backward = running_max - x
    finite_backward = backward[np.isfinite(backward)]
    return {
        'preset': run_label(frame),
        'csv': str(frame.attrs['path']),
        'mean_abs_pitch_error_deg': (
            float(pitch_error.mean()) if not pitch_error.empty else np.nan
        ),
        'pitch_std_deg': float(pitch.std(ddof=0)) if not pitch.empty else np.nan,
        'rms_angular_velocity_radps': angular_rms,
        'mean_abs_slip_ratio': (
            float(finite_slips.mean()) if finite_slips.size else np.nan
        ),
        'peak_abs_slip_ratio': (
            float(finite_slips.max()) if finite_slips.size else np.nan
        ),
        'mean_forward_velocity_mps': (
            float(ramp['linear_vx_mps'].mean()) if not ramp.empty else np.nan
        ),
        'completion_time_sec': float(completion_rows.iloc[0]) if successful else np.nan,
        'climb_success': successful,
        'maximum_backward_displacement_m': (
            float(finite_backward.max()) if finite_backward.size else np.nan
        ),
    }


def comparison(csv_paths: list[Path], output_directory: Path) -> tuple[Path, Path]:
    """Generate cross-preset traces and a machine-readable summary table."""
    frames = [load_run(path) for path in csv_paths]
    output_directory.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(3, 2, figsize=(16, 14), constrained_layout=True)
    trace_columns = (
        ('pitch', 'Pitch (deg)'),
        ('average_longitudinal_slip_ratio', 'Average slip ratio (-)'),
        ('linear_vx_mps', 'Forward velocity (m/s)'),
        ('distance_travelled_m', 'Distance travelled (m)'),
    )
    for axis, (column, ylabel) in zip(axes.flat[:4], trace_columns):
        for frame in frames:
            selected_column = (
                available_column(frame, 'imu_pitch_deg', 'pitch_deg')
                if column == 'pitch' else column
            )
            valid = frame[['elapsed_time_sec', selected_column]].dropna()
            axis.plot(
                valid['elapsed_time_sec'], valid[selected_column],
                label=run_label(frame),
            )
        axis.set_xlabel('Elapsed time (s)')
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=0.3)
        axis.legend(fontsize='small')
    speed_axis = axes.flat[4]
    for frame in frames:
        wheel_columns = [
            f'wheel_{wheel}_circumferential_velocity_mps' for wheel in WHEELS
        ]
        wheel_mean = frame[wheel_columns].mean(axis=1)
        speed_axis.plot(frame['linear_vx_mps'], wheel_mean, '.', ms=2, label=run_label(frame))
    speed_axis.set_xlabel('Base forward velocity (m/s)')
    speed_axis.set_ylabel('Mean wheel tread velocity (m/s)')
    speed_axis.grid(True, alpha=0.3)
    speed_axis.legend(fontsize='small')

    summaries = pd.DataFrame(summarize(frame) for frame in frames)
    bar_axis = axes.flat[5]
    bar_axis.bar(summaries['preset'], summaries['mean_abs_slip_ratio'])
    bar_axis.set_ylabel('Mean absolute slip ratio (-)')
    bar_axis.tick_params(axis='x', rotation=25)
    bar_axis.grid(True, axis='y', alpha=0.3)
    figure.suptitle('Go2-W friction preset comparison', fontsize=16)
    plot_path = output_directory / 'go2w_friction_comparison.png'
    table_path = output_directory / 'go2w_friction_summary.csv'
    figure.savefig(plot_path, dpi=160)
    plt.close(figure)
    summaries.to_csv(table_path, index=False)
    return plot_path, table_path


def main(argv=None) -> None:
    """Run the single-run or comparison plotting command."""
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command', required=True)
    single = subparsers.add_parser('single')
    single.add_argument('csv', type=Path)
    single.add_argument('--output-directory', type=Path, default=Path('plots'))
    compare = subparsers.add_parser('compare')
    compare.add_argument('csv', nargs='+', type=Path)
    compare.add_argument('--output-directory', type=Path, default=Path('plots'))
    args = parser.parse_args(argv)
    if args.command == 'single':
        print(single_run(args.csv, args.output_directory))
    else:
        for path in comparison(args.csv, args.output_directory):
            print(path)
