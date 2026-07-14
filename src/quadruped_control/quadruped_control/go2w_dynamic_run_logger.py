"""CSV and JSON run logging with no ROS control dependencies."""

from __future__ import annotations

import csv
import json
import math
import re
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping

from quadruped_control.go2w_dynamic_models import RunMetrics


@dataclass(frozen=True)
class RunLoggerConfig:
    """Run path, metadata, and flushing settings."""

    output_directory: str
    run_id: str = ''
    expected_slope_deg: float = 0.0
    flush_period_sec: float = 1.0


class DynamicRunLogger:
    """Own paths, schema, statistics, periodic flush, and finalization."""

    BASE_FIELDS = [
        'ros_time_sec', 'elapsed_time_sec', 'state', 'contact_phase',
        'expected_slope_deg', 'estimated_slope_deg', 'slope_estimate_valid',
        'raw_slope_deg', 'fast_slope_deg', 'stable_slope_deg',
        'slope_confidence', 'slope_rate_degps', 'base_x_m', 'base_y_m',
        'base_z_m', 'height_gain_m', 'forward_progress_m', 'base_vx_mps',
        'base_vy_mps', 'base_vz_mps', 'measured_forward_speed_mps',
        'commanded_speed_mps', 'requested_speed_mps', 'effective_speed_mps',
        'wheel_cmd_fl_radps', 'wheel_cmd_fr_radps', 'wheel_cmd_rl_radps',
        'wheel_cmd_rr_radps', 'raw_roll_deg', 'filtered_roll_deg',
        'desired_roll_deg', 'roll_correction_deg', 'raw_pitch_deg',
        'filtered_pitch_deg', 'target_pitch_deg', 'desired_level_pitch_deg',
        'feasible_target_pitch_deg', 'pitch_error_deg', 'pitch_rate_radps',
        'pid_p_rad', 'pid_i_rad', 'pid_d_rad', 'pid_output_deg',
        'feedforward_deg', 'feedback_deg', 'final_correction_deg',
        'front_offset_m', 'rear_offset_m', 'left_offset_m', 'right_offset_m',
        'max_positive_correction_deg', 'max_negative_correction_deg',
        'feasibility_joint_margin_rad', 'limiting_leg', 'limiting_joint',
        'limiting_reason', 'workspace_usage', 'longitudinal_margin_m',
        'joint_limit_usage',
        'lateral_margin_m', 'minimum_stability_margin_m', 'stability_state',
        'slip_fl', 'slip_fr', 'slip_rl', 'slip_rr', 'average_abs_slip',
        'peak_abs_slip', 'traction_state', 'slope_scale', 'transition_scale',
        'pitch_scale', 'stability_scale', 'traction_scale', 'workspace_scale',
        'ik_valid', 'saturated', 'workspace_limited', 'ramp_detected',
        'top_detected', 'controller_conflict', 'safe_stop_reason',
        'infeasible_reason', 'final_result',
    ]

    def __init__(
        self, config: RunLoggerConfig, joint_names: tuple[str, ...],
    ) -> None:
        """Create a unique run path and write the CSV header."""
        if config.flush_period_sec <= 0.0:
            raise ValueError('log flush period must be positive')
        directory = Path(config.output_directory).expanduser()
        slope_label = (
            'auto' if config.expected_slope_deg <= 0.0
            else str(int(round(config.expected_slope_deg)))
        )
        directory = directory / f'slope_{slope_label}'
        directory.mkdir(parents=True, exist_ok=True)
        requested = config.run_id.strip()
        if requested and not re.fullmatch(r'[A-Za-z0-9_.-]+', requested):
            raise ValueError(
                'run_id may contain only letters, numbers, ., _, and -'
            )
        base_id = requested or datetime.now(timezone.utc).strftime(
            'run_%Y%m%dT%H%M%S_%fZ'
        )
        run_id = base_id
        suffix = 1
        while (
            (directory / f'{run_id}.csv').exists()
            or (directory / f'{run_id}_summary.json').exists()
        ):
            run_id = f'{base_id}_{suffix:03d}'
            suffix += 1
        self.run_id = run_id
        self.csv_path = directory / f'{run_id}.csv'
        self.summary_path = directory / f'{run_id}_summary.json'
        self.fields = list(self.BASE_FIELDS)
        self.fields.extend(f'current_{name}_rad' for name in joint_names)
        self.fields.extend(f'commanded_{name}_rad' for name in joint_names)
        self.fields.extend(f'error_{name}_rad' for name in joint_names)
        self._file = self.csv_path.open('x', newline='', encoding='utf-8')
        self._writer = csv.DictWriter(
            self._file, fieldnames=self.fields, extrasaction='ignore',
        )
        self._writer.writeheader()
        self._flush_period_sec = config.flush_period_sec
        self._last_flush_wall = time.monotonic()
        self._last_row_time: float | None = None
        self._last_phase = ''
        self.metrics = RunMetrics()
        self.finalized = False

    @staticmethod
    def _finite_float(value: Any) -> float | None:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return None
        return result if math.isfinite(result) else None

    def _record_metrics(self, row: Mapping[str, Any]) -> None:
        mappings = (
            ('pitch_error_deg', self.metrics.pitch_errors_deg),
            ('filtered_pitch_deg', self.metrics.pitch_values_deg),
            ('filtered_roll_deg', self.metrics.roll_values_deg),
            ('effective_speed_mps', self.metrics.speeds_mps),
            ('average_abs_slip', self.metrics.slips),
        )
        for key, destination in mappings:
            value = self._finite_float(row.get(key))
            if value is not None:
                destination.append(value)
        operational = row.get('state') not in {
            'WAITING_FOR_DATA', 'READY',
        }
        margin = self._finite_float(row.get('minimum_stability_margin_m'))
        if operational and margin is not None:
            self.metrics.minimum_stability_margin_m = min(
                self.metrics.minimum_stability_margin_m, margin,
            )
        slip = self._finite_float(row.get('peak_abs_slip'))
        if slip is not None:
            self.metrics.peak_abs_slip = max(self.metrics.peak_abs_slip, slip)
        ik_valid = bool(row.get('ik_valid'))
        workspace = self._finite_float(row.get('workspace_usage'))
        if ik_valid and workspace is not None:
            self.metrics.maximum_workspace_usage = max(
                self.metrics.maximum_workspace_usage, workspace,
            )
        joint_usage = self._finite_float(row.get('joint_limit_usage'))
        if ik_valid and joint_usage is not None:
            self.metrics.maximum_joint_limit_usage = max(
                self.metrics.maximum_joint_limit_usage, joint_usage,
            )
        phase = str(row.get('contact_phase', ''))
        ros_time = self._finite_float(row.get('ros_time_sec'))
        if (
            ros_time is not None
            and self._last_row_time is not None
            and self._last_phase
        ):
            dt = max(0.0, ros_time - self._last_row_time)
            self.metrics.phase_durations_sec[self._last_phase] = (
                self.metrics.phase_durations_sec.get(
                    self._last_phase, 0.0,
                ) + dt
            )
        if ros_time is not None:
            self._last_row_time = ros_time
        self._last_phase = phase
        speed = self._finite_float(row.get('effective_speed_mps'))
        if speed is not None:
            if phase in {
                'ENTRY_DETECTED', 'FRONT_AXLE_ON_RAMP', 'MIXED_CONTACT_ENTRY',
            }:
                self.metrics.entry_minimum_speed_mps = min(
                    self.metrics.entry_minimum_speed_mps, speed,
                )
            if phase == 'FULL_SLOPE':
                self.metrics.climb_speeds_mps.append(speed)

    def write(
        self, row: Mapping[str, Any], now_wall: float | None = None,
    ) -> None:
        """Append one schema-filtered row and update run statistics."""
        if self.finalized or self._file.closed:
            return
        clean = {field: row.get(field, '') for field in self.fields}
        self._writer.writerow(clean)
        self._record_metrics(clean)
        wall = time.monotonic() if now_wall is None else now_wall
        if wall - self._last_flush_wall >= self._flush_period_sec:
            self._file.flush()
            self._last_flush_wall = wall

    def statistics(self) -> dict[str, Any]:
        """Return JSON-safe aggregate run statistics."""
        metrics = self.metrics
        finite_minimum = (
            metrics.minimum_stability_margin_m
            if math.isfinite(metrics.minimum_stability_margin_m) else None
        )
        entry_minimum = (
            metrics.entry_minimum_speed_mps
            if math.isfinite(metrics.entry_minimum_speed_mps) else None
        )
        return {
            'mean_body_pitch_error_deg': metrics.mean(
                metrics.pitch_errors_deg
            ),
            'maximum_body_pitch_error_deg': max(
                (abs(value) for value in metrics.pitch_errors_deg),
                default=None,
            ),
            'mean_pitch_deg': metrics.mean(metrics.pitch_values_deg),
            'maximum_abs_pitch_deg': max(
                (abs(value) for value in metrics.pitch_values_deg),
                default=None,
            ),
            'maximum_abs_roll_deg': max(
                (abs(value) for value in metrics.roll_values_deg),
                default=None,
            ),
            'mean_speed_mps': metrics.mean(metrics.speeds_mps),
            'climb_mean_speed_mps': metrics.mean(metrics.climb_speeds_mps),
            'entry_minimum_speed_mps': entry_minimum,
            'mean_abs_slip': metrics.mean(metrics.slips),
            'peak_abs_slip': metrics.peak_abs_slip,
            'minimum_stability_margin_m': finite_minimum,
            'maximum_workspace_usage': metrics.maximum_workspace_usage,
            'maximum_joint_limit_usage': metrics.maximum_joint_limit_usage,
            'contact_phase_durations_sec': metrics.phase_durations_sec,
        }

    def finalize(self, summary: Mapping[str, Any]) -> None:
        """Close the CSV and atomically create the final JSON summary."""
        if self.finalized:
            return
        self._file.flush()
        self._file.close()
        document = dict(summary)
        document.update(self.statistics())
        document['run_id'] = self.run_id
        document['csv_path'] = str(self.csv_path)
        document['json_path'] = str(self.summary_path)
        with self.summary_path.open('x', encoding='utf-8') as summary_file:
            json.dump(document, summary_file, indent=2, allow_nan=False)
            summary_file.write('\n')
        self.finalized = True
