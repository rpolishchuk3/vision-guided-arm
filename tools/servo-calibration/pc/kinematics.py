"""Analytical kinematics for the five positioning joints. Public units are mm and degrees."""

from dataclasses import dataclass
from itertools import product
import math

if __package__:
    from . import geometry_config as geom
    from .joint_calibration import JointCalibration
else:
    import geometry_config as geom
    from joint_calibration import JointCalibration


EPS_MM = 1e-8
EPS_DEG = 1e-8
LINKS = (geom.UPPER_ARM_MM, geom.FOREARM_MM, geom.TOOL_MM)


@dataclass(frozen=True)
class Pose:
    tcp: tuple[float, float, float]
    pitch_deg: float
    roll_deg: float
    joints: dict


@dataclass(frozen=True)
class IKSolution:
    angles_deg: tuple
    pwm: tuple
    elbow: str


@dataclass(frozen=True)
class _CalibratedAxis:
    calibration: JointCalibration
    reference: float
    direction: int
    pwm_limits: tuple
    angle_limits: tuple
    max_slope: float


def _joint_values(values):
    values = tuple(values)
    if len(values) != len(geom.JOINTS) or not all(math.isfinite(v) for v in values):
        raise ValueError('Expected five finite values ordered S1 through S5.')
    return values


def forward_kinematics(angles_deg):
    """Compute TCP and joint locations from kinematic angles ordered S1–S5."""
    a1, a2, a3, a4, a5 = _joint_values(angles_deg)
    yaw = math.radians(a1)
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    radial, z = geom.SHOULDER_OFFSET_MM, geom.BASE_HEIGHT_MM
    joints = {'S1': (0.0, 0.0, z), 'S2': (radial * cos_yaw, radial * sin_yaw, z)}
    pitch = a2 + a3 + a4
    for name, length, angle in zip(('S3', 'S4', 'TCP'), LINKS, (a2, a2 + a3, pitch)):
        direction = math.radians(angle)
        radial += length * math.cos(direction)
        z += length * math.sin(direction)
        joints[name] = (radial * cos_yaw, radial * sin_yaw, z)
    return Pose(joints['TCP'], pitch, a5, joints)


def _clamp(value, lo, hi):
    return min(hi, max(lo, value))


def _equivalents(value, limits):
    lo, hi = limits
    value = math.remainder(value, 360)
    first = math.ceil((lo - value - EPS_DEG) / 360)
    last = math.floor((hi - value + EPS_DEG) / 360)
    return [_clamp(value + 360 * k, lo, hi) for k in range(first, last + 1)]


class ArmKinematics:
    """Use the existing calibration curves and the limits advertised by the controller."""

    def __init__(self, calibrations, servo_limits):
        self.axes = []
        for i in geom.JOINTS:
            calibration = calibrations.get(i)
            if calibration is None or not calibration.ready:
                raise ValueError(f'S{i} needs an angular calibration.')
            board_lo, board_hi = servo_limits[i]
            lo = max(board_lo, calibration.curve[0][0])
            hi = min(board_hi, calibration.curve[-1][0])
            if lo >= hi:
                raise ValueError(f'S{i} has no calibrated range within its PWM limits.')
            try:
                reference = calibration.pwm_to_angle(geom.REFERENCE_PWM[i])
            except ValueError as exc:
                raise ValueError(f'S{i} reference PWM is outside its calibration.') from exc
            direction = geom.JOINT_DIRECTION[i]
            bounds = tuple(sorted(direction * (calibration.pwm_to_angle(p) - reference)
                                  for p in (lo, hi)))
            slope = max(abs((b[1] - a[1]) / (b[0] - a[0]))
                        for a, b in zip(calibration.curve, calibration.curve[1:]))
            self.axes.append(_CalibratedAxis(calibration, reference, direction, (lo, hi), bounds, slope))
        self.angle_limits = tuple(axis.angle_limits for axis in self.axes)

    def pwm_to_angles(self, pwm):
        angles = []
        for i, pulse, axis in zip(geom.JOINTS, _joint_values(pwm), self.axes):
            lo, hi = axis.pwm_limits
            if not lo <= pulse <= hi:
                raise ValueError(f'S{i} PWM must be in its calibrated range {lo}–{hi} µs.')
            angles.append(axis.direction * (axis.calibration.pwm_to_angle(pulse) - axis.reference))
        return tuple(angles)

    def angles_to_pwm(self, angles_deg):
        pwm = []
        for i, angle, axis in zip(geom.JOINTS, _joint_values(angles_deg), self.axes):
            lo, hi = axis.pwm_limits
            amin, amax = axis.angle_limits
            if not amin - EPS_DEG <= angle <= amax + EPS_DEG:
                raise ValueError(f'S{i} angle is outside its calibrated servo limits.')
            gui_angle = axis.reference + axis.direction * _clamp(angle, amin, amax)
            gui_angle = _clamp(gui_angle, *axis.calibration.angle_limits)
            pulse = axis.calibration.angle_to_pwm(gui_angle)
            if not lo <= pulse <= hi:
                raise ValueError(f'S{i} angle converts outside the configured PWM limits.')
            pwm.append(pulse)
        return tuple(pwm)

    def forward_pwm(self, pwm):
        return forward_kinematics(self.pwm_to_angles(pwm))

    def inverse_kinematics(self, x, y, z, pitch_deg, roll_deg=None, *, current=None):
        """Return valid branches ordered by least travel from current kinematic angles."""
        if not all(math.isfinite(v) for v in (x, y, z, pitch_deg)):
            raise ValueError('X, Y, Z and pitch must be finite numbers.')
        if roll_deg is not None and not math.isfinite(roll_deg):
            raise ValueError('Roll must be a finite number.')
        preferred = _joint_values(current) if current is not None else (0,) * len(geom.JOINTS)
        yaw_limits, shoulder_limits, elbow_limits, wrist_limits, roll_limits = self.angle_limits
        rolls = ([_clamp(preferred[4], *roll_limits)] if roll_deg is None
                 else _equivalents(roll_deg, roll_limits))
        radial = math.hypot(x, y)
        if radial <= EPS_MM:
            # Base angle is free on the Z axis. Preserve the current angle if possible.
            planes = [(_clamp(preferred[0], *yaw_limits), 0.0)]
        else:
            yaw = math.degrees(math.atan2(y, x))
            planes = [(base, sign * radial)
                      for heading, sign in ((yaw, 1), (yaw + 180, -1))
                      for base in _equivalents(heading, yaw_limits)]

        upper, forearm, tool = LINKS
        pitch_deg = math.remainder(pitch_deg, 360)
        pitch = math.radians(pitch_deg)
        wrist_z = z - geom.BASE_HEIGHT_MM - tool * math.sin(pitch)
        # Check both signed radial planes geometrically before reporting a joint-limit error.
        wrist_offset = geom.SHOULDER_OFFSET_MM + tool * math.cos(pitch)
        if not any(abs(upper - forearm) - EPS_MM <= math.hypot(r - wrist_offset, wrist_z)
                   <= upper + forearm + EPS_MM for r in (radial, -radial)):
            raise ValueError('Target is outside the workspace at the requested pitch.')

        solutions = {}
        for base, radius in planes:
            wrist_r = radius - wrist_offset
            distance = math.hypot(wrist_r, wrist_z)
            if not abs(upper - forearm) - EPS_MM <= distance <= upper + forearm + EPS_MM:
                continue
            cosine = (distance**2 - upper**2 - forearm**2) / (2 * upper * forearm)
            if abs(abs(cosine) - 1) <= 1e-12:
                cosine = math.copysign(1, cosine)
            bend = math.acos(_clamp(cosine, -1, 1))
            singular = abs(math.sin(bend)) < 1e-12
            for elbow, label in ((-bend, 'elbow-up'), (bend, 'elbow-down')):
                shoulder = (math.atan2(wrist_z, wrist_r)
                            - math.atan2(forearm * math.sin(elbow), upper + forearm * math.cos(elbow)))
                s2, s3 = math.degrees(shoulder), math.degrees(elbow)
                s4 = pitch_deg - s2 - s3
                for s2_eq, s3_eq, s4_eq, roll in product(_equivalents(s2, shoulder_limits),
                                                         _equivalents(s3, elbow_limits),
                                                         _equivalents(s4, wrist_limits), rolls):
                    angles = (base, s2_eq, s3_eq, s4_eq, roll)
                    pwm = self.angles_to_pwm(angles)
                    solutions.setdefault(pwm, IKSolution(angles, pwm, 'singular' if singular else label))
        if not solutions:
            raise ValueError('No IK solution within the calibrated servo limits at this pitch and roll.')
        return sorted(solutions.values(), key=lambda s: sum((q - p)**2 for q, p in
                                                            zip(self.pwm_to_angles(s.pwm), preferred)))

    def check_table(self, pose):
        for name in ('S2', 'S3', 'S4', 'TCP'):
            if pose.joints[name][2] < geom.TABLE_CLEARANCE_MM:
                raise ValueError(f'{name} goes below the {geom.TABLE_CLEARANCE_MM:g} mm table clearance.')

    def validate_path(self, start_pwm, target_pwm):
        """Bound table clearance over the synchronized, monotonic PWM interpolation."""
        start_pwm, target_pwm = tuple(start_pwm), tuple(target_pwm)
        self.check_table(self.forward_pwm(start_pwm))
        self.check_table(self.forward_pwm(target_pwm))
        delta = tuple(b - a for a, b in zip(start_pwm, target_pwm))
        pitch_slopes = tuple(axis.max_slope for axis in self.axes[1:4])

        def check_interval(lo, hi, depth):
            mid = (lo + hi) * 0.5
            pose = self.forward_pwm(a + mid * d for a, d in zip(start_pwm, delta))
            self.check_table(pose)
            # Bound height loss using |sin(a) - sin(b)| <= |a - b| and the
            # largest calibration slope. The extra half µs covers PWM rounding.
            sweep = height_bound = 0.0
            for name, length, d, slope in zip(('S3', 'S4', 'TCP'), LINKS, delta[1:4], pitch_slopes):
                sweep += math.radians((abs(d) * (hi - lo) * 0.5 + 0.5) * slope) if d else 0.0
                height_bound += length * sweep
                if pose.joints[name][2] - height_bound < geom.TABLE_CLEARANCE_MM:
                    break
            else:
                return
            if depth == 10:
                raise ValueError('Cannot verify table clearance over this motion. Choose a higher waypoint.')
            check_interval(lo, mid, depth + 1)
            check_interval(mid, hi, depth + 1)

        check_interval(0.0, 1.0, 0)
