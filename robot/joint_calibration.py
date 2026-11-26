"""Desktop calibration data and piecewise-linear joint conversion."""

import json
import math
from pathlib import Path
from tempfile import NamedTemporaryFile


CALIBRATION_PATH = Path(__file__).resolve().parents[1] / 'config' / 'joint_calibration.json'
ANGLE_JOINTS = range(1, 6)
PWM_LIMITS_US = (200, 3000)


class JointCalibration:
    def __init__(self, points=(), limits=PWM_LIMITS_US, *, extrapolation_pwm=None):
        checked = []
        for point in points:
            if not isinstance(point, (list, tuple)) or len(point) != 2:
                raise ValueError('Expected a pulse and an angle for each point')
            pulse, angle = point
            if type(pulse) is not int or not limits[0] <= pulse <= limits[1]:
                raise ValueError('Calibration pulse outside PWM limits')
            if type(angle) not in (int, float) or not -math.inf < angle < math.inf:
                raise ValueError('Calibration angle must be finite')
            checked.append((pulse, angle))
        self.points = tuple(sorted(checked))
        steps = []
        for (p0, a0), (p1, a1) in zip(self.points, self.points[1:]):
            if p0 == p1:
                raise ValueError('Each calibration pulse must be distinct')
            steps.append(a1 - a0)
        if steps and not (all(0 < d < math.inf for d in steps) or
                          all(-math.inf < d < 0 for d in steps)):
            raise ValueError('Angles must increase or decrease strictly with PWM')
        self.extrapolation_pwm = None
        self.curve = self.points
        if extrapolation_pwm is not None:
            if (not isinstance(extrapolation_pwm, (list, tuple)) or
                    len(extrapolation_pwm) != 2 or
                    any(type(pulse) is not int for pulse in extrapolation_pwm) or
                    not PWM_LIMITS_US[0] <= extrapolation_pwm[0] < extrapolation_pwm[1] <= PWM_LIMITS_US[1]):
                raise ValueError('Invalid PWM limits for extrapolation')
            self.extrapolation_pwm = tuple(extrapolation_pwm)
            if self.ready:
                lo = max(limits[0], extrapolation_pwm[0])
                hi = min(limits[1], extrapolation_pwm[1])
                if lo < self.points[0][0]:
                    angle = self._interpolate(lo, *self.points[:2])
                    if not math.isfinite(angle):
                        raise ValueError('Estimated angle must be finite')
                    self.curve = ((lo, angle),) + self.curve
                if hi > self.points[-1][0]:
                    angle = self._interpolate(hi, *self.points[-2:])
                    if not math.isfinite(angle):
                        raise ValueError('Estimated angle must be finite')
                    self.curve += ((hi, angle),)

    def with_servo_limits(self, limits):
        return JointCalibration(self.points, limits, extrapolation_pwm=self.extrapolation_pwm)

    def is_extrapolated(self, pulse):
        return self.ready and not self.points[0][0] <= pulse <= self.points[-1][0]

    @property
    def ready(self):
        return len(self.points) >= 2

    @property
    def angle_limits(self):
        return tuple(sorted((self.curve[0][1], self.curve[-1][1])))

    @staticmethod
    def _interpolate(value, start, end):
        x0, y0 = start
        x1, y1 = end
        if value == x1:
            return y1
        return y0 + ((value - x0) / (x1 - x0)) * (y1 - y0)

    def _convert(self, value, points):
        if not self.ready:
            raise ValueError('Record at least two calibration points')
        if not points[0][0] <= value <= points[-1][0]:
            raise ValueError('Value outside calibrated range')
        for start, end in zip(points, points[1:]):
            if value <= end[0]:
                return self._interpolate(value, start, end)

    def pwm_to_angle(self, pulse):
        return self._convert(pulse, self.curve)

    def angle_to_pwm(self, angle):
        inverse = sorted((angle, pulse) for pulse, angle in self.curve)
        return round(self._convert(angle, inverse))


def load_calibrations(path=CALIBRATION_PATH):
    try:
        data = json.loads(path.read_text(encoding='utf-8'))
    except FileNotFoundError:
        return {}
    if not isinstance(data, dict) or data.get('version') != 1 or not isinstance(data.get('joints'), dict):
        raise ValueError('Invalid joint calibration file')
    extensions = data.get('extrapolation_pwm', {})
    if not isinstance(extensions, dict) or extensions.keys() - data['joints'].keys():
        raise ValueError('Invalid extrapolation limits in calibration file')
    joints = {}
    for key, points in data['joints'].items():
        if key not in map(str, ANGLE_JOINTS) or not isinstance(points, list):
            raise ValueError('Invalid joint calibration record')
        joints[int(key)] = JointCalibration(points, extrapolation_pwm=extensions.get(key))
    return joints


def save_calibrations(joints, path=CALIBRATION_PATH):
    data = {'version': 1, 'joints': {key: item.points for key, item in joints.items() if item.points}}
    extensions = {key: item.extrapolation_pwm for key, item in joints.items()
                  if item.points and item.extrapolation_pwm is not None}
    if extensions:
        data['extrapolation_pwm'] = extensions
    staged = None
    try:
        with NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent, delete=False) as output:
            staged = Path(output.name)
            json.dump(data, output, indent=2, allow_nan=False)
            output.write('\n')
        staged.replace(path)
    finally:
        if staged is not None:
            staged.unlink(missing_ok=True)
