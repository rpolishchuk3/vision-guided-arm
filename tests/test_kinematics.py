import math
from pathlib import Path
import runpy
import sys
import unittest


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from config.geometry_config import JOINTS, REFERENCE_PWM
from robot.joint_calibration import load_calibrations
from robot.kinematics import ArmKinematics, forward_kinematics


REFERENCE = tuple(REFERENCE_PWM[i] for i in JOINTS)


class KinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config = runpy.run_path(str(REPO_ROOT / 'esp32' / 'servo_config.py'))
        cls.limits = {i: (s['min_us'], s['max_us']) for i, s in config['SERVOS'].items()}
        cls.calibrations = load_calibrations()
        cls.arm = ArmKinematics(cls.calibrations, cls.limits)

    def assertPointAlmostEqual(self, point, expected, places=8):
        for value, reference in zip(point, expected):
            self.assertAlmostEqual(value, reference, places=places)

    def test_horizontal_reference_uses_exact_calibration(self):
        self.assertEqual(self.arm.pwm_to_angles(REFERENCE), (0, 0, 0, 0, 0))
        pose = self.arm.forward_pwm(REFERENCE)
        self.assertEqual(pose.tcp, (375, 0, 112))
        self.assertEqual(pose.pitch_deg, 0)
        self.assertEqual(pose.roll_deg, 0)
        self.assertEqual(pose.joints['S2'], (0, 0, 112))
        self.assertEqual(pose.joints['S3'], (104, 0, 112))
        self.assertEqual(pose.joints['S4'], (203, 0, 112))

    def test_experimental_pose_with_corrected_shoulder_direction(self):
        pwm = (1491, 1296, 1385, 1625, 1990)
        pose = self.arm.forward_pwm(pwm)
        self.assertPointAlmostEqual(pose.tcp, (80.006, -1.126, 476.796), places=3)
        self.assertAlmostEqual(pose.pitch_deg, 74.312, places=3)
        self.assertAlmostEqual(pose.roll_deg, -3.514, places=3)

        current = self.arm.pwm_to_angles(pwm)
        self.assertGreater(current[1], 0)
        solutions = self.arm.inverse_kinematics(*pose.tcp, pose.pitch_deg, pose.roll_deg, current=current)
        self.assertEqual(solutions[0].pwm, pwm)
        self.arm.validate_path(REFERENCE, solutions[0].pwm)

    def test_base_rotation_is_counterclockwise(self):
        self.assertPointAlmostEqual(forward_kinematics((90, 0, 0, 0, 0)).tcp, (0, 375, 112))
        self.assertPointAlmostEqual(forward_kinematics((-90, 0, 0, 0, 0)).tcp, (0, -375, 112))

    def test_individual_pitch_rotations(self):
        for axis, expected in ((1, (0, 0, 487)), (2, (104, 0, 383)), (3, (203, 0, 284))):
            with self.subTest(axis=axis):
                angles = [0] * 5
                angles[axis] = 90
                pose = forward_kinematics(angles)
                self.assertPointAlmostEqual(pose.tcp, expected)
                self.assertEqual(pose.pitch_deg, 90)

    def test_roll_does_not_change_tcp(self):
        pose = forward_kinematics((0, 0, 0, 0, 28))
        self.assertEqual(pose.tcp, (375, 0, 112))
        self.assertEqual(pose.roll_deg, 28)

    def test_fk_to_ik_to_pwm_round_trip(self):
        for pwm in (REFERENCE, (1650, 693, 1231, 1912, 1791), (1500, 2280, 1021, 2300, 2309)):
            with self.subTest(pwm=pwm):
                angles = self.arm.pwm_to_angles(pwm)
                pose = self.arm.forward_pwm(pwm)
                solution = self.arm.inverse_kinematics(*pose.tcp, pose.pitch_deg, pose.roll_deg,
                                                       current=angles)[0]
                self.assertEqual(solution.pwm, pwm)
                reconstructed = forward_kinematics(solution.angles_deg)
                self.assertPointAlmostEqual(reconstructed.tcp, pose.tcp)
                self.assertAlmostEqual(math.remainder(reconstructed.pitch_deg - pose.pitch_deg, 360), 0)

    def test_both_elbows_and_signed_radial_base_branch(self):
        # This target has negative radial reach, while its heading in table coordinates is 180°.
        current = (0, 170, -30, 60, 0)
        pose = forward_kinematics(current)
        solutions = self.arm.inverse_kinematics(*pose.tcp, pose.pitch_deg, current=current)
        self.assertLess(pose.tcp[0], 0)
        self.assertEqual([s.elbow for s in solutions], ['elbow-up', 'elbow-down'])
        reranked = self.arm.inverse_kinematics(*pose.tcp, pose.pitch_deg, current=solutions[1].angles_deg)
        self.assertEqual(reranked[0].elbow, 'elbow-down')
        for solution in solutions:
            self.assertPointAlmostEqual(forward_kinematics(solution.angles_deg).tcp, pose.tcp)

    def test_unreachable_and_joint_limit_violations(self):
        with self.assertRaisesRegex(ValueError, 'outside the workspace'):
            self.arm.inverse_kinematics(376, 0, 112, 0)
        with self.assertRaisesRegex(ValueError, 'outside the workspace'):
            self.arm.inverse_kinematics(172, 0, 112, 0)  # Wrist inside the inner 5 mm radius.
        with self.assertRaisesRegex(ValueError, 'servo limits'):
            self.arm.inverse_kinematics(375, 0, 112, 0, 100)
        limited = ArmKinematics(self.calibrations, {**self.limits, 1: (1450, 1550)})
        turned = forward_kinematics((30, 0, 0, 0, 0))
        with self.assertRaisesRegex(ValueError, 'servo limits'):
            limited.inverse_kinematics(*turned.tcp, 0)
        with self.assertRaisesRegex(ValueError, 'PWM'):
            limited.forward_pwm((1560, *REFERENCE[1:]))
        with self.assertRaisesRegex(ValueError, 'S2'):
            self.arm.forward_pwm((1500, 200, *REFERENCE[2:]))  # Board limit is wider than calibration.
        with self.assertRaisesRegex(ValueError, 'finite'):
            self.arm.inverse_kinematics(float('nan'), 0, 112, 0)

    def test_straight_singularity_has_one_solution(self):
        solution, = self.arm.inverse_kinematics(375, 0, 112, 360)
        self.assertEqual(solution.elbow, 'singular')
        self.assertEqual(solution.pwm, REFERENCE)

    def test_axis_singularity_preserves_current_base_and_optional_roll(self):
        current = (40, 90, 0, 0, 10)
        solution, = self.arm.inverse_kinematics(0, 0, 487, 90, current=current)
        self.assertEqual(solution.angles_deg[0], 40)
        self.assertEqual(solution.angles_deg[4], 10)
        self.assertPointAlmostEqual(forward_kinematics(solution.angles_deg).tcp, (0, 0, 487))

    def test_pwm_angle_round_trips_including_reversed_axes(self):
        for axis, limits in enumerate(self.arm.angle_limits):
            lo, hi = [self.arm.angles_to_pwm(tuple(bound if j == axis else 0
                                                    for j in range(5)))[axis] for bound in limits]
            for pulse in range(min(lo, hi), max(lo, hi) + 1):
                pwm = (*REFERENCE[:axis], pulse, *REFERENCE[axis + 1:])
                self.assertEqual(self.arm.angles_to_pwm(self.arm.pwm_to_angles(pwm)), pwm)

    def test_path_checks_intermediate_links_and_pwm_limits(self):
        safe = self.arm.angles_to_pwm((20, 15, 25, 0, 5))
        self.arm.validate_path(REFERENCE, safe)
        with self.assertRaisesRegex(ValueError, 'PWM'):
            self.arm.validate_path(REFERENCE, (419, *REFERENCE[1:]))
        # Both endpoints clear the table, but S4 dips below the margin mid-move.
        start = self.arm.angles_to_pwm((0, -4, -108, -100, 0))
        target = self.arm.angles_to_pwm((0, -4, -58, 70, 0))
        self.arm.check_table(self.arm.forward_pwm(start))
        self.arm.check_table(self.arm.forward_pwm(target))
        with self.assertRaisesRegex(ValueError, 'S4.*table clearance'):
            self.arm.validate_path(start, target)


if __name__ == '__main__':
    unittest.main()
