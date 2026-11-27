"""Main robot GUI with joint sliders, Cartesian control, and parking."""

from pathlib import Path
import sys
import tkinter as tk
from tkinter import ttk

if not __package__:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from gui.cartesian_control import CartesianControl
from gui.joint_controls import JointControlApp
from robot.joint_calibration import ANGLE_JOINTS


class App(JointControlApp):
    def __init__(self, root):
        self.await_pose = False
        self.parking = False
        self.syncing = False
        super().__init__(root, title='Robot arm control',
                         subtitle='Joint control in PWM or calibrated degrees', size='960x690', minsize=(800, 615))
        self.cartesian = CartesianControl(self.extra_panel, self)
        self.cartesian.pack(fill='x', pady=(0, 12))
        self.park_button = ttk.Button(self.bottom, text='Park & Disable PWM', command=self.park, state='disabled')
        self.park_button.pack(side='left')
        self.copy = ttk.Button(self.bottom, text='Copy current pose', command=self.copy_pose, state='disabled')
        self.copy.pack(side='right')

    @property
    def park_supported(self):
        return 'park' in self.client.capabilities

    @property
    def sync_supported(self):
        return 'sync' in self.client.capabilities

    @property
    def motion_locked(self):
        return self.parking or self.syncing

    def on_disconnected(self):
        self.await_pose = False
        self.parking = False
        self.syncing = False
        self.cartesian.invalidate()

    def on_robot_event(self, event):
        kind = event['type']
        if kind == 'capability':
            self.on_view_changed()
        elif kind == 'park':
            self.park_event(event['stage'], event['motors'])
        elif kind == 'sync':
            self.sync_event(event['stage'], event['motors'])
        elif kind == 'cancel' and self.motion_locked:
            hold = event['command'] == 'hold'
            if hold:
                self.sync_slider_targets(self.rows)
            self.parking = self.syncing = False
            self.update_motion_controls()
            self.status.set('Motion cancelled. PWM held.' if hold else 'Motion cancelled. PWM disabled.')
        elif kind == 'pose' and self.await_pose:
            self.await_pose = False
            values = event['current']
            if any(pulse is None for pulse in values.values()):
                self.status.set('Cannot copy a complete pose: enable all joints first.')
                return
            text = 'HOME_POSITION = {\n' + ''.join(f'    {i}: {pulse},\n' for i, pulse in sorted(values.items())) + '}'
            self.root.clipboard_clear()
            self.root.clipboard_append(text)
            self.status.set('Commanded pose copied to clipboard (not measured angles).')

    def on_view_changed(self):
        enabled = self.ready and not self.motion_locked
        self.park_button.config(state='normal' if enabled and self.park_supported else 'disabled')
        self.copy.config(state='normal' if enabled else 'disabled')
        self.cartesian.refresh()

    def set_parking(self, active):
        self.parking = active
        self.update_motion_controls()

    def park(self):
        if not self.park_supported or self.motion_locked:
            return
        self.clear_pending()
        self.await_pose = False
        if self.client.park():
            self.set_parking(True)
            self.status.set('Requesting parking pose...')

    def sync_slider_targets(self, motors):
        for i in motors:
            row = self.rows[i]
            pulse = row['pulse']
            if pulse is not None:
                try:
                    self.show_target(i, pulse)
                except ValueError:
                    # Hold may stop a joint outside its degree-control range.
                    self.set_units(i, False, pulse)

    def park_event(self, stage, motors):
        if stage == 'moving':
            self.clear_pending()
            for row in self.rows.values():
                row['awaiting'] = False
                row['holding'] = False
            self.set_parking(True)
            self.status.set('Parking: moving to the parking pose.')
        elif stage == 'settling':
            self.sync_slider_targets(self.rows)
            self.status.set('Parking pose commanded. Disabling PWM in 1 second.')
        elif stage in ('done', 'held', 'stopped', 'init', 'invalid', 'busy'):
            if stage == 'held':
                self.sync_slider_targets(self.rows)
            self.set_parking(False)
            if stage == 'done':
                message = 'Parked. PWM disabled. Servos may still hold torque.'
            elif stage == 'held':
                message = 'Parking cancelled. PWM held at the last commanded pose.'
            elif stage == 'stopped':
                message = 'Parking cancelled. PWM disabled.'
            elif stage == 'busy':
                message = 'Wait for synchronized motion to finish before parking.'
            else:
                joints = ', '.join('{} ({})'.format(i, self.rows[i]['name']) for i in motors)
                if stage == 'init':
                    message = 'Initialize these motors before parking: ' + joints
                else:
                    message = 'Parking blocked by PWM limits for motors: ' + joints
            self.status.set(message)

    def move_cartesian(self, start, target):
        if self.motion_locked or not self.sync_supported:
            return
        self.clear_pending()
        self.await_pose = False
        if self.client.move_group(start, target):
            self.syncing = True
            self.update_motion_controls()
            self.status.set('Requesting synchronized Cartesian motion...')

    def sync_event(self, stage, motors):
        if stage == 'moving':
            self.clear_pending()
            self.syncing = True
            self.update_motion_controls()
            self.status.set('Moving to the Cartesian target.')
        elif stage in ('done', 'held', 'stopped', 'busy', 'init', 'limits', 'start'):
            if stage in ('done', 'held'):
                self.sync_slider_targets(ANGLE_JOINTS)
            self.syncing = False
            self.update_motion_controls()
            joints = ', '.join('S{}'.format(i) for i in motors)
            message = {
                'done': 'Cartesian target commanded. See the estimated pose.',
                'held': 'Cartesian move cancelled. PWM held at the last commanded pose.',
                'stopped': 'Cartesian move cancelled. PWM disabled.',
                'busy': 'Controller is busy. Wait for motion to settle.',
                'init': 'Cartesian move blocked. Initialize ' + joints + '.',
                'limits': 'Cartesian move blocked by PWM limits for ' + joints + '.',
                'start': 'Starting command changed on ' + joints + '. Calculate IK again.',
            }[stage]
            self.status.set(message)

    def copy_pose(self):
        self.clear_pending()
        if any(r['busy'] or r['awaiting'] for r in self.rows.values()):
            self.status.set('Wait until all joints finish moving, then copy the pose.'); return
        self.await_pose=self.client.request_pose()

if __name__ == '__main__':
    root = tk.Tk()
    App(root)
    root.mainloop()
