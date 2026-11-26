"""Cartesian preview and controls for the desktop GUI."""

import tkinter as tk
from tkinter import ttk

from config.geometry_config import JOINTS, TABLE_CLEARANCE_MM
from robot.kinematics import ArmKinematics


class CartesianControl(ttk.LabelFrame):
    def __init__(self, parent, app):
        super().__init__(parent, text='Cartesian Control', padding=8)
        self.app = app
        self.solution = None
        self.preview_start = None
        self.estimate = tk.StringVar(value='Current estimate unavailable. Connect to load joint limits.')
        self.result = tk.StringVar(value='Enter a target. Leave roll blank to keep the current roll.')
        ttk.Label(self, textvariable=self.estimate).pack(anchor='w')
        fields = ttk.Frame(self)
        fields.pack(fill='x', pady=5)
        self.inputs = []
        for label, default in (('X (mm)', ''), ('Y (mm)', ''), ('Z (mm)', ''),
                               ('Pitch (°)', '0'), ('Roll (°)', '')):
            ttk.Label(fields, text=label).pack(side='left', padx=(0, 3))
            value = tk.StringVar(value=default)
            ttk.Entry(fields, textvariable=value, width=7).pack(side='left', padx=(0, 8))
            self.inputs.append(value)
        self.calculate_button = ttk.Button(fields, text='Calculate IK', command=self.calculate, state='disabled')
        self.calculate_button.pack(side='left')
        self.move_button = ttk.Button(fields, text='Move to Target', command=self.move, state='disabled')
        self.move_button.pack(side='left', padx=(6, 0))
        ttk.Label(self, textvariable=self.result, wraplength=740).pack(anchor='w')
        ttk.Label(self, text=f'Modeled table clearance: {TABLE_CLEARANCE_MM} mm. Other collisions are unchecked.').pack(anchor='w')
        for value in self.inputs:
            value.trace_add('write', self.invalidate)

    def invalidate(self, *_):
        self.solution = None
        self.preview_start = None
        self.move_button.config(state='disabled')
        self.result.set('Calculate IK to preview the target.')

    def model(self):
        return ArmKinematics(self.app.calibrations,
                             {i: (self.app.rows[i]['lo'], self.app.rows[i]['hi']) for i in JOINTS})

    def settled_pwm(self):
        if not self.app.ready or self.app.motion_locked:
            return None
        if any(row['busy'] or row['awaiting'] or row['job'] is not None for row in self.app.rows.values()):
            return None
        pwm = tuple(self.app.rows[i]['pulse'] for i in JOINTS)
        return pwm if None not in pwm else None

    def refresh(self):
        available = self.app.ready and not self.app.motion_locked
        self.calculate_button.config(state='normal' if available else 'disabled')
        start = self.settled_pwm()
        can_move = self.solution and start is not None and start == self.preview_start and self.app.sync_supported
        self.move_button.config(state='normal' if can_move else 'disabled')
        if start is None:
            if not self.app.ready:
                detail = 'Connect to load joint limits.'
            elif self.app.motion_locked or any(row['busy'] or row['awaiting'] for row in self.app.rows.values()):
                detail = 'Waiting for motion to settle.'
            else:
                detail = 'S1–S5 must have known, settled PWM commands.'
            self.estimate.set('Current estimate unavailable. ' + detail)
            return
        try:
            pose = self.model().forward_pwm(start)
        except ValueError as exc:
            self.estimate.set('Current estimate unavailable. ' + str(exc))
            return
        self.estimate.set('Current estimate  X {:.1f}   Y {:.1f}   Z {:.1f} mm   |   Pitch {:.1f}°   Roll {:.1f}°'.format(
            *pose.tcp, pose.pitch_deg, pose.roll_deg))

    def calculate(self):
        if not self.app.ready or self.app.motion_locked:
            return
        self.solution = None
        self.preview_start = None
        try:
            values = [var.get().strip().replace(',', '.') for var in self.inputs]
            try:
                x, y, z, pitch = map(float, values[:4])
                roll = float(values[4]) if values[4] else None
            except ValueError as exc:
                raise ValueError('Enter numbers for X, Y, Z and pitch. Roll is optional.') from exc
            model = self.model()
            start = self.settled_pwm()
            current = None if start is None else model.pwm_to_angles(start)
            candidates = model.inverse_kinematics(x, y, z, pitch, roll, current=current)
            blocked = []
            for candidate in candidates:
                try:
                    if start is None:
                        model.check_table(model.forward_pwm(candidate.pwm))
                    else:
                        model.validate_path(start, candidate.pwm)
                except ValueError as exc:
                    blocked.append(str(exc))
                else:
                    self.solution = candidate
                    break
            if self.solution is None:
                raise ValueError('No solution passed the table check. ' + blocked[0])
            self.preview_start = start
            angles = model.pwm_to_angles(self.solution.pwm)
            joints = '  |  '.join(f'S{i} {angle:.2f}° / {pulse} µs'
                                  for i, angle, pulse in zip(JOINTS, angles, self.solution.pwm))
            result = f'{self.solution.elbow}: {joints}'
            if start is None:
                result += '\nA settled starting pose is required to check the path.'
            if not self.app.sync_supported:
                result += '\nInstall a bridge with synchronized motion support to move.'
            self.result.set(result)
        except ValueError as exc:
            self.result.set(str(exc))
        self.refresh()

    def move(self):
        start = self.settled_pwm()
        if not self.solution or not self.app.sync_supported or start is None or start != self.preview_start:
            self.result.set('Calculate IK again from the current settled commanded pose.')
            self.refresh()
            return
        try:
            self.model().validate_path(start, self.solution.pwm)
        except ValueError as exc:
            self.result.set('Move blocked. ' + str(exc))
            return
        self.app.move_cartesian(start, self.solution.pwm)
