"""Standalone GUI for recording measured joint angles."""

from pathlib import Path
import sys
import tkinter as tk
from tkinter import messagebox, ttk

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from gui.joint_controls import JointControlApp
from robot.joint_calibration import ANGLE_JOINTS, CALIBRATION_PATH, JointCalibration, save_calibrations


class App(JointControlApp):
    def __init__(self, root):
        self.calibration_window = None
        super().__init__(root, title='Servo calibration',
                         subtitle='Set PWM and record measured joint angles', size='900x540', minsize=(720, 450))

    def on_disconnected(self):
        self.close_calibration()

    def add_row_action(self, parent, motor):
        if motor not in ANGLE_JOINTS:
            return None
        button = ttk.Button(parent, text='Calibrate', command=lambda: self.open_calibration(motor))
        button.pack(side='left', padx=(4, 0))
        return button

    def open_calibration(self, i):
        if self.calibration_window is not None:
            self.calibration_window.destroy()
        row = self.rows[i]
        if row['degrees']:
            pulse = row['angle_calibration'].angle_to_pwm(row['value'].get())
            self.set_units(i, False, pulse)
        win = self.calibration_window = tk.Toplevel(self.root)
        win.title('Angle calibration | {}'.format(row['name']))
        panel = ttk.Frame(win, padding=12)
        panel.pack(fill='both', expand=True)
        ttk.Label(panel, text='Set PWM in the main window, wait for motion to stop,\n'
                              'then measure the joint angle and record it.').pack(anchor='w')
        live_state = ttk.Label(panel, textvariable=row['state'])
        live_state.pack(anchor='w', pady=4)
        points = tk.Listbox(panel, height=6, exportselection=False)
        points.pack(fill='both', expand=True)
        angle = ttk.Entry(panel)
        angle.pack(fill='x', pady=5)
        ttk.Label(panel, text='Measured angle (°)').pack(anchor='w')
        feedback = tk.StringVar()
        ttk.Label(panel, textvariable=feedback, wraplength=380).pack(anchor='w', pady=5)
        original = self.calibrations.get(i, JointCalibration())
        draft = original.points

        def refresh():
            points.delete(0, 'end')
            for pulse, degrees in draft:
                points.insert('end', '{} us   →   {:g}°'.format(pulse, degrees))
            feedback.set('{} point(s). At least two are required for degree control.'.format(len(draft)))

        def record():
            nonlocal draft
            if self.motion_locked:
                feedback.set('Wait until motion is finished before recording.')
                return
            if row['pulse'] is None or row['busy'] is not False or row['awaiting'] or row['job'] is not None:
                feedback.set('Wait for a confirmed, stationary PWM output before recording.')
                return
            try:
                value = float(angle.get().strip().replace(',', '.'))
                draft = JointCalibration((*draft, (row['pulse'], value)), (row['lo'], row['hi'])).points
            except ValueError as exc:
                feedback.set(str(exc))
                return
            angle.delete(0, 'end')
            refresh()

        def remove():
            nonlocal draft
            selection = points.curselection()
            if selection:
                index = selection[0]
                draft = draft[:index] + draft[index + 1:]
                refresh()

        def save():
            if self.motion_locked:
                feedback.set('Wait until motion is finished before saving.')
                return
            try:
                calibration = JointCalibration(draft, (row['lo'], row['hi']),
                                               extrapolation_pwm=original.extrapolation_pwm)
                if self.calibration_load_error:
                    if not messagebox.askyesno('Replace calibration file',
                                              'Replace the unreadable calibration file?', parent=win):
                        return
                updated = dict(self.calibrations)
                if calibration.points:
                    updated[i] = calibration
                else:
                    updated.pop(i, None)
                save_calibrations(updated)
            except (OSError, ValueError) as exc:
                feedback.set(str(exc))
                return
            if row['degrees']:
                pulse = row['angle_calibration'].angle_to_pwm(row['value'].get())
                self.set_units(i, False, pulse)
            self.calibrations = updated
            self.calibration_load_error = None
            self.update_output(i)
            feedback.set('Saved to ' + CALIBRATION_PATH.name)

        buttons = ttk.Frame(panel)
        buttons.pack(fill='x')
        for label, command in (('Record point', record), ('Remove selected', remove), ('Save', save)):
            ttk.Button(buttons, text=label, command=command).pack(side='left', padx=(0, 5))
        refresh()
        win.protocol('WM_DELETE_WINDOW', self.close_calibration)

    def close_calibration(self):
        if self.calibration_window is not None:
            self.calibration_window.destroy()
            self.calibration_window = None

if __name__ == '__main__':
    root = tk.Tk()
    App(root)
    root.mainloop()
