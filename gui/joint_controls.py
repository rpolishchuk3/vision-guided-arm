"""Joint slider and connection widgets shared by the two desktop applications."""

import tkinter as tk
from tkinter import messagebox, simpledialog, ttk

from robot.ble_transport import SCAN_SECONDS
from robot.client import RobotClient
from robot.joint_calibration import load_calibrations


class JointControlApp:
    def __init__(self, root, *, title, subtitle, size, minsize):
        self.root = root
        root.title(title)
        root.geometry(size)
        root.minsize(*minsize)
        self.rows = {}
        self.client = RobotClient(self.on_event)
        self.status = tk.StringVar(value='Disconnected')
        self.reload_calibration()
        if self.calibration_load_error:
            self.status.set('Calibration file: ' + self.calibration_load_error)
        self.port = tk.StringVar(value='COM10')
        self.live = tk.BooleanVar(value=True)
        style = ttk.Style()
        style.configure('Title.TLabel', font=('Segoe UI', 18, 'bold'))
        style.configure('Danger.TButton', foreground='#a51d2d')
        outer = ttk.Frame(root, padding=20)
        outer.pack(fill='both', expand=True)
        ttk.Label(outer, text=title, style='Title.TLabel').pack(anchor='w')
        ttk.Label(outer, text=subtitle).pack(anchor='w', pady=(2, 14))
        bar = ttk.Frame(outer)
        bar.pack(fill='x')
        self.connection_mode = tk.StringVar(value='USB Serial')
        self.mode_combo = ttk.Combobox(bar, textvariable=self.connection_mode,
                                      values=('USB Serial', 'Bluetooth LE'), state='readonly', width=13)
        self.mode_combo.pack(side='left', padx=(0, 8))
        self.mode_combo.bind('<<ComboboxSelected>>', self.show_transport)
        self.transport_options = ttk.Frame(bar)
        self.transport_options.pack(side='left')
        self.usb_options = ttk.Frame(self.transport_options)
        self.combo = ttk.Combobox(self.usb_options, textvariable=self.port, width=12)
        self.combo.pack(side='left', padx=(0, 8))
        self.refresh_button = ttk.Button(self.usb_options, text='Refresh', command=self.refresh_ports)
        self.refresh_button.pack(side='left')
        self.ble_options = ttk.Frame(self.transport_options)
        self.ble_device = tk.StringVar()
        self.ble_combo = ttk.Combobox(self.ble_options, textvariable=self.ble_device, state='readonly', width=27)
        self.ble_combo.pack(side='left', padx=(0, 8))
        self.ble_combo.bind('<<ComboboxSelected>>', lambda _event: self.update_connection_controls())
        self.scan_button = ttk.Button(self.ble_options, text='Scan', command=self.scan_ble)
        self.scan_button.pack(side='left')
        self.connect_button = ttk.Button(bar, text='Connect', command=self.connect)
        self.connect_button.pack(side='left', padx=8)
        ttk.Button(bar, text='Serial log', command=self.show_log).pack(side='left', padx=8)
        ttk.Checkbutton(bar, text='Live sliders', variable=self.live).pack(side='right')
        self.refresh_ports()
        self.show_transport()
        self.body = ttk.Frame(outer)
        self.body.pack(fill='both', expand=True, pady=14)
        ttk.Label(self.body, text='Connect to load joint settings. No joint is automatically enabled.').pack(anchor='w')
        self.extra_panel = ttk.Frame(outer)
        self.extra_panel.pack(fill='x')
        self.bottom = ttk.Frame(outer)
        self.bottom.pack(fill='x')
        self.hold = ttk.Button(self.bottom, text='Hold / cancel motion', command=lambda: self.control(hold=True), state='disabled')
        self.hold.pack(side='left')
        self.stop = ttk.Button(self.bottom, text='Disable all PWM', style='Danger.TButton',
                               command=lambda: self.control(hold=False), state='disabled')
        self.stop.pack(side='left', padx=8)
        ttk.Label(outer, textvariable=self.status, wraplength=850).pack(anchor='w', pady=(12, 0))
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.after(20, self.poll)

    @property
    def ready(self):
        return self.client.ready

    @property
    def motion_locked(self):
        return False

    def reload_calibration(self):
        self.calibration_load_error = None
        try:
            self.calibrations = load_calibrations()
        except (OSError, ValueError) as exc:
            self.calibrations = {}
            self.calibration_load_error = str(exc)

    def refresh_ports(self):
        ports = self.client.ports()
        self.combo['values'] = ports
        if ports and self.port.get() not in ports:
            self.port.set(ports[0])

    def show_transport(self, _event=None):
        self.usb_options.pack_forget()
        self.ble_options.pack_forget()
        panel = self.ble_options if self.connection_mode.get() == 'Bluetooth LE' else self.usb_options
        panel.pack(side='left')
        self.update_connection_controls()

    def update_connection_controls(self):
        connected = self.client.connected
        locked = connected or self.client.scanning
        self.mode_combo.config(state='disabled' if locked else 'readonly')
        self.combo.config(state='disabled' if locked else 'normal')
        self.refresh_button.config(state='disabled' if locked else 'normal')
        self.ble_combo.config(state='disabled' if locked else 'readonly')
        self.scan_button.config(state='disabled' if locked else 'normal')
        selection = self.connection_mode.get() == 'USB Serial' or bool(self.ble_device.get())
        connect_enabled = connected or (selection and not self.client.scanning)
        self.connect_button.config(text='Disconnect' if connected else 'Connect',
                                   state='normal' if connect_enabled else 'disabled')

    def scan_ble(self):
        self.client.scan_ble()
        self.status.set(f'Scanning for RobotArm over Bluetooth LE (up to {SCAN_SECONDS} seconds)...')
        self.update_connection_controls()

    def connect(self):
        if self.client.connected:
            self.client.disconnect()
            return
        if self.connection_mode.get() == 'Bluetooth LE':
            key = simpledialog.askstring('RobotArm BLE key', 'Enter the 64-character hexadecimal key.',
                                         parent=self.root, show='*')
            if key is not None:
                self.client.connect_ble(self.ble_device.get(), key)
            return
        if messagebox.askokcancel(
            'Support the arm',
            'Connect to the ESP32. If needed, the GUI will interrupt the current program and launch the serial '
            'bridge; this can disable PWM. Support the arm and keep its travel clear. MS24 may still hold torque. Continue?',
        ):
            self.client.connect(self.port.get())

    def poll(self):
        try:
            self.client.poll()
        finally:
            self.root.after(20, self.poll)

    def on_event(self, event):
        kind = event['type']
        if kind == 'connected':
            self.update_connection_controls()
        elif kind == 'disconnected':
            self.clear_pending()
            self.update_connection_controls()
            self.on_disconnected()
            self.update_motion_controls()
            self.status.set(event['message'])
        elif kind == 'scan':
            devices = event['devices']
            self.ble_combo['values'] = devices
            self.ble_device.set(devices[0] if devices else '')
            self.update_connection_controls()
            self.status.set('Select RobotArm and click Connect.' if devices else 'No RobotArm BLE device found.')
        elif kind == 'scan_error':
            self.update_connection_controls()
            self.status.set(event['message'])
        elif kind == 'hello':
            self.reload_calibration()
            self.build(event['motors'], event['current'])
            self.update_motion_controls()
            self.status.set('Calibration file: ' + self.calibration_load_error if self.calibration_load_error else
                            'Connected. Set a pulse close to the actual joint position before enabling.')
        elif kind == 'state':
            motor = int(event['motor'])
            row = self.rows[motor]
            row['pulse'] = event['pulse']
            row['busy'] = event['busy']
            if not row['busy'] and (row['pulse'] == row['expected'] or row['holding']):
                row['awaiting'] = False
                row['holding'] = False
            row['enabled'] = row['pulse'] is not None
            row['button'].config(text='Off' if row['enabled'] else 'Enable')
            self.update_output(motor)
            self.on_view_changed()
        elif kind in ('status', 'error', 'info'):
            self.status.set(event['message'])
        else:
            self.on_robot_event(event)

    def on_disconnected(self):
        pass

    def on_robot_event(self, event):
        pass

    def on_view_changed(self):
        pass

    def add_row_action(self, parent, motor):
        return None

    def update_motion_controls(self):
        enabled = self.ready and not self.motion_locked
        state = 'normal' if enabled else 'disabled'
        for button in (self.hold, self.stop):
            button.config(state='normal' if self.ready else 'disabled')
        for i, row in self.rows.items():
            for key in ('scale', 'entry', 'button', 'apply', 'action'):
                widget = row[key]
                if widget is not None:
                    widget.config(state=state)
            self.update_output(i)
        self.on_view_changed()

    def send_joint(self, command, i, pulse=None):
        if self.motion_locked:
            return
        sent = command(i) if pulse is None else command(i, pulse)
        if sent:
            row = self.rows[i]
            row['expected'] = pulse
            row['awaiting'] = True
            row['holding'] = False
            self.update_output(i)
            self.on_view_changed()

    def control(self, *, hold):
        self.clear_pending()
        if (self.client.hold if hold else self.client.stop)():
            for i, row in self.rows.items():
                row['holding'] = hold
                row['expected'] = None
                row['awaiting'] = True
                self.update_output(i)
            self.on_view_changed()

    def close(self):
        self.client.shutdown()
        self.root.destroy()

    def show_log(self):
        win = tk.Toplevel(self.root)
        win.title('ESP32 serial log')
        win.geometry('850x450')
        text = tk.Text(win, wrap='word')
        text.pack(fill='both', expand=True)
        def update():
            if not win.winfo_exists():
                return
            log = '\n'.join(self.client.log)
            if text.get('1.0', 'end-1c') != log:
                text.delete('1.0', 'end')
                text.insert('1.0', log)
                text.see('end')
            win.after(250, update)
        update()

    def clear_pending(self):
        for r in self.rows.values():
            if r['job'] is not None:
                self.root.after_cancel(r['job']); r['job'] = None

    def build(self, motors, current):
        self.clear_pending()
        for w in self.body.winfo_children(): w.destroy()
        self.rows.clear()
        for col, label in enumerate(('Joint', 'Target', 'Value', 'Command', 'Output', 'Units')):
            ttk.Label(self.body, text=label).grid(row=0,column=col,sticky='w',padx=5,pady=4)
        self.body.columnconfigure(1,weight=1)
        for row, (i, m) in enumerate(sorted(motors.items()), 1):
            lo, hi, p = m['min_us'], m['max_us'], current[i]
            value=tk.DoubleVar(value=p if p is not None else m['center_us'])
            entry_var=tk.StringVar(value=str(int(value.get())))
            state=tk.StringVar(value='PWM off' if p is None else 'Set: {} us'.format(p))
            ttk.Label(self.body,text='{}  {}\n{}'.format(i,m['name'],m['model'])).grid(row=row,column=0,sticky='w',pady=7,padx=5)
            sf=ttk.Frame(self.body); sf.grid(row=row,column=1,sticky='ew',padx=8)
            scale=ttk.Scale(sf,from_=lo,to=hi,variable=value)
            scale.pack(fill='x')
            limits=ttk.Frame(sf); limits.pack(fill='x')
            low = ttk.Label(limits, text=str(lo)); low.pack(side='left')
            high = ttk.Label(limits, text=str(hi)); high.pack(side='right')
            entry=ttk.Entry(self.body,textvariable=entry_var,width=7); entry.grid(row=row,column=2,padx=5)
            bf=ttk.Frame(self.body); bf.grid(row=row,column=3,padx=5)
            button=ttk.Button(bf,text='Enable' if p is None else 'Off',width=8,command=lambda i=i:self.toggle(i)); button.pack(side='left')
            apply=ttk.Button(bf,text='Set',width=5,command=lambda i=i:self.apply(i)); apply.pack(side='left',padx=3)
            ttk.Label(self.body,textvariable=state,width=21).grid(row=row,column=4,sticky='w',padx=5)
            controls = ttk.Frame(self.body); controls.grid(row=row, column=5, sticky='w')
            units = ttk.Button(controls, text='µs', width=4, command=lambda i=i: self.switch_units(i))
            units.pack(side='left')
            action = self.add_row_action(controls, i)
            self.rows[i]={'lo':lo,'hi':hi,'value':value,'text':entry_var,'state':state,'enabled':p is not None,'job':None,'scale':scale,'entry':entry,'button':button,'apply':apply,
                          'pulse':p,'busy':None,'awaiting':False,'expected':None,'holding':False,'degrees':False,
                          'units':units,'action':action,'low':low,'high':high,'name':m['name']}
            self.update_output(i)
            scale.config(command=lambda v,i=i:self.slide(i,v))
            scale.bind('<ButtonRelease-1>',lambda e,i=i:self.release(i))
            entry.bind('<Return>',lambda e,i=i:self.apply(i))

    def valid_calibration(self, i):
        row = self.rows[i]
        calibration = self.calibrations.get(i)
        if calibration and calibration.ready:
            if all(row['lo'] <= pulse <= row['hi'] for pulse, _ in calibration.points):
                return calibration.with_servo_limits((row['lo'], row['hi']))
        return None

    def update_output(self, i):
        row = self.rows[i]
        calibration = self.valid_calibration(i)
        row['angle_calibration'] = calibration
        row['units'].config(state='normal' if calibration and self.ready and not self.motion_locked else 'disabled')
        pulse = row['pulse']
        if row['busy'] or row['awaiting']:
            output = 'Moving / pending'
        elif pulse is None:
            output = 'PWM off'
        else:
            output = '{} us'.format(pulse)
            if calibration:
                try:
                    output += ' / ~{:.1f}°'.format(calibration.pwm_to_angle(pulse))
                    if calibration.is_extrapolated(pulse):
                        output += '*'
                except ValueError:
                    output += ' / out of range'
        row['state'].set(output)

    def show_target(self, i, pulse):
        row = self.rows[i]
        value = row['angle_calibration'].pwm_to_angle(pulse) if row['degrees'] else pulse
        row['value'].set(value)
        row['text'].set(str(value))

    def set_units(self, i, degrees, pulse):
        row = self.rows[i]
        self.cancel_job(i)
        calibration = row['angle_calibration']
        lo, hi = calibration.angle_limits if degrees else (row['lo'], row['hi'])
        row['degrees'] = degrees
        row['scale'].config(from_=lo, to=hi)
        row['units'].config(text='°' if degrees else 'µs')
        for label, value in ((row['low'], lo), (row['high'], hi)):
            estimated = degrees and calibration.is_extrapolated(calibration.angle_to_pwm(value))
            label.config(text='{:.1f}*'.format(value) if estimated else '{:g}'.format(value))
        self.show_target(i, pulse)

    def switch_units(self, i):
        row = self.rows[i]
        pulse = self.target(i)
        if pulse is None:
            return
        if not row['degrees']:
            calibration = row['angle_calibration']
            if calibration is None:
                return
            try:
                calibration.pwm_to_angle(pulse)
            except ValueError:
                self.status.set('Select a PWM target within the degree-control range before switching units.')
                return
            if calibration.extrapolation_pwm is not None:
                self.status.set('* marks an extrapolated angle beyond the measured points.')
        self.set_units(i, not row['degrees'], pulse)

    def target(self,i):
        r=self.rows[i]
        try:
            raw = r['text'].get().strip()
            if r['degrees']:
                val = r['angle_calibration'].angle_to_pwm(float(raw.replace(',', '.')))
            else:
                val = int(raw)
            if not r['lo']<=val<=r['hi']: raise ValueError('Pulse outside PWM limits')
            return val
        except ValueError:
            lo, hi = r['angle_calibration'].angle_limits if r['degrees'] else (r['lo'], r['hi'])
            self.status.set('Motor {}: enter {} to {} {}'.format(i, lo, hi, '°' if r['degrees'] else 'us'))
            return None

    def slide(self,i,v):
        r=self.rows[i]
        r['text'].set(str(float(v)) if r['degrees'] else str(round(float(v))))
        if r['enabled'] and self.live.get() and self.ready and r['job'] is None:
            r['job']=self.root.after(100,lambda:self.flush(i))

    def flush(self,i):
        r=self.rows[i]; r['job']=None
        if self.live.get() and r['enabled']:
            self.apply(i)

    def release(self,i):
        if self.live.get() and self.rows[i]['enabled']:
            self.cancel_job(i); self.apply(i)

    def cancel_job(self,i):
        r=self.rows[i]
        if r['job'] is not None: self.root.after_cancel(r['job']); r['job']=None

    def apply(self,i):
        p=self.target(i)
        if p is None: return
        r=self.rows[i]
        self.show_target(i, p)
        if r['enabled']: self.send_joint(self.client.move, i, p)
        else: self.status.set('Motor {} is off; press Enable to apply the selected pulse.'.format(i))

    def toggle(self,i):
        self.cancel_job(i); r=self.rows[i]
        if r['enabled']:
            self.send_joint(self.client.disable, i)
        else:
            p=self.target(i)
            if p is None: return
            self.send_joint(self.client.enable, i, p)
