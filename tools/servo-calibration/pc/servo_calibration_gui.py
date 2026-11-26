"""Windows/desktop GUI for the MicroPython arm. Requires pyserial."""
import json
import time
import tkinter as tk
from tkinter import ttk, messagebox
import serial
from serial.tools import list_ports
if __package__:
    from .joint_calibration import ANGLE_JOINTS, CALIBRATION_PATH, PWM_LIMITS_US, JointCalibration, load_calibrations, save_calibrations
else:
    from joint_calibration import ANGLE_JOINTS, CALIBRATION_PATH, PWM_LIMITS_US, JointCalibration, load_calibrations, save_calibrations


class App:
    def __init__(self, root):
        self.root = root
        root.title('Robot Arm | PWM Controller')
        root.geometry('900x540')
        root.minsize(720, 450)
        self.ser = None
        self.buffer = bytearray()
        self.rows = {}
        self.ready = False
        self.startup = None
        self.deadline = 0
        self.last_hello = 0
        self.phase = None
        self.phase_since = 0
        self.serial_log = []
        self.traceback_lines = []
        self.in_traceback = False
        self.await_pose = False
        self.parts = {}
        self.joint_index = 1
        self.joint_count = 6
        self.status = tk.StringVar(value='Disconnected')
        self.calibration_window = None
        self.calibration_load_error = None
        try:
            self.calibrations = load_calibrations()
        except (OSError, ValueError) as exc:
            self.calibrations = {}
            self.calibration_load_error = str(exc)
            self.status.set('Calibration file: ' + str(exc))
        self.port = tk.StringVar(value='COM10')
        self.live = tk.BooleanVar(value=True)
        self.style = ttk.Style()
        self.style.configure('Title.TLabel', font=('Segoe UI', 18, 'bold'))
        self.style.configure('Danger.TButton', foreground='#a51d2d')
        outer = ttk.Frame(root, padding=20)
        outer.pack(fill='both', expand=True)
        ttk.Label(outer, text='Robot arm control', style='Title.TLabel').pack(anchor='w')
        ttk.Label(outer, text='Joint control in PWM or calibrated degrees').pack(anchor='w', pady=(2,14))
        bar = ttk.Frame(outer); bar.pack(fill='x')
        ttk.Label(bar, text='Serial port').pack(side='left')
        self.combo = ttk.Combobox(bar, textvariable=self.port, width=12)
        self.combo.pack(side='left', padx=8)
        ttk.Button(bar, text='Refresh', command=self.refresh).pack(side='left')
        self.connect_button = ttk.Button(bar, text='Connect', command=self.connect)
        self.connect_button.pack(side='left', padx=8)
        ttk.Button(bar, text='Serial log', command=self.show_log).pack(side='left', padx=8)
        ttk.Checkbutton(bar, text='Live sliders', variable=self.live).pack(side='right')
        self.refresh()
        self.body = ttk.Frame(outer); self.body.pack(fill='both', expand=True, pady=14)
        ttk.Label(self.body, text='Connect to load joint settings. No joint is automatically enabled.').pack(anchor='w')
        bottom = ttk.Frame(outer); bottom.pack(fill='x')
        self.hold = ttk.Button(bottom, text='Hold / cancel motion', command=lambda: self.control('hold'), state='disabled')
        self.hold.pack(side='left')
        self.stop = ttk.Button(bottom, text='Disable all PWM', style='Danger.TButton', command=self.stop_all, state='disabled')
        self.stop.pack(side='left', padx=8)
        self.copy = ttk.Button(bottom, text='Copy current pose', command=self.copy_pose, state='disabled')
        self.copy.pack(side='right')
        ttk.Label(outer, textvariable=self.status, wraplength=850).pack(anchor='w', pady=(12,0))
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.after(20, self.poll)

    def show_log(self):
        win = tk.Toplevel(self.root)
        win.title('ESP32 serial log')
        win.geometry('850x450')
        text = tk.Text(win, wrap='word')
        text.pack(fill='both', expand=True)
        def update():
            if not win.winfo_exists():
                return
            log = '\n'.join(self.serial_log)
            if text.get('1.0', 'end-1c') != log:
                text.delete('1.0', 'end')
                text.insert('1.0', log)
                text.see('end')
            win.after(250, update)
        update()

    def refresh(self):
        ports = [p.device for p in list_ports.comports()]
        self.combo['values'] = ports
        if ports and self.port.get() not in ports:
            self.port.set(ports[0])

    def connect(self):
        if self.ser:
            self.disconnect(); return
        if not messagebox.askokcancel('Support the arm', 'Connect to the ESP32. If needed, the GUI will interrupt the current program and launch the serial bridge; this can disable PWM. Support the arm and keep its travel clear. MS24 may still hold torque. Continue?'):
            return
        try:
            s = serial.Serial(port=None, baudrate=115200, timeout=0, write_timeout=.25)
            s.port = self.port.get().strip()
            s.dtr = False; s.rts = False
            s.open(); self.ser = s
            self.buffer.clear(); self.ready = False
            self.parts = {}; self.joint_index = 1
            self.serial_log.clear(); self.traceback_lines.clear(); self.in_traceback = False
            self.connect_button.config(text='Disconnect')
            self.status.set('Waiting for the running ESP32 controller...')
            self.startup = None
            self.last_hello = 0
            self.phase = 'probe'
            self.phase_since = time.monotonic()
            self.deadline = self.phase_since + 30
        except (serial.SerialException, OSError) as exc:
            self.status.set(str(exc))

    def raw(self, data):
        if self.ser:
            try:
                self.ser.write(data)
                return True
            except (serial.SerialException, OSError) as exc:
                self.disconnect(str(exc))
        return False

    def send(self, line):
        return self.ready and self.raw((line+'\n').encode())

    def clear_pending(self):
        for r in self.rows.values():
            if r['job'] is not None:
                self.root.after_cancel(r['job']); r['job'] = None

    def disconnect(self, reason='Disconnected; last PWM may remain active.'):
        self.clear_pending()
        if self.ser:
            if self.ready:
                try: self.ser.write(b'hold\n')
                except (serial.SerialException, OSError): pass
            self.ser.close()
        self.ser = None; self.ready = False; self.startup = None
        self.phase = None
        self.await_pose = False
        if self.calibration_window is not None:
            self.calibration_window.destroy()
            self.calibration_window = None
        self.connect_button.config(text='Connect')
        for r in self.rows.values():
            for k in ('scale','entry','button','apply','units','calibrate'):
                r[k].config(state='disabled')
        for b in (self.hold,self.stop,self.copy): b.config(state='disabled')
        self.status.set(reason)

    def start_bridge(self):
        # Called only after seeing the friendly REPL prompt.
        self.phase = 'launch'
        self.phase_since = time.monotonic()
        self.in_traceback = False
        self.traceback_lines.clear()
        self.buffer.clear()
        self.status.set('MicroPython REPL detected; starting the serial bridge...')
        command = "import os, sys; sys.path.insert(0, '/robot_arm'); bridge = 'serial_bridge.py' if 'serial_bridge.py' in os.listdir('/robot_arm') else 'slider_control_v2.py'; exec(open('/robot_arm/' + bridge).read())"
        self.raw((command + '\r').encode())

    def poll(self):
        try:
            if self.ser:
                now = time.monotonic()
                self.buffer.extend(self.ser.read(min(8192, self.ser.in_waiting)))
                repl_seen = False
                while b'\n' in self.buffer:
                    line, _, rest = self.buffer.partition(b'\n')
                    self.buffer = bytearray(rest)
                    line = line.decode(errors='replace').strip()
                    if line:
                        self.serial_log.append(line)
                        self.serial_log = self.serial_log[-500:]
                        print('[ESP32]', line, flush=True)
                    if line.strip() == '>>>':
                        repl_seen = True
                    if line.startswith('@') or line.startswith('>>> @'):
                        try:
                            self.frame(json.loads(line[line.index('@')+1:]))
                        except (ValueError, KeyError, TypeError, IndexError) as exc:
                            self.status.set('Protocol error: ' + str(exc))
                    elif 'Traceback' in line:
                        self.in_traceback = True
                        self.traceback_lines = [line]
                    elif self.in_traceback and line:
                        if line.startswith('>>>'):
                            self.in_traceback = False
                        else:
                            self.traceback_lines.append(line)
                            self.status.set(line)
                # Friendly REPL prompt has no terminating newline.
                if self.buffer.strip().endswith(b'>>>'):
                    repl_seen = True
                if len(self.buffer) > 32768:
                    self.buffer.clear()
                if not self.ready:
                    if self.phase in ('probe', 'interrupt') and repl_seen:
                        self.start_bridge()
                    elif self.phase == 'probe':
                        if now - self.phase_since >= 3:
                            self.phase = 'interrupt'
                            self.phase_since = now
                            self.status.set('No bridge response; requesting MicroPython REPL...')
                            self.raw(b'\x03\x03\r')
                        elif now - self.phase_since >= .3 and now - self.last_hello >= 1:
                            self.last_hello = now
                            self.raw(b'hello2\r')
                    elif self.phase == 'interrupt' and now - self.phase_since >= 2:
                        # A second prompt request covers delayed USB enumeration.
                        self.phase = 'wait_prompt'
                        self.raw(b'\x03\r')
                    elif self.phase == 'wait_prompt' and repl_seen:
                        self.start_bridge()
                    elif self.phase == 'collect' and now - self.last_hello >= .8:
                        self.request_joint()
                    elif self.phase == 'launch' and not repl_seen and now - self.phase_since >= 1 and now - self.last_hello >= 1:
                        self.last_hello = now
                        self.raw(b'hello2\r')
                    elif self.phase == 'launch' and repl_seen and self.traceback_lines:
                        detail = self.traceback_lines[-1]
                        self.disconnect('ESP32 startup failed: ' + detail + ' | Open Serial log')
                    if self.ser and not self.ready and now > self.deadline:
                        detail = ('Incomplete joint settings: motor {}'.format(self.joint_index) if self.phase == 'collect' else (self.traceback_lines[-1] if self.traceback_lines else 'No valid short-frame response from MicroPython'))
                        self.disconnect('No handshake: ' + detail + ' | Open Serial log')
        except (serial.SerialException, OSError) as exc:
            self.disconnect(str(exc))
        finally:
            self.root.after(20, self.poll)

    def request_joint(self):
        self.last_hello = time.monotonic()
        self.raw(('joint {}\n'.format(self.joint_index)).encode())

    def frame(self, packet):
        if not isinstance(packet, list) or not packet:
            return  # Ignore the obsolete large-JSON protocol.
        kind = packet[0]
        if kind == 'h2' and len(packet) == 2:
            if self.ready or self.phase == 'collect':
                return
            if packet[1] != 6:
                raise ValueError('Expected six joints')
            self.parts = {}
            self.joint_count = packet[1]
            self.joint_index = 1
            self.phase = 'collect'
            self.deadline = time.monotonic() + 25
            self.status.set('Reading joint 1/6...')
            self.request_joint()
        elif kind in ('j', 'n', 'v') and self.phase == 'collect':
            if packet[1] != self.joint_index:
                return
            record = self.parts.setdefault(self.joint_index, {})
            if kind == 'j' and len(packet) == 6:
                _, _, lo, hi, center, current = packet
                if not all(type(v) is int for v in (lo, hi, center)) or not PWM_LIMITS_US[0] <= lo < hi <= PWM_LIMITS_US[1] or not lo <= center <= hi:
                    raise ValueError('Invalid joint limits')
                if current is not None and (type(current) is not int or not lo <= current <= hi):
                    raise ValueError('Invalid current pulse')
                record['j'] = (lo, hi, center, current)
            elif kind == 'n' and len(packet) == 4:
                if not isinstance(packet[2], str) or not isinstance(packet[3], str):
                    raise ValueError('Invalid joint name')
                record['n'] = packet[2:]
            elif kind == 'v' and len(packet) == 4:
                if not all(isinstance(v, (int, float)) and v > 0 for v in packet[2:]):
                    raise ValueError('Invalid trajectory settings')
                record['v'] = packet[2:]
            if all(k in record for k in ('j', 'n', 'v')):
                if self.joint_index < self.joint_count:
                    self.joint_index += 1
                    self.status.set('Reading joint {}/6...'.format(self.joint_index))
                    self.request_joint()
                else:
                    motors, current = {}, {}
                    for motor, row in self.parts.items():
                        lo, hi, center, p = row['j']
                        name, model = row['n']
                        speed, accel = row['v']
                        motors[str(motor)] = dict(min_us=lo, max_us=hi, center_us=center,
                                                  name=name, model=model, speed=speed, accel=accel)
                        current[str(motor)] = p
                    self.message(dict(type='hello', motors=motors, current=current))
        elif kind == 's' and self.ready and len(packet) == 4:
            self.message(dict(type='state', motor=packet[1], pulse=packet[2], busy=packet[3]))
        elif kind == 'p' and self.ready and len(packet) == 7:
            self.message(dict(type='pose', current={str(i): packet[i] for i in range(1, 7)}))
        elif kind in ('e', 'i') and len(packet) == 2:
            self.message(dict(type='error' if kind == 'e' else 'info', message=packet[1]))

    def message(self, msg):
        kind = msg['type']
        if kind == 'hello':
            if self.ready:
                return
            self.build(msg['motors'], msg['current'])
            self.ready = True
            self.phase = None
            self.in_traceback = False
            self.status.set('Calibration file: ' + self.calibration_load_error if self.calibration_load_error else
                            'Connected. Set a pulse close to the actual joint position before enabling.')
        elif kind == 'state':
            motor = int(msg['motor']); r = self.rows[motor]
            r['pulse'] = msg['pulse']
            r['busy'] = msg['busy']
            if not r['busy'] and (r['pulse'] == r['expected'] or r['holding']):
                r['awaiting'] = False
                r['holding'] = False
            r['enabled'] = r['pulse'] is not None
            r['button'].config(text='Off' if r['enabled'] else 'Enable', state='normal')
            self.update_output(motor)
        elif kind == 'pose':
            if self.await_pose:
                self.await_pose = False
                values = {int(k):v for k,v in msg['current'].items()}
                if any(v is None for v in values.values()):
                    self.status.set('Cannot copy a complete pose: enable all joints first.'); return
                text = 'HOME_POSITION = {\n' + ''.join('    {}: {},\n'.format(k,v) for k,v in sorted(values.items())) + '}'
                self.root.clipboard_clear(); self.root.clipboard_append(text)
                self.status.set('Commanded pose copied to clipboard (not measured angles).')
        elif kind in ('error','info'):
            self.status.set(msg['message'])

    def build(self, motors, current):
        self.clear_pending()
        for w in self.body.winfo_children(): w.destroy()
        self.rows.clear()
        for col, label in enumerate(('Joint','Target','Value','Command','Output','Calibration')):
            ttk.Label(self.body, text=label).grid(row=0,column=col,sticky='w',padx=5,pady=4)
        self.body.columnconfigure(1,weight=1)
        for row,(key,m) in enumerate(sorted(motors.items(),key=lambda x:int(x[0])),1):
            i=int(key); lo=int(m['min_us']); hi=int(m['max_us']); p=current.get(key)
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
            calibrate = ttk.Button(controls, text='Calibrate', command=lambda i=i: self.open_calibration(i))
            calibrate.pack(side='left', padx=(4, 0))
            if i not in ANGLE_JOINTS:
                calibrate.config(state='disabled')
            self.rows[i]={'lo':lo,'hi':hi,'value':value,'text':entry_var,'state':state,'enabled':p is not None,'job':None,'scale':scale,'entry':entry,'button':button,'apply':apply,
                          'pulse':p,'busy':None,'awaiting':False,'expected':None,'holding':False,'degrees':False,
                          'units':units,'calibrate':calibrate,'low':low,'high':high,'name':m['name']}
            self.update_output(i)
            scale.config(command=lambda v,i=i:self.slide(i,v))
            scale.bind('<ButtonRelease-1>',lambda e,i=i:self.release(i))
            entry.bind('<Return>',lambda e,i=i:self.apply(i))
        for b in (self.hold,self.stop,self.copy): b.config(state='normal')

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
        row['units'].config(state='normal' if calibration else 'disabled')
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

    def send_joint(self, op, i, pulse=None):
        command = '{} {}'.format(op, i)
        if pulse is not None:
            command += ' {}'.format(pulse)
        if self.send(command):
            row = self.rows[i]
            row['expected'] = pulse
            row['awaiting'] = True
            row['holding'] = False
            self.update_output(i)

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
        if r['enabled']: self.send_joint('move', i, p)
        else: self.status.set('Motor {} is off; press Enable to apply the selected pulse.'.format(i))

    def toggle(self,i):
        self.cancel_job(i); r=self.rows[i]
        if r['enabled']:
            self.send_joint('off', i)
        else:
            p=self.target(i)
            if p is None: return
            self.send_joint('enable', i, p)

    def control(self,op):
        self.clear_pending()
        if self.send(op):
            for i, row in self.rows.items():
                row['holding'] = op == 'hold'
                row['expected'] = None
                row['awaiting'] = True
                self.update_output(i)

    def stop_all(self):
        self.control('stop')

    def copy_pose(self):
        self.clear_pending()
        if any(r['busy'] or r['awaiting'] for r in self.rows.values()):
            self.status.set('Wait until all joints finish moving, then copy the pose.'); return
        self.await_pose=self.send('pose')

    def close(self):
        self.disconnect(); self.root.destroy()


if __name__=='__main__':
    root=tk.Tk(); App(root); root.mainloop()
