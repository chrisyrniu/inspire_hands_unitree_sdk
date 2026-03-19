#!/usr/bin/env python3
"""
Real-time visualization for two Inspire RH56F1 hands via RS485.

Displays per-joint angle, force curves and per-finger/palm touch bar charts
for left and right hands on separate serial ports.

Usage:
    python vision_driver_485_double.py --port-left /dev/ttyUSB0 --port-right /dev/ttyUSB1
    python vision_driver_485_double.py --port-right /dev/ttyUSB0   # right hand only
"""

import sys
import os
import struct
import argparse
import threading
import time

import numpy as np
import pyqtgraph as pg
from PyQt5 import QtCore
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QTabWidget, QWidget,
    QGridLayout, QVBoxLayout, QLabel, QSplitter,
)
from PyQt5.QtCore import Qt

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from inspire_f1_sdk.registers import (
    REG_ANGLE_ACT, REG_FORCE_ACT, REG_CURRENT_ACT,
    REG_ERROR, REG_STATUS, REG_TEMP, REG_TOUCH_BASE,
    NUM_DOFS, DOF_NAMES, STATUS_CODES, decode_error, decode_status,
)
from pymodbus.client import ModbusSerialClient

HISTORY_LEN = 200
TOUCH_FINGER_NAMES = ['little', 'ring', 'middle', 'index', 'thumb']
TOUCH_PALM_NAMES = ['palm_left', 'palm_center', 'palm_right']
FINGER_COLORS = ['#e6194b', '#3cb44b', '#4363d8', '#f58231', '#911eb4', '#42d4f4']


class ModbusReader:
    """Reads F1 hand state + touch registers over RS485 Modbus RTU."""

    STATE_GROUPS = [
        ('ANGLE_ACT',  REG_ANGLE_ACT,  6),
        ('FORCE_ACT',  REG_FORCE_ACT,  6),
        ('CURRENT',    REG_CURRENT_ACT, 6),
        ('ERROR',      REG_ERROR,      6),
        ('STATUS',     REG_STATUS,     6),
        ('TEMP',       REG_TEMP,       6),
    ]

    def __init__(self, port, device_id=1, baudrate=115200):
        self.port = port
        self.device_id = device_id
        self.client = ModbusSerialClient(
            port=port, baudrate=baudrate, bytesize=8,
            stopbits=1, parity='N', timeout=1,
        )
        self._lock = threading.Lock()
        if not self.client.connect():
            raise ConnectionError(f"Cannot open {port}")

    def read_state(self):
        start = REG_ANGLE_ACT
        total = REG_TEMP + 6 - REG_ANGLE_ACT
        with self._lock:
            resp = self.client.read_holding_registers(start, count=total, device_id=self.device_id)
        if resp.isError():
            return None
        packed = struct.pack('>' + 'H' * total, *resp.registers)
        raw = list(struct.unpack('>' + 'h' * total, packed))

        result = {}
        for name, addr, cnt in self.STATE_GROUPS:
            off = addr - start
            result[name] = raw[off:off + cnt]
        return result

    def read_touch(self):
        with self._lock:
            resp = self.client.read_holding_registers(REG_TOUCH_BASE, count=34, device_id=self.device_id)
        if resp.isError():
            return None
        vals = list(resp.registers)
        touch = {}
        for i, fn in enumerate(TOUCH_FINGER_NAMES):
            base = i * 5
            touch[fn] = {
                'normal': vals[base], 'tangential': vals[base + 1],
                'direction': vals[base + 2],
                'proximity': vals[base + 3] | (vals[base + 4] << 16),
            }
        for j, pn in enumerate(TOUCH_PALM_NAMES):
            base = 25 + j * 3
            touch[pn] = {
                'normal': vals[base], 'tangential': vals[base + 1],
                'direction': vals[base + 2],
            }
        return touch

    def close(self):
        self.client.close()


class HandPanel(QWidget):
    """Curves + touch visualization for a single hand."""

    def __init__(self, side_label, read_touch=True):
        super().__init__()
        self.read_touch = read_touch
        self.history = {
            name: [np.zeros(HISTORY_LEN) for _ in range(6)]
            for name in ['ANGLE_ACT', 'FORCE_ACT', 'CURRENT', 'ERROR', 'STATUS', 'TEMP']
        }

        root_layout = QVBoxLayout()
        self.setLayout(root_layout)

        self.title_label = QLabel(f"  {side_label}")
        self.title_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        root_layout.addWidget(self.title_label)

        self.error_label = QLabel("ERROR: —")
        self.error_label.setStyleSheet("color: #cc0000;")
        self.status_label = QLabel("STATUS: —")
        root_layout.addWidget(self.error_label)
        root_layout.addWidget(self.status_label)

        splitter = QSplitter(Qt.Vertical)
        root_layout.addWidget(splitter)

        curve_widget = QWidget()
        curve_layout = QGridLayout()
        curve_widget.setLayout(curve_layout)
        splitter.addWidget(curve_widget)

        plot_names = ['ANGLE_ACT', 'FORCE_ACT', 'CURRENT', 'TEMP']
        self.plot_widgets = {}
        self.curves = {}
        for i, name in enumerate(plot_names):
            pw = pg.PlotWidget(title=name)
            pw.setBackground((30, 30, 30))
            pw.addLegend(offset=(60, 10))
            pw.showGrid(x=True, y=True, alpha=0.2)
            curve_layout.addWidget(pw, i // 2, i % 2)
            self.plot_widgets[name] = pw
            self.curves[name] = [
                pw.plot(pen=pg.mkPen(FINGER_COLORS[j], width=2), name=DOF_NAMES[j])
                for j in range(6)
            ]

        if read_touch:
            touch_widget = QWidget()
            touch_layout = QGridLayout()
            touch_widget.setLayout(touch_layout)
            splitter.addWidget(touch_widget)

            self.normal_bar = pg.PlotWidget(title="Touch Normal Force")
            self.normal_bar.setBackground((30, 30, 30))
            touch_layout.addWidget(self.normal_bar, 0, 0)

            self.tangential_bar = pg.PlotWidget(title="Touch Tangential Force")
            self.tangential_bar.setBackground((30, 30, 30))
            touch_layout.addWidget(self.tangential_bar, 0, 1)

            self.proximity_bar = pg.PlotWidget(title="Touch Proximity")
            self.proximity_bar.setBackground((30, 30, 30))
            touch_layout.addWidget(self.proximity_bar, 1, 0)

            self.direction_bar = pg.PlotWidget(title="Touch Direction")
            self.direction_bar.setBackground((30, 30, 30))
            touch_layout.addWidget(self.direction_bar, 1, 1)

            all_touch_names = TOUCH_FINGER_NAMES + TOUCH_PALM_NAMES
            x_ticks = list(enumerate(all_touch_names))
            for bw in [self.normal_bar, self.tangential_bar, self.proximity_bar, self.direction_bar]:
                ax = bw.getAxis('bottom')
                ax.setTicks([x_ticks])

            bar_colors = ['#e6194b', '#3cb44b', '#4363d8', '#f58231', '#911eb4',
                          '#aaffc3', '#ffe119', '#fabebe']
            self._bar_brushes = [pg.mkBrush(c) for c in bar_colors]

            splitter.setSizes([600, 400])

    def update_state(self, state_dict):
        if state_dict is None:
            return
        for name in ['ANGLE_ACT', 'FORCE_ACT', 'CURRENT', 'TEMP']:
            vals = state_dict.get(name)
            if vals is None:
                continue
            for j in range(6):
                self.history[name][j] = np.roll(self.history[name][j], -1)
                self.history[name][j][-1] = vals[j]
                self.curves[name][j].setData(self.history[name][j])

        errs = state_dict.get('ERROR', [0] * 6)
        parts = []
        for i, e in enumerate(errs):
            reasons = decode_error(int(e))
            tag = ', '.join(reasons) if reasons else 'OK'
            parts.append(f"{DOF_NAMES[i]}:{tag}")
        self.error_label.setText("ERROR: " + "  |  ".join(parts))

        statuses = state_dict.get('STATUS', [0] * 6)
        st_parts = [f"{DOF_NAMES[i]}:{decode_status(int(s))}" for i, s in enumerate(statuses)]
        self.status_label.setText("STATUS: " + "  |  ".join(st_parts))

    def update_touch(self, touch_dict):
        if not self.read_touch or touch_dict is None:
            return
        all_names = TOUCH_FINGER_NAMES + TOUCH_PALM_NAMES
        normals, tangentials, directions, proximities = [], [], [], []
        for name in all_names:
            d = touch_dict.get(name, {})
            normals.append(d.get('normal', 0))
            tangentials.append(d.get('tangential', 0))
            directions.append(d.get('direction', 0))
            proximities.append(d.get('proximity', 0))

        x = np.arange(len(all_names))
        for bw, vals in [(self.normal_bar, normals), (self.tangential_bar, tangentials),
                         (self.proximity_bar, proximities), (self.direction_bar, directions)]:
            bw.clear()
            bg = pg.BarGraphItem(
                x=x, height=vals, width=0.6,
                brushes=self._bar_brushes[:len(all_names)],
            )
            bw.addItem(bg)
            ax = bw.getAxis('bottom')
            ax.setTicks([list(enumerate(all_names))])


class VisionWindow(QMainWindow):
    def __init__(self, readers, read_touch=True, dt_ms=80):
        super().__init__()
        self.readers = readers
        self.read_touch = read_touch
        self.setWindowTitle("Inspire F1 Vision Driver (RS485)")
        self.setGeometry(50, 50, 1400, 900)

        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        self.panels = {}
        for side, reader in readers.items():
            label = f"{side.upper()} hand  ({reader.port})"
            panel = HandPanel(label, read_touch=read_touch)
            self.panels[side] = panel
            self.tabs.addTab(panel, side.upper())

        self.freq_label = QLabel("  freq: —")
        self.statusBar().addPermanentWidget(self.freq_label)

        self._call_count = 0
        self._t0 = time.perf_counter()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._tick)
        self.timer.start(dt_ms)

    def _tick(self):
        results = {}

        def _read_one(side, reader):
            state = reader.read_state()
            touch = reader.read_touch() if self.read_touch else None
            results[side] = (state, touch)

        if len(self.readers) > 1:
            threads = []
            for side, reader in self.readers.items():
                t = threading.Thread(target=_read_one, args=(side, reader))
                t.start()
                threads.append(t)
            for t in threads:
                t.join()
        else:
            for side, reader in self.readers.items():
                _read_one(side, reader)

        for side, (state, touch) in results.items():
            self.panels[side].update_state(state)
            self.panels[side].update_touch(touch)

        self._call_count += 1
        if self._call_count % 10 == 0:
            elapsed = time.perf_counter() - self._t0
            freq = self._call_count / elapsed if elapsed > 0 else 0
            self.freq_label.setText(f"  freq: {freq:.1f} Hz")

    def closeEvent(self, event):
        self.timer.stop()
        for r in self.readers.values():
            r.close()
        event.accept()


def main():
    parser = argparse.ArgumentParser(description='F1 RS485 Vision Driver (double)')
    parser.add_argument('--port-left', type=str, default=None,
                        help='Left hand serial port (omit to skip left)')
    parser.add_argument('--port-right', type=str, default=None,
                        help='Right hand serial port (omit to skip right)')
    parser.add_argument('--id-left', type=int, default=1)
    parser.add_argument('--id-right', type=int, default=1)
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--no-touch', action='store_true',
                        help='Skip touch sensor reads')
    parser.add_argument('--dt', type=int, default=50,
                        help='Refresh interval in ms (default: 50)')
    args = parser.parse_args()

    if args.port_left is None and args.port_right is None:
        parser.error("Provide at least one of --port-left or --port-right")

    readers = {}
    if args.port_left:
        readers['left'] = ModbusReader(args.port_left, args.id_left, args.baudrate)
        print(f"Left  hand: {args.port_left} (id={args.id_left})")
    if args.port_right:
        readers['right'] = ModbusReader(args.port_right, args.id_right, args.baudrate)
        print(f"Right hand: {args.port_right} (id={args.id_right})")

    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    win = VisionWindow(readers, read_touch=not args.no_touch, dt_ms=args.dt)
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
