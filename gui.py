#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import threading
import tkinter as tk
from tkinter import messagebox
import serial
import time


# ---------- Modbus 驱动器 ----------
class ModbusMotor:
    def __init__(self, port="/dev/ttyUSB1", baudrate=115200, slave_id=1, timeout=1):
        self.slave_id = slave_id
        self.serial = serial.Serial(port=port, baudrate=baudrate,
                                    bytesize=8, parity='N', stopbits=1,
                                    timeout=timeout)
        if not self.serial.is_open:
            raise Exception("串口打开失败")
        print("已连接 Modbus 驱动器")

        # 寄存器偏移
        self.trigger   = 0x6002
        self.pr0_mode  = 0x6200
        self.pos_high  = 0x6201
        self.pos_low   = 0x6202
        self.speed     = 0x6203
        self.acc       = 0x6204
        self.dec       = 0x6205
        self.enable_addr = 0x000F

        self.move_init()
        self.enable()

    # ---------- CRC ----------
    def _calculate_crc(self, data):
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                crc = (crc >> 1) ^ 0xA001 if (crc & 1) else crc >> 1
        return crc

    # ---------- 组帧 ----------
    def _create_modbus_frame(self, func, addr, value=None, cnt=1):
        if func == 0x03:
            frame = bytearray([self.slave_id, func, addr >> 8, addr & 0xFF, cnt >> 8, cnt & 0xFF])
        elif func == 0x06:
            frame = bytearray([self.slave_id, func, addr >> 8, addr & 0xFF, value >> 8, value & 0xFF])
        else:
            raise ValueError("不支持功能码")
        crc = self._calculate_crc(frame)
        frame += bytearray([crc & 0xFF, crc >> 8])
        return frame

    # ---------- 收发 ----------
    def write_register(self, addr, val):
        frame = self._create_modbus_frame(0x06, addr, val)
        self.serial.reset_input_buffer()
        self.serial.write(frame)
        time.sleep(0.02)
        rsp = self.serial.read(8)
        if len(rsp) != 8 or rsp[0] != self.slave_id or rsp[1] != 0x06:
            raise Exception("写寄存器异常")

    def read_holding_registers(self, addr, cnt=1):
        frame = self._create_modbus_frame(0x03, addr, cnt=cnt)
        self.serial.reset_input_buffer()
        self.serial.write(frame)
        time.sleep(0.02)
        rsp = self.serial.read(5 + 2 * cnt)
        if len(rsp) < 5 or rsp[0] != self.slave_id or rsp[1] != 0x03:
            raise Exception("读寄存器异常")
        return [(rsp[3 + i * 2] << 8) | rsp[4 + i * 2] for i in range(cnt)]

    # ---------- 业务 ----------
    def enable(self):
        self.write_register(self.enable_addr, 0x0001)
        time.sleep(0.2)

    def emergency_stop(self):
        """急停：写 0x6002=0x0008（厂家定义）"""
        try:
            self.write_register(self.trigger, 0x040)
            time.sleep(0.1)
            self.enable()          # 清除报警
        except Exception:
            pass

    def move_init(self):
        self.write_register(self.pr0_mode, 0x0001)
        self.write_register(self.speed, 0x0058)
        self.write_register(self.acc, 0x0032)
        self.write_register(self.dec, 0x0032)
        time.sleep(0.2)

    def split_32bit(self, v):
        return (v >> 16) & 0xFFFF, v & 0xFFFF

    def move_to(self, pos):
        high, low = self.split_32bit(pos)
        self.write_register(self.pr0_mode, 0x0001)
        self.write_register(self.pos_high, high)
        self.write_register(self.pos_low, low)
        self.write_register(self.trigger, 0x0010)

    def is_reached(self):
        try:
            return self.read_holding_registers(self.trigger, 1)[0] == 0
        except Exception:
            return None

    def to_zero(self):
        self.write_register(0x600a, 0x000c)
        self.write_register(0x600f, 0x0064)
        self.write_register(self.trigger, 0x0020)
        while not self.is_reached():
            time.sleep(0.2)

    def close(self):
        if self.serial and self.serial.is_open:
            self.serial.close()


# ---------- GUI ----------
class MotorGUI:
    def __init__(self, root):
        self.root = root
        root.title("Motor Control")
        root.geometry("450x400")
        self.motor = None

        # 输入
        tk.Label(root, text="Target Position:", font=("Arial", 12)).pack(pady=5)
        self.entry_pos = tk.Entry(root, font=("Arial", 14), justify="center")
        self.entry_pos.pack(pady=5)

        # 按钮区
        self.btn_connect = tk.Button(root, text="Enable", command=self.connect_motor, font=("Arial", 12))
        self.btn_connect.pack(pady=5)

        self.btn_move = tk.Button(root, text="Start Move", command=self.move_motor,
                                  state=tk.DISABLED, font=("Arial", 12))
        self.btn_move.pack(pady=5)

        # 固定位置 + 回零
        frame = tk.Frame(root)
        frame.pack(pady=10)

        self.btn_home = tk.Button(frame, text="Home", command=self.move_home,
                                  state=tk.DISABLED, width=8)
        self.btn_home.grid(row=0, column=0, padx=5)

        for idx, (name, pos) in enumerate([("100k", 100000), ("170k", 170000),
                                           ("200k", 200000), ("300k", 300000), ("318k", 318000)], 1):
            btn = tk.Button(frame, text=name, width=8,
                            command=lambda p=pos: self.move_fixed(p),
                            state=tk.DISABLED)
            btn.grid(row=(idx-1)//3, column=(idx-1)%3+1, padx=5)
            setattr(self, f"btn_pos{idx}", btn)

        # 急停
        self.btn_estop = tk.Button(root, text="EMERGENCY STOP", bg="red", fg="white",
                                   font=("Arial", 14, "bold"), command=self.emergency_stop)
        self.btn_estop.pack(pady=10)

        # 状态
        self.status_label = tk.Label(root, text="Status: disconnected", fg="red", font=("Arial", 12))
        self.status_label.pack(pady=10)

        # 退出
        self.btn_exit = tk.Button(root, text="Exit", command=self.on_exit, font=("Arial", 12))
        self.btn_exit.pack(pady=5)

    # ---------- 业务 ----------
    def connect_motor(self):
        try:
            self.motor = ModbusMotor()
            self.update_status("connected", "green")
            self._disable_buttons()
            threading.Thread(target=self._auto_home, daemon=True).start()
        except Exception as e:
            messagebox.showerror("Connect fail", str(e))

    def _auto_home(self):
        try:
            self.update_status("homing...", "blue")
            self.motor.to_zero()
            self.update_status("home done", "green")
        except Exception as e:
            self.update_status(f"error: {e}", "red")
        finally:
            self._enable_buttons()

    def move_motor(self):
        if not self.motor:
            return messagebox.showwarning("Warn", "Not enabled")
        try:
            pos = int(self.entry_pos.get())
        except ValueError:
            return messagebox.showwarning("Warn", "Invalid position")
        self._start_move(pos)

    def move_fixed(self, pos):
        if not self.motor:
            return messagebox.showwarning("Warn", "Not enabled")
        self._start_move(pos)

    def move_home(self):
        if not self.motor:
            return messagebox.showwarning("Warn", "Not enabled")
        self._start_move_home()

    def _start_move_home(self):
        self.update_status("homing...", "blue")
        self._disable_buttons()
        threading.Thread(target=self._run_home, daemon=True).start()

    def _run_home(self):
        try:
            self.motor.to_zero()
            self.update_status("home done", "green")
        except Exception as e:
            self.update_status(f"error: {e}", "red")
        finally:
            self._enable_buttons()

    def _start_move(self, pos):
        self.update_status("moving...", "blue")
        self._disable_buttons()
        threading.Thread(target=self._run_motion, args=(pos,), daemon=True).start()

    def _run_motion(self, pos):
        try:
            self.motor.move_to(pos)
            while True:
                st = self.motor.is_reached()
                if st is None:
                    self.update_status("comm error", "red")
                    break
                if st:
                    self.update_status("in position", "green")
                    break
                time.sleep(0.3)
        except Exception as e:
            self.update_status(f"error: {e}", "red")
        finally:
            self._enable_buttons()

    def emergency_stop(self):
        if not self.motor:
            return
        try:
            self.motor.emergency_stop()
            self.update_status("ESTOP !", "red")
        except Exception as e:
            self.update_status(f"estop fail: {e}", "red")

    # ---------- 按钮同步 ----------
    def _disable_buttons(self):
        for b in [self.btn_move, self.btn_home, self.btn_pos1, self.btn_pos2,
                  self.btn_pos3, self.btn_pos4, self.btn_pos5]:
            b.config(state=tk.DISABLED)

    def _enable_buttons(self):
        for b in [self.btn_move, self.btn_home, self.btn_pos1, self.btn_pos2,
                  self.btn_pos3, self.btn_pos4, self.btn_pos5]:
            b.config(state=tk.NORMAL)

    def update_status(self, txt, color):
        self.status_label.config(text=f"Status: {txt}", fg=color)

    def on_exit(self):
        if self.motor:
            self.motor.close()
        self.root.destroy()


# ---------- main ----------
if __name__ == "__main__":
    root = tk.Tk()
    MotorGUI(root)
    root.mainloop()
