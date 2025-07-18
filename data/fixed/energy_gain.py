#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Log PV data from Arduino (INA219) to CSV.

CSV columns: t_s, V_avg, I_avg_mA, P_avg_mW, P_min_mW, P_max_mW
"""

import serial
import csv
import time
from datetime import datetime

# -------- 可按需修改 --------
PORT      = 'COM19'     # Windows 端口, Linux/Mac 改成 /dev/ttyACM0
BAUD      = 115200
CSV_FILE  = 'test_1min_01.csv'
# --------------------------------

def wait_numeric_line(ser):
    """丢弃非数据行，直到读到首个以数字开头的有效数据行"""
    while True:
        line = ser.readline().decode(errors='ignore').strip()
        if line and line[0].isdigit():
            return line

def main():
    print(f'Opening {PORT} @ {BAUD} …')
    with serial.Serial(PORT, BAUD, timeout=2) as ser:
        # SAMD 板复位需要 1~2 s
        time.sleep(2)

        # 发送 START 指令
        ser.write(b'START\n')
        ser.flush()

        # 读到第一行数据，确保 Arduino 已经进入采样
        first = wait_numeric_line(ser)
        print('First frame:', first)

        with open(CSV_FILE, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(
                ['t_s', 'V_avg', 'I_avg_mA', 'P_avg_mW', 'P_min_mW', 'P_max_mW']
            )
            writer.writerow(first.split(','))

            print(f'Logging → {CSV_FILE} (Ctrl-C to stop)')
            try:
                while True:
                    line = ser.readline().decode(errors='ignore').strip()
                    if not line:
                        continue
                    writer.writerow(line.split(','))
                    print(line)
            except KeyboardInterrupt:
                print('\nUser interrupt — log closed.')

if __name__ == '__main__':
    main()