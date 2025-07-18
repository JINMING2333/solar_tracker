import serial
import csv
import matplotlib.pyplot as plt
from collections import deque
import threading
import time

PORT = 'COM6'          # 根据实际端口修改
BAUD = 115200
MAX_POINTS = 300        # 实时显示的点数
CSV_FILE = 'DC_encoder_100ms_01.csv'

# 数据缓冲区
t_buf = deque(maxlen=MAX_POINTS)
pow_buf = deque(maxlen=MAX_POINTS)
v_buf = deque(maxlen=MAX_POINTS)
i_buf = deque(maxlen=MAX_POINTS)
angle_buf = deque(maxlen=MAX_POINTS)
angle_change_points = []

lock = threading.Lock()
last_angle = None

def serial_reader():
    global last_angle
    ser = serial.Serial(PORT, BAUD, timeout=1)
    ser.write(b"START\n")
    csvfile = open(CSV_FILE, 'w', newline='')
    writer = csv.writer(csvfile)
    writer.writerow(['t_ms', 'angle_deg', 'v', 'I_mA', 'power_mW'])
    csvfile.flush()  # ⬅️ 添加这行！
    try:
        while True:
            line = ser.readline().decode('utf-8').strip()
            if line:
                parts = line.split(',')
                if len(parts) == 5:
                    t, angle, v, I, power = map(float, parts)
                    I = abs(I)
                    power = abs(power)
                    with lock:
                        t_buf.append(t)
                        pow_buf.append(power)
                        v_buf.append(v)
                        i_buf.append(I)
                        angle_buf.append(angle)
                        writer.writerow([t, angle, v, I, power])
                        # 检测角度变化
                        if last_angle is not None and angle != last_angle:
                            angle_change_points.append((t, angle))
                        last_angle = angle
    except Exception as e:
        print("串口采集异常：", e)
    finally:
        ser.close()
        csvfile.close()

def plotter():
    plt.ion()
    fig, ax = plt.subplots()
    line_pow, = ax.plot([], [], 'r-', label='Power (mW)')
    line_v,   = ax.plot([], [], 'g-', label='Voltage (V)')
    line_i,   = ax.plot([], [], 'b-', label='Current (mA)')
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Value')
    ax.legend(loc='upper right')
    plotted_angles = set()
    while True:
        with lock:
            line_pow.set_data(t_buf, pow_buf)
            line_v.set_data(t_buf, v_buf)
            line_i.set_data(t_buf, i_buf)
            ax.relim()
            ax.autoscale_view()
            # 画角度变化的竖线（只画一次）
            for t, angle in angle_change_points:
                if (t, angle) not in plotted_angles:
                    ax.axvline(x=t, color='k', linestyle='--', alpha=0.7)
                    ax.annotate(f'{angle:.1f}°', xy=(t, max(pow_buf) if pow_buf else 0), 
                                xytext=(t, (max(pow_buf) if pow_buf else 0) + 2),
                                arrowprops=dict(facecolor='black', shrink=0.05), fontsize=8, rotation=90)
                    plotted_angles.add((t, angle))
            angle_change_points.clear()
        plt.pause(0.2)  # 每0.2秒刷新一次

if __name__ == '__main__':
    t1 = threading.Thread(target=serial_reader, daemon=True)
    t2 = threading.Thread(target=plotter, daemon=True)
    t1.start()
    t2.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("程序已停止。")
