import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks

# ───────────────────── 参数区 ─────────────────────
CSV_PATH = "DC_encoder_100ms_01.csv"
STEP_DEGREE = 12
TOLERANCE = 1.0
PEAK_HEIGHT_THRESHOLD = 20  # mW 以上才认为是“有功率峰”
# ────────────────────────────────────────────────

# 1. 读取 CSV 数据
df = pd.read_csv(CSV_PATH, names=["ms", "deg", "V", "I", "P"], skiprows=1)
df["t_s"] = df["ms"] * 1e-3  # ms → s
df["P_W"] = df["P"] * 1e-3   # mW → W

# 2. 识别周期区间（第一次 >1° 到最后一次 <1°）
start_idx = df.index[df["deg"] > 1.0][0]
end_idx = df.index[::-1][df["deg"][::-1] < 1.0][0]
df_cycle = df.loc[start_idx:end_idx].reset_index(drop=True)

# 3. 寻找功率峰位置
peaks, _ = find_peaks(df_cycle["P"], height=PEAK_HEIGHT_THRESHOLD)

# 4. 对每个功率峰找前面的角度变化点（起始）
change_points = []
last_angle = df_cycle.at[0, "deg"]
for i in range(1, len(df_cycle)):
    current_angle = df_cycle.at[i, "deg"]
    if abs(current_angle - last_angle) >= TOLERANCE:
        change_points.append(i)
    last_angle = current_angle

# 5. 为每个峰配对角度点（向前找最近的变化点）
ticks_t = []
ticks_lbl = []
used_angles = set()
for p in peaks:
    # 找变化点中最靠近当前峰值前的位置
    candidates = [cp for cp in change_points if cp < p]
    if not candidates:
        continue
    idx = candidates[-1]
    t = df_cycle.at[idx, "t_s"]
    angle = df_cycle.at[idx, "deg"]
    rounded_angle = int(round(angle / STEP_DEGREE) * STEP_DEGREE) % 360
    # 防止重复标注
    if rounded_angle in used_angles:
        continue
    used_angles.add(rounded_angle)
    ticks_t.append(t)
    ticks_lbl.append(f"{rounded_angle}°")

# 6. 计算能量（单位：mWh）
from numpy import trapz
energy_mWh = trapz(df_cycle["P_W"], df_cycle["t_s"]) * (1000 / 3600)

# 7. 绘图
plt.figure(figsize=(12,5))
plt.plot(df_cycle["t_s"], df_cycle["P"], 'r-', linewidth=1, label="Power (mW)")

# 垂直线和角度文字
for t, lbl in zip(ticks_t, ticks_lbl):
    plt.axvline(t, color="blue", linestyle="--", linewidth=0.6)
    plt.text(t, plt.ylim()[1]*1.01, lbl, ha="center", va="bottom", fontsize=8)

plt.title(f"Power Consumption_50ms (Energy = {energy_mWh:.3f} mWh)", pad=25)
plt.xlabel("Time (s)")
plt.ylabel("Power (mW)")
plt.legend(loc='lower left', bbox_to_anchor=(1.01, 0.0), borderaxespad=0.)
plt.grid(True, linestyle="--", color='gray', alpha=0.3)

# 去边框
ax = plt.gca()
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

plt.tight_layout(rect=[0, 0, 0.85, 1])
plt.show()
