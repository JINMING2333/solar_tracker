import pandas as pd
import matplotlib.pyplot as plt

# 1. 读取数据
df = pd.read_csv('DC_encoder_10ms_2.csv')

angles = df['angle_deg'].values
times = df['t_ms'].values

def is_close(a, b, tol=1):
    return abs(a - b) < tol

# 1. 找到第一个30度为周期起点
start_idx = None
for i in range(len(angles)):
    if is_close(angles[i], 0):
        start_idx = i
        break

# 2. 找到第一个150->30的跳变为周期终点
end_idx = None
for i in range(start_idx+1, len(angles)):
    if is_close(angles[i-1], 119) and is_close(angles[i], 0):
        end_idx = i
        break

if start_idx is None or end_idx is None:
    raise ValueError('未找到完整的30°→150°→30°周期，请检查数据或调整容差。')

# 3. 截取一个周期的数据
df_cycle = df.iloc[start_idx:end_idx+1].reset_index(drop=True)

# 4. 找到周期内所有角度变化点
change_points = [0]
for i in range(1, len(df_cycle)):
    if not is_close(df_cycle.loc[i, 'angle_deg'], df_cycle.loc[i-1, 'angle_deg']):
        change_points.append(i)

# 5. 计算能量消耗
power = df_cycle['power_mW'].values
t = df_cycle['t_ms'].values
dt = (t[1:] - t[:-1])  # ms
energy_mWh = (power[:-1] * dt).sum() / 3600000  # mWh

# 6. 绘制功率-时间曲线，并标记角度变化
plt.figure(figsize=(14,6))
plt.plot(df_cycle['t_ms']/1000, df_cycle['power_mW'], label='Power (mW)', color='r', linewidth=1)

ax = plt.gca()
ymax = df_cycle['power_mW'].max()
ymin = df_cycle['power_mW'].min()
y_range = ymax - ymin

# 统一角度标注高度（不超过顶部，略低于ymax）
y_annotate = ymax - 0.05 * y_range

for idx in change_points:
    t_sec = df_cycle.loc[idx, 't_ms'] / 1000
    # 角度数值位于虚线左侧，稍微偏移
    x_annotate = t_sec - 0.01 * (df_cycle['t_ms'].max()/1000 - df_cycle['t_ms'].min()/1000)
    plt.axvline(x=t_sec, color='blue', linestyle='--', alpha=0.7, linewidth=1)
    plt.annotate(f"{df_cycle.loc[idx, 'angle_deg']:.0f}°", 
                 xy=(x_annotate, y_annotate), 
                 xytext=(x_annotate, y_annotate), 
                 ha='right', va='bottom',
                 fontsize=10, color='black', rotation=0,
                 bbox=None)  # 无边框

plt.xlabel('Time (s)', fontsize=14)
plt.ylabel('Power (mW)', fontsize=14)
# 大标题与图像保持距离
plt.title(f'DC_encoder Power Consumption in One Cycle (Energy = {energy_mWh:.3f} mWh)', fontsize=18, pad=30)

# 图例放在图像外部右下角
plt.legend(fontsize=12, loc='lower left', bbox_to_anchor=(1.01, 0.0), borderaxespad=0.)

# 恢复背景横竖虚线
plt.grid(True, which='both', linestyle='--', color='gray', alpha=0.4)

# 去掉顶部和右侧边界线
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

plt.tight_layout(rect=[0, 0, 0.85, 1])  # 为右侧图例留出空间
plt.show()