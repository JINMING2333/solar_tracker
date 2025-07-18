import pandas as pd
import matplotlib.pyplot as plt

# 1) 读取刚才 Arduino 采集到的文件
#    —— 把文件名改成你自己的
df = pd.read_csv('fixed_last12h_01.csv')

# 2) 计算逐分钟能量（Wh）
df['energy_mWh'] = df['P_avg_mW'] * (1/60)   # mW → W，再乘 1/60 h
total_energy = df['energy_mWh'].sum()

# 3) 如果只想看最近 15 min，可 tail(15)
df = df.tail(15)

# 4) 时间戳转为可读 datetime；如果 t_s 是从零开始，可给定起始日期
df['time'] = pd.to_datetime(df['t_s'], unit='s')

# 5) 画图
plt.figure(figsize=(6,4))
plt.plot(df['time'], df['P_avg_mW'])              # W
plt.xlabel('Time')
plt.ylabel('Average Power (mW)')
plt.title(f'15-minute Power Curve\nTotal Energy = {total_energy:.3f} mWh')
plt.xticks(rotation=45)
plt.grid(True)
plt.tight_layout()
plt.show()
