import pandas as pd
import matplotlib.pyplot as plt

# 读取数据
df = pd.read_csv("single_last24h_0729.csv")

# 转换时间（假设已有 time 字段为 ISO 格式）
df["time"] = pd.to_datetime(df["time"])

# 计算电机总能耗
df["E_motor_total_mWh"] = df["Ea_motor_mWh"] + df["Ei_motor_mWh"]

# 绘图
plt.figure(figsize=(12, 6))

plt.plot(df["time"], df["E_solar_mWh"], label="Solar Energy Absorbed (mWh)", color="orange")
plt.plot(df["time"], df["E_logic_mWh"], label="Logic Circuit Energy (mWh)", color="green")
plt.plot(df["time"], df["E_motor_total_mWh"], label="Motor Total Energy (mWh)", color="purple")

plt.title("Real-time Energy Flow Comparison (Single-axis System)")
plt.xlabel("Time")
plt.ylabel("Energy (mWh per 5 min)")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.xticks(rotation=45)
plt.show()
