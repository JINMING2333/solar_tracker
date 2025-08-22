import pandas as pd
import matplotlib.pyplot as plt

fixed_df = pd.read_csv("fixed_last48h_0729.csv")
single_df = pd.read_csv("single_last24h_0729.csv")

# 清洗列名
fixed_df.columns = fixed_df.columns.str.strip()
single_df.columns = single_df.columns.str.strip()

# 时间转换
fixed_df["time"] = pd.to_datetime(fixed_df["time"])
single_df["time"] = pd.to_datetime(single_df["time"])

# 绘图
plt.figure(figsize=(12,6))
plt.plot(fixed_df["time"], fixed_df["solar_mWh"].cumsum(), label="Fixed Panel (solar_mWh)")
plt.plot(single_df["time"], single_df["E_solar_mWh"].cumsum(), label="Single-Axis Tracker (E_solar_mWh)")
plt.xlabel("Time")
plt.ylabel("Cumulative Solar Energy (mWh)")
plt.title("Cumulative Solar Energy Comparison on 2025-07-29")
plt.legend()
plt.grid(True)
plt.xticks(rotation=45)
plt.tight_layout()
plt.show()
