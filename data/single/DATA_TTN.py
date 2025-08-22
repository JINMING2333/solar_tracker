import requests, pandas as pd, json, datetime as dt

# ---------- 配置 ----------
TENANT   = "eu1"
APP_ID   = "microsolar"
DEV_ID   = "single"
TOKEN    = "NNSXS.3ECGGBEKOUIGU4EOB7NXXODLANGEHUXFDN4PK7I.6Q4TITULSLPXKZXEVHAEQB4ZP2WZ3N2MXXLOGD7GMJ5AQNCKERNQ"

HOURS    = 48            # 抓取最近 N 小时
# --------------------------------

url = f"https://{TENANT}.cloud.thethings.network/api/v3/as/applications/{APP_ID}/devices/{DEV_ID}/packages/storage/uplink_message"
hdr = {"Authorization": f"Bearer {TOKEN}",
       "Accept": "application/json"}        # ← 保持或换成 text/event-stream 都行

resp = requests.get(url, headers=hdr, params={"last": f"{HOURS}h"}, timeout=60)
resp.raise_for_status()

rows = []
for raw in resp.text.splitlines():
    raw = raw.strip()
    if not raw:
        continue
    js  = raw.split("data:",1)[1].strip() if raw.startswith("data:") else raw
    try:
        msg = json.loads(js).get("result", {})
    except json.JSONDecodeError:
        continue

    uplink = msg["uplink_message"]
    pl     = uplink["decoded_payload"]

    rows.append({
        "time": pd.to_datetime(msg["received_at"]),
        "fcnt": uplink.get("f_cnt", 0),

        "angle_current_deg"     : pl.get("angle_current_deg"),
        "angle_target_deg"      : pl.get("angle_target_deg"),
        "ldr_east_raw"          : pl.get("ldr_east_raw"),
        "ldr_west_raw"          : pl.get("ldr_west_raw"),
        "wake_time_s"           : pl.get("wake_time_s"),

        "energy_logic_mWh"      : pl.get("energy_logic_mWh"),
        "energy_motor_idle_mWh" : pl.get("energy_motor_idle_mWh"),
        "energy_motor_run_mWh"  : pl.get("energy_motor_run_mWh"),
        "energy_solar_mWh"      : pl.get("energy_solar_mWh"),

        "V_logic_V"             : pl.get("V_logic_V"),
        "I_logic_mA"            : pl.get("I_logic_mA"),
        "V_motor_idle_V"        : pl.get("V_motor_idle_V"),
        "I_motor_idle_mA"       : pl.get("I_motor_idle_mA"),
        "V_motor_run_V"         : pl.get("V_motor_run_V"),
        "I_motor_run_mA"        : pl.get("I_motor_run_mA"),
        "V_solar_V"             : pl.get("V_solar_V"),
        "I_solar_mA"            : pl.get("I_solar_mA"),
    })


if not rows:
    raise SystemExit("❌  解码后仍为空，确认字段名与设备 ID")

df = (pd.DataFrame(rows)
        .assign(time=lambda d: pd.to_datetime(d["time"]))
        .sort_values("time")
        .reset_index(drop=True))

out = f"{DEV_ID}_last{HOURS}h_0816-17.csv"
df.to_csv(out, index=False)
print(f"✔ {len(df)} rows → {out}")
print(df.head())