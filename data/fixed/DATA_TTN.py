import requests, pandas as pd, json, datetime as dt

# ---------- 配置 ----------
TENANT   = "eu1"
APP_ID   = "microsolar"
DEV_ID   = "fixed2"
TOKEN    = "NNSXS.O75FV7W623KO4JWSGWX5DTFRDS7IWRI5NKHD7EI.LGGAAGWB3BZ4CWQJRSVR6JORAAWTN6ZEW5Q64EMTYJKHMPNXLFTA"

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

    uplink = msg.get("uplink_message", {})
    pl     = uplink.get("decoded_payload", {})

    rows.append({
        "time"              : pd.to_datetime(msg["received_at"]),
        "fcnt"              : uplink.get("f_cnt", 0),

        "LDR_east"          : pl.get("LDR_east"),
        "LDR_west"          : pl.get("LDR_west"),
        "wake_duration_s"   : pl.get("wake_duration_s"),

        "solar_energy_mWh"  : pl.get("solar_energy_mWh"),
        "solar_voltage_V"   : pl.get("solar_voltage_V"),
        "solar_current_mA"  : pl.get("solar_current_mA"),

        "logic_energy_mWh"  : pl.get("logic_energy_mWh"),
        "logic_voltage_V"   : pl.get("logic_voltage_V"),
        "logic_current_mA"  : pl.get("logic_current_mA"),
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