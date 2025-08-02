import requests, pandas as pd, json, datetime as dt

# ---------- 配置 ----------
TENANT   = "eu1"
APP_ID   = "microsolar"
DEV_ID   = "single"
TOKEN    = "NNSXS.O75FV7W623KO4JWSGWX5DTFRDS7IWRI5NKHD7EI.LGGAAGWB3BZ4CWQJRSVR6JORAAWTN6ZEW5Q64EMTYJKHMPNXLFTA"

HOURS    = 24            # 抓取最近 N 小时
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
        "time"          : pd.to_datetime(msg["received_at"]),
        "fcnt"          : uplink.get("f_cnt", 0),

        "curAngle_deg"  : pl.get("curAngle_deg"),
        "tgtAngle_deg"  : pl.get("tgtAngle_deg"),

        "Ea_motor_mWh"  : pl.get("Ea_motor_mWh"),
        "Ei_motor_mWh"  : pl.get("Ei_motor_mWh"),
        "E_logic_mWh"   : pl.get("E_logic_mWh"),
        "E_solar_mWh"   : pl.get("E_solar_mWh"),

        "LDR_E"         : pl.get("LDR_E"),
        "LDR_W"         : pl.get("LDR_W"),

        "Vm_motor"      : pl.get("Vm_motor"),
        "Im_motor"      : pl.get("Im_motor"),

        "Vm_logic"      : pl.get("Vm_logic"),
        "Im_logic"      : pl.get("Im_logic"),

        "Vm_solar"      : pl.get("Vm_solar"),
        "Im_solar"      : pl.get("Im_solar"),
    })


if not rows:
    raise SystemExit("❌  解码后仍为空，确认字段名与设备 ID")

df = (pd.DataFrame(rows)
        .assign(time=lambda d: pd.to_datetime(d["time"]))
        .sort_values("time")
        .reset_index(drop=True))

out = f"{DEV_ID}_last{HOURS}h_0730.csv"
df.to_csv(out, index=False)
print(f"✔ {len(df)} rows → {out}")
print(df.head())