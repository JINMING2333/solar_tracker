import requests, pandas as pd, json, datetime as dt

# ---------- 配置 ----------
TENANT   = "eu1"
APP_ID   = "microsolar"
DEV_ID   = "fixed"
TOKEN    = "NNSXS.O75FV7W623KO4JWSGWX5DTFRDS7IWRI5NKHD7EI.LGGAAGWB3BZ4CWQJRSVR6JORAAWTN6ZEW5Q64EMTYJKHMPNXLFTA"

HOURS    = 12            # 抓取最近 N 小时
# --------------------------------

# ---- URL ----
url = (f"https://{TENANT}.cloud.thethings.network/api/v3/as/applications/"
       f"{APP_ID}/devices/{DEV_ID}/packages/storage/uplink_message")

params = { "last": f"{HOURS}h" }
headers = {
    "Authorization": f"Bearer {TOKEN}",
    "Accept"       : "application/json"
}

print("⌛ request …")
resp = requests.get(url, params=params, headers=headers, timeout=60)
resp.raise_for_status()

rows = []
for line in resp.text.strip().splitlines():
    m  = json.loads(line)["result"]
    pl = m["uplink_message"]["decoded_payload"]

    # —— 必须包含所有新字段，否则跳过 ——
    req_keys = {"Ppv_mW","Psys_mW","Vpv_V","Vsys_V","Ipv_mA","Isys_mA","idx"}
    if not req_keys.issubset(pl):
        continue

    rows.append({
        "time"     : pd.to_datetime(m["received_at"]),
        "Ppv_mW"   : pl["Ppv_mW"],
        "Psys_mW"  : pl["Psys_mW"],
        "Vpv_V"    : pl["Vpv_V"],
        "Vsys_V"   : pl["Vsys_V"],
        "Ipv_mA"   : pl["Ipv_mA"],
        "Isys_mA"  : pl["Isys_mA"],
        "idx"      : pl["idx"]
    })

df = pd.DataFrame(rows).sort_values("time").reset_index(drop=True)
print(df.head(), "…", len(df), "rows")

csv_name = f"{DEV_ID}_last{HOURS}h.csv"
df.to_csv(csv_name, index=False)
print("CSV written →", csv_name)
