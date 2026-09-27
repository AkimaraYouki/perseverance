"""pv robust 의 jump 결과 json 에서 착지 순간 상태와 성공 여부를 요약 (wbctrl 이 jumps 에 남긴 값).

    python3 jump_why.py ~/pv_out/robust/<stamp>_jump.json [...]
"""
import json
import statistics as st
import sys

K = ("pitch_takeoff", "pitch_min_air", "pitch_land", "v_land", "wheel_v_land")
for f in sys.argv[1:]:
    r = json.load(open(f)); R = r["rows"]; t = r["tune"]
    groups = {"성공": [], "뒤로 떨어짐": [], "앞으로 지나침": [], "넘어짐": []}
    for x in R:
        j = (x.get("jumps") or [{}])[0]
        g = "성공" if x["ok"] else ("넘어짐" if x["fell_t"] is not None else ("앞으로 지나침" if x["xmax"] >= 1.9 else "뒤로 떨어짐"))
        groups[g].append(dict(j, x_land_cm=100 * (j.get("x_land", 1.0) - 1.0)))
    print(f"{f.split('/')[-1]}: 공중 {t.get('air_pitch')} 착지목표 {t.get('land_pitch', t.get('air_pitch'))} kd {t.get('air_kd')} | 통과 {r['npass']}/{r['n']}")
    for g, rows in groups.items():
        if not rows:
            continue
        med = {k: (st.median([v[k] for v in rows if v.get(k) is not None]) if any(v.get(k) is not None for v in rows) else None) for k in K + ("x_land_cm",)}
        print(f"   {g:8s} {len(rows):2d} 대 | " + "  ".join(f"{k} {med[k]:+.2f}" if med[k] is not None else f"{k} -" for k in K + ("x_land_cm",)))
