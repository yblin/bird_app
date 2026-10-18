import csv
from collections import Counter

rows = list(csv.DictReader(open("F:/bird_app/bird_difficulty.csv", encoding="utf-8")))
cnts = [int(r["记录次数"]) for r in rows]
valid = [c for c in cnts if c >= 0]
print("rows", len(rows), "valid", len(valid), "-1", len(cnts) - len(valid))

s = sorted(valid)
thresholds = []
rem = s
for step in range(5):
    m = rem[len(rem) // 2]
    thresholds.append(m)
    rem = [v for v in rem if v >= m]
print("thresholds(medians)", thresholds)


def tier(c):
    for i, t in enumerate(thresholds):
        if c < t:
            return i
    return 5


dist = Counter(tier(c) for c in valid)
labels = ["菜", "易", "中", "难", "很难", "极难"]
print("tier distribution:", [dist.get(i, 0) for i in range(6)])
for c in [0, 5, 50, 500, 1000, 5000, 10000, 57599, 653537]:
    print(c, "->", labels[tier(c)])
for r in rows:
    if "红胸田鸡" in r["中文名"]:
        print("红胸田鸡:", r["中文名"], r["学名"], r["记录次数"],
              "->", labels[tier(int(r["记录次数"]))])
