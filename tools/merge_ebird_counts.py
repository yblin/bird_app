# -*- coding: utf-8 -*-
# Merge eBird obsstats counts into bird_difficulty.csv for the count-0 birds.
import csv, json

CSV = r'F:\bird_app\bird_difficulty.csv'
COUNTS = r'F:\bird_app\_ebird_counts.json'

# 1) build sciName -> ebird count from bulk fetch
sci2cnt = {}
for code, sci, n in json.load(open(COUNTS, encoding='utf-8')):
    if isinstance(n, int):
        sci2cnt[sci] = n

# 2) 7 unmapped species (manually resolved codes / parent-species counts)
extra = {
    'Grus carunculata': 5344,               # watcra2 Wattled Crane
    'Campephilus splendens': 2828,          # crbwoo1 Crimson-bellied (parent)
    'Parvipsitta porphyrocephala': 30625,   # puclor1
    'Parvipsitta pusilla': 41412,           # litlor1
    'Silvicultrix spodionota': 4242,        # crocht1 Crowned Chat-Tyrant (parent)
    'Sakesphorus pulchellus': 35384,        # blcant4 Black-crested Antshrike (parent)
    'Sylviorthorhynchus desmurii': 17274,  # demwir1
}
sci2cnt.update(extra)

rows = list(csv.reader(open(CSV, encoding='utf-8-sig')))
header, body = rows[0], rows[1:]
updated = 0
still_zero = 0
unmatched = []
for r in body:
    sci = r[2].strip()
    v = r[3].strip()
    if v == '0':
        if sci in sci2cnt:
            r[3] = str(sci2cnt[sci])
            updated += 1
            if sci2cnt[sci] == 0:
                still_zero += 1
        else:
            unmatched.append(sci)

with open(CSV, 'w', encoding='utf-8-sig', newline='') as f:
    w = csv.writer(f)
    w.writerow(header)
    w.writerows(body)

print('rows', len(body))
print('zero rows updated', updated)
print('  of which remain 0 (genuinely none on eBird)', still_zero)
print('count-0 sciNames WITHOUT ebird count (should be 0):', len(unmatched))
for u in unmatched:
    print('  UNMATCHED', u)
