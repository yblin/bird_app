# -*- coding: utf-8 -*-
# Query zero/-1 birds by English name: resolve exact taxonomy code, fetch obsstats, update CSV.
import csv, json, urllib.request, ssl, time, os

KEY='epgloncoi3vp'
ctx=ssl._create_unverified_context()
H={'X-eBirdApiToken':KEY,'User-Agent':'Mozilla/5.0'}
TAX=r'F:\bird_app\_ebird_taxonomy_full.json'
CSV=r'F:\bird_app\bird_difficulty.csv'
RES=r'F:\bird_app\_ebird_en_counts.json'

tax=json.load(open(TAX,encoding='utf-8'))

def resolve(E):
    el=E.lower()
    exact=[t for t in tax if t['comName'].lower()==el]
    if exact:
        sp=[t for t in exact if t['category']=='species']
        if sp: return sp[0]['speciesCode']
        for t in exact:
            base=' '.join(t['sciName'].split()[:2])
            for u in tax:
                if u['category']=='species' and u['sciName'].lower()==base.lower():
                    return u['speciesCode']
        return exact[0]['speciesCode']
    return None

def fetch(code, retries=3):
    url='https://api.ebird.org/v2/product/obsstats/'+code
    for a in range(retries):
        try:
            req=urllib.request.Request(url,headers=H)
            j=json.loads(urllib.request.urlopen(req,timeout=40,context=ctx).read().decode())
            return j.get('obsCount')
        except Exception:
            time.sleep(2*(a+1))
    return None

# load current CSV rows
rows=[r for r in csv.reader(open(CSV,encoding='utf-8-sig'))]
header=rows[0]; body=rows[1:]
target=[r for r in body if r[3].strip() in ('0','-1')]

# map english name -> row
# resolve and fetch
updates={}  # (cn,en) -> new count
saved={}
if os.path.exists(RES):
    saved=json.load(open(RES,encoding='utf-8'))

for r in target:
    en=r[1].strip()
    code=resolve(en)
    if not code:
        continue
    if en in saved:
        n=saved[en]
    else:
        n=fetch(code)
        saved[en]=n
        json.dump(saved,open(RES,'w',encoding='utf-8'),ensure_ascii=False)
        time.sleep(0.3)
    key=(r[0].strip(),en)
    updates[key]=('CODE' if code else 'NONE', code, n)

# apply updates
changed=0
for r in body:
    key=(r[0].strip(),r[1].strip())
    cur=r[3].strip()
    if key in updates:
        _,code,n=updates[key]
        if n is not None and isinstance(n,int):
            # for -1: set directly; for 0: only if nonzero
            if cur=='-1' or (cur=='0' and n!=0):
                if str(n)!=cur:
                    r[3]=str(n); changed+=1
        # note: for 0 rows where n==0, no change

with open(CSV,'w',encoding='utf-8-sig',newline='') as f:
    w=csv.writer(f); w.writerow(header); w.writerows(body)

print('target',len(target),'resolved+fetched',len([u for u in updates.values() if u[1]]))
print('rows changed',changed)
print('--- updates (code,count) ---')
for k,(how,code,n) in updates.items():
    if how=='CODE':
        print(k[1],'->',code,n)
