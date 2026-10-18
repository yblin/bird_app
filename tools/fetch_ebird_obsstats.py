# -*- coding: utf-8 -*-
import urllib.request, ssl, json, time, os, sys

KEY = 'epgloncoi3vp'
RESULT = r'F:\bird_app\_ebird_counts.json'
ctx = ssl._create_unverified_context()
H = {'X-eBirdApiToken': KEY, 'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64)'}

def fetch(code, retries=3):
    url = 'https://api.ebird.org/v2/product/obsstats/' + code
    for a in range(retries):
        try:
            req = urllib.request.Request(url, headers=H)
            b = urllib.request.urlopen(req, timeout=40, context=ctx).read()
            j = json.loads(b.decode('utf-8'))
            return j.get('obsCount')
        except Exception as e:
            # 429 / timeout -> wait and retry
            if a < retries - 1:
                time.sleep(3 * (a + 1))
            else:
                return ('ERR', str(e)[:80])
    return None

def main():
    codes = json.load(open(r'F:\bird_app\_zero_codes.json', encoding='utf-8'))
    items = list(codes.items())
    acc = []
    if os.path.exists(RESULT):
        acc = json.load(open(RESULT, encoding='utf-8'))
    done = {row[0] for row in acc}
    print('total', len(items), 'already', len(done), flush=True)
    for sci, c in items:
        if c in done:
            continue
        n = fetch(c)
        acc.append([c, sci, n])
        json.dump(acc, open(RESULT, 'w', encoding='utf-8'), ensure_ascii=False)
        print(c, n, flush=True)
        time.sleep(0.4)  # keep under rate limit
    print('DONE total', len(acc), flush=True)

if __name__ == '__main__':
    main()
