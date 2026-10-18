# -*- coding: utf-8 -*-
"""Query eBird record counts by scientific name and write bird_difficulty.csv.

WHY GBIF:
  - The eBird species HTML pages (which show a per-species record count) are
    protected by the Anubis anti-bot wall, so they cannot be scraped reliably.
  - The official eBird API requires a personal key AND does not expose a
    global per-species record count (it only serves recent/summarised data).
  - Cornell Lab publishes its full eBird data to GBIF as the
    "EOD - eBird Observation Dataset" (datasetKey 4fa7b334-...d049e, ~1.77
    billion records). GBIF's public occurrence API is keyless and lets us
    count how many eBird records exist for a given scientific name. That is a
    faithful programmatic proxy for "how many times this bird has been
    recorded on eBird".

COUNTING METHOD (important):
  The project's birds.csv uses a RECENT eBird taxonomy, whose scientific
  names are often newer than the GBIF backbone (e.g. "Anarhynchus mongolus"
  vs GBIF's older "Charadrius mongolus"). Searching GBIF by the backbone
  name therefore misses those species. GBIF preserves the ORIGINAL name that
  eBird submitted in a "verbatimScientificName" field, so we count records by
  verbatimScientificName instead. A species whose verbatim count is 0 is
  distinguished from a truly unknown name using the official eBird taxonomy
  (fetched once, keyless): present in eBird taxonomy -> 0 records; absent ->
  not found (-1).

Semantics of the output:
  - count >= 0 : number of eBird records for this species (found).
  - count == -1: could NOT be found / resolved (unmatched name or query error).

Usage:
    python query_ebird_difficulty.py [--input birds.csv] [--output bird_difficulty.csv]
             [--workers 8] [--delay 0.0] [--no-resume]

Only stdlib is used (urllib, json, concurrent.futures), matching the style of
tools/download_ebird_audio.py. The script resumes from an existing output file
so interrupted runs can be continued.
"""

import argparse
import concurrent.futures
import csv
import json
import os
import random
import sys
import time
import urllib.parse
import urllib.request

# Cornell Lab's eBird Observation Dataset published on GBIF.
EBIRD_DATASET_KEY = "4fa7b334-ce0d-4e88-aaae-2e0c138d049e"

OCCURRENCE_URL = "https://api.gbif.org/v1/occurrence/search"
EBIRD_TAXONOMY_URL = "https://api.ebird.org/v2/ref/taxonomy/ebird?fmt=json"

HEADERS = {"User-Agent": "bird_app/1.0 (birding data tool; contact: local)"}

MAX_RETRIES = 4          # per-request retries on 429 / 5xx / network errors
BASE_BACKOFF = 1.5       # seconds, exponential

# eBird scientific names, loaded lazily (one keyless API call) to decide
# between "real species with 0 eBird records" and "name not found".
_EBIRD_SCI_NAMES = None


def load_ebird_taxonomy():
    """Populate the set of scientific names recognised by the official eBird taxonomy."""
    global _EBIRD_SCI_NAMES
    if _EBIRD_SCI_NAMES is not None:
        return
    data = http_get_json(EBIRD_TAXONOMY_URL)
    _EBIRD_SCI_NAMES = {t.get("sciName", "").strip() for t in data} if data else set()
    print("Loaded eBird taxonomy ({} scientific names).".format(len(_EBIRD_SCI_NAMES)), flush=True)


def http_get_json(url):
    """GET a JSON endpoint with retries and exponential backoff."""
    last_err = None
    for attempt in range(MAX_RETRIES):
        try:
            req = urllib.request.Request(url, headers=HEADERS)
            with urllib.request.urlopen(req, timeout=60) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except urllib.error.HTTPError as e:
            last_err = e
            if e.code in (429, 500, 502, 503, 504):
                time.sleep(BASE_BACKOFF * (2 ** attempt) + random.uniform(0, 1.0))
                continue
            return None  # other HTTP errors (404 etc.) are treated as not found
        except Exception as e:  # URLError, timeout, json decode...
            last_err = e
            time.sleep(BASE_BACKOFF * (2 ** attempt) + random.uniform(0, 1.0))
    return None


def ebird_record_count(scientific_name):
    """Return the number of eBird records for a scientific name, or -1.

    Logic:
      1. Count occurrences by the ORIGINAL name eBird submitted
         (verbatimScientificName), which handles names newer than the GBIF
         backbone (the birds.csv taxonomy is a recent eBird taxonomy).
      2. If the count is 0, decide "found species with no records" (-> 0)
         versus "name not found" (-> -1) by membership in the official eBird
         taxonomy.
    """
    sci = scientific_name.strip()
    if not sci:
        return -1

    params = urllib.parse.urlencode({
        "datasetKey": EBIRD_DATASET_KEY,
        "verbatimScientificName": sci,
        "limit": 0,
    })
    data = http_get_json(OCCURRENCE_URL + "?" + params)
    if data is None:
        return -1  # query failed -> unknown, mark as not found

    count = int(data.get("count", 0))
    if count > 0:
        return count

    load_ebird_taxonomy()
    if sci in _EBIRD_SCI_NAMES:
        return 0
    return -1


def load_birds(input_path):
    """Parse birds.csv -> list of (chinese, english, scientific)."""
    birds = []
    with open(input_path, "r", encoding="utf-8", newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            # columns: 中文名, 英文名, 学名, 描述(可有逗号，用 maxsplit 前3列)
            parts = [c.strip() for c in (row[0:3] if len(row) >= 3 else row)]
            if len(parts) < 3:
                continue
            birds.append((parts[0], parts[1], parts[2]))
    return birds


def load_existing(output_path):
    """Return {scientific_name: record_count} already present in output."""
    result = {}
    if not os.path.exists(output_path):
        return result
    with open(output_path, "r", encoding="utf-8", newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            return result
        for row in reader:
            if len(row) >= 4:
                sci = row[2].strip()
                try:
                    result[sci] = int(row[3])
                except ValueError:
                    result[sci] = -1
    return result


def main():
    parser = argparse.ArgumentParser(description="eBird record counts by scientific name.")
    parser.add_argument("--input", default="birds.csv", help="input csv (default: birds.csv)")
    parser.add_argument("--output", default="bird_difficulty.csv",
                        help="output csv (default: bird_difficulty.csv)")
    parser.add_argument("--workers", type=int, default=8,
                        help="number of concurrent queries (default: 8)")
    parser.add_argument("--delay", type=float, default=0.0,
                        help="extra sleep between requests (default: 0)")
    parser.add_argument("--no-resume", action="store_true",
                        help="ignore an existing output file and re-query everything")
    args = parser.parse_args()

    # Resolve relative paths. Try, in order: the script's own folder, its
    # parent (the project root, where birds.csv usually lives), then cwd.
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [script_dir, os.path.dirname(script_dir), os.getcwd()]

    def resolve(rel):
        if os.path.isabs(rel):
            return rel
        for base in candidates:
            p = os.path.join(base, rel)
            if os.path.exists(p):
                return p
        return os.path.join(script_dir, rel)

    args.input = resolve(args.input)
    # Output lands next to birds.csv in the project root (the app's data dir).
    args.output = os.path.join(os.path.dirname(script_dir), args.output) \
        if not os.path.isabs(args.output) else args.output

    birds = load_birds(args.input)
    total = len(birds)
    print("Loaded {} birds from {}".format(total, args.input))

    existing = {} if args.no_resume else load_existing(args.output)
    resolved = {k: v for k, v in existing.items() if v != -1}
    print("Already have {} species resolved (resume); re-querying {} as -1.".format(
        len(resolved), len(existing) - len(resolved)))

    # to_query: birds not yet resolved (either absent or currently -1)
    to_query = []
    for idx, (cn, en, sci) in enumerate(birds):
        if sci in resolved:
            continue
        to_query.append((idx, cn, en, sci))

    # new_results: {sci: count}
    new_results = dict(resolved)
    done_count = len(resolved)

    def query_one(item):
        idx, cn, en, sci = item
        count = ebird_record_count(sci)
        return sci, count

    with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = {pool.submit(query_one, it): it for it in to_query}
        for future in concurrent.futures.as_completed(futures):
            it = futures[future]
            idx, cn, en, sci = it
            try:
                _, count = future.result()
            except Exception:
                count = -1
            new_results[sci] = count
            done_count += 1
            if done_count % 50 == 0 or done_count == total:
                print("  {}/{} done".format(done_count, total), flush=True)
                _write_output(args.output, birds, new_results)
            if args.delay > 0:
                time.sleep(args.delay)

    _write_output(args.output, birds, new_results)
    not_found = sum(1 for v in new_results.values() if v == -1)
    print("Finished. Wrote {} species to {} (not found / -1: {})".format(
        len(new_results), args.output, not_found))


def _write_output(output_path, birds, counts):
    """Write bird_difficulty.csv: 中文名,英文名,学名,记录次数 (-1 = not found)."""
    with open(output_path, "w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["中文名", "英文名", "学名", "记录次数"])
        for cn, en, sci in birds:
            writer.writerow([cn, en, sci, counts.get(sci, -1)])


if __name__ == "__main__":
    sys.exit(main())
