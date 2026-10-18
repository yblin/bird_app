# -*- coding: utf-8 -*-
"""Count eBird record numbers from the eBird Basic Dataset (EBD) and write
bird_difficulty.csv.

WHY EBD:
  The only public data source whose per-species observation counts match the
  number shown on eBird species pages ("Statistics > 观察记录") is the eBird
  Basic Dataset itself. The species pages are behind the Anubis anti-bot wall
  and the official eBird API (key) does not expose a global per-species count,
  so we count rows in the EBD by SCIENTIFIC_NAME locally.

Input:
  One or more EBD files (tab-delimited .txt), or a directory / glob. The EBD
  is often delivered split into many files (by year/region). Supply as many as
  you downloaded. Header must contain a "SCIENTIFIC_NAME" column.

Counting semantics:
  Each EBD row is one species-per-checklist observation, so counting rows per
  SCIENTIFIC_NAME equals the "observation records" number eBird shows on the
  species page. Only the species listed in birds.csv are counted (memory
  efficient on the huge EBD files). A species not present anywhere in the EBD
  is written as -1 (not found).

Usage:
    python count_ebd_difficulty.py "F:\\bird_app\\ebd\\*.txt" [--input birds.csv]
        [--output bird_difficulty.csv] [--encoding utf-8]

Note: set --encoding to the EBD file encoding (EBD downloads are normally
UTF-8; older regional releases may be Latin-1 / windows-1252).
"""

import argparse
import csv
import glob
import io
import os
import sys


def load_birds(input_path):
    """Parse birds.csv -> list of (chinese, english, scientific)."""
    birds = []
    with open(input_path, "r", encoding="utf-8", newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            parts = [c.strip() for c in (row[0:3] if len(row) >= 3 else row)]
            if len(parts) < 3:
                continue
            birds.append((parts[0], parts[1], parts[2]))
    return birds


def find_sci_col(header):
    """Index of the SCIENTIFIC_NAME column (last occurrence if duplicated)."""
    idx = None
    for i, h in enumerate(header):
        if h.strip().upper() == "SCIENTIFIC_NAME":
            idx = i
    if idx is None:
        raise RuntimeError("No SCIENTIFIC_NAME column found in EBD header: "
                           + repr(header[:12]))
    return idx


def count_ebd(files, target_names, encoding="utf-8"):
    """Stream-count occurrences of target scientific names across EBD files."""
    counts = {s: 0 for s in target_names}
    errors = []
    for path in files:
        got_header = False
        sci_col = None
        n = 0
        with io.open(path, "r", encoding=encoding, errors="replace", newline="") as f:
            for line in f:
                if not line.strip():
                    continue
                if not got_header:
                    sci_col = find_sci_col(line.rstrip("\n").split("\t"))
                    got_header = True
                    continue
                # Most rows have a species; check the sci name column membership
                # cheaply before any split (rare names are not worth the cost).
                fields = line.rstrip("\n").split("\t")
                if len(fields) <= sci_col:
                    continue
                name = fields[sci_col].strip()
                if name in counts:
                    counts[name] += 1
                n += 1
        print("  counted {} rows from {}".format(n, path), flush=True)
    return counts


def resolve_ebd_files(paths):
    """Expand each arg into concrete file paths (files, dirs, or globs)."""
    out = []
    for p in paths:
        if os.path.isdir(p):
            out.extend(sorted(glob.glob(os.path.join(p, "*.txt"))))
        else:
            out.extend(sorted(glob.glob(p)))
    # de-duplicate while preserving order
    seen = set()
    uniq = []
    for p in out:
        ap = os.path.abspath(p)
        if ap not in seen:
            seen.add(ap)
            uniq.append(ap)
    return uniq


def main():
    parser = argparse.ArgumentParser(
        description="Count eBird record numbers from the eBird Basic Dataset.")
    parser.add_argument("ebd", nargs="+",
                        help="EBD file(s), a directory, or a glob (tab-delimited txt)")
    parser.add_argument("--input", default="birds.csv", help="input csv (default: birds.csv)")
    parser.add_argument("--output", default="bird_difficulty.csv",
                        help="output csv (default: bird_difficulty.csv)")
    parser.add_argument("--encoding", default="utf-8",
                        help="EBD file encoding (default: utf-8)")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [script_dir, os.path.dirname(script_dir), os.getcwd()]
    for rel, want in ((args.input, False), (args.output, False)):
        if not os.path.isabs(rel):
            for base in candidates:
                p = os.path.join(base, rel)
                if os.path.exists(p):
                    if want:
                        args.output = p
                    else:
                        args.input = p
                    break
    if not os.path.isabs(args.output):
        args.output = os.path.join(os.path.dirname(script_dir), args.output)

    birds = load_birds(args.input)
    target_names = {sci for _, _, sci in birds}
    print("Loaded {} birds from {}".format(len(birds), args.input))

    files = resolve_ebd_files(args.ebd)
    if not files:
        print("No EBD files matched from args: {}".format(args.ebd))
        return 2
    print("Counting {} EBD file(s)...".format(len(files)))

    counts = count_ebd(files, target_names, encoding=args.encoding)

    not_found = 0
    with open(args.output, "w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["中文名", "英文名", "学名", "记录次数"])
        for cn, en, sci in birds:
            c = counts.get(sci, 0)
            if c == 0:
                c = -1
                not_found += 1
            writer.writerow([cn, en, sci, c])

    print("Finished. Wrote {} species to {} (not found / -1: {}).".format(
        len(birds), args.output, not_found))


if __name__ == "__main__":
    sys.exit(main())
