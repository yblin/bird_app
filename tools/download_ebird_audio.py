"""Download audio from Macaulay Library by asset ID.

Usage:
    python download_audio.py 36443
    python download_audio.py 36443 --out ./audio
"""

import argparse
import os
import sys
import urllib.request

# Macaulay Library static media host. The audio is served directly from this
# CDN without the Anubis bot-check that protects the HTML pages.
AUDIO_URL = "https://cdn.download.ams.birds.cornell.edu/api/v1/asset/{}/audio"


def download_audio(asset_id, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    url = AUDIO_URL.format(asset_id)
    out_path = os.path.join(out_dir, "{}.mp3".format(asset_id))

    print("Downloading {} ...".format(url))
    request = urllib.request.Request(
        url, headers={"User-Agent": "Mozilla/5.0"})

    try:
        with urllib.request.urlopen(request, timeout=60) as resp, \
                open(out_path, "wb") as f:
            while True:
                chunk = resp.read(65536)
                if not chunk:
                    break
                f.write(chunk)
    except urllib.error.HTTPError as e:
        sys.exit("HTTP error {}: {}".format(e.code, e.reason))
    except urllib.error.URLError as e:
        sys.exit("Network error: {}".format(e.reason))

    size = os.path.getsize(out_path)
    print("Saved to {} ({} bytes)".format(out_path, size))


def main():
    parser = argparse.ArgumentParser(
        description="Download audio from Macaulay Library.")
    parser.add_argument("asset_id", help="Macaulay Library asset ID")
    parser.add_argument("--out", default=".",
                        help="Output directory (default: current directory)")
    args = parser.parse_args()

    download_audio(args.asset_id, args.out)


if __name__ == "__main__":
    main()
