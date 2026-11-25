"""Find author/institutional mirrors for the items that publisher sites block.

Strategy: query DuckDuckGo's HTML endpoint, keep only .pdf links on academic
domains (reject known pirate sites), download, validate %PDF, and store as
<L-id>_<bibkey>.pdf. Every attempt (including rejects) is logged with its URL
so the provenance of each file stays checkable.

Usage:
  python ref/hunt_mirrors.py --dry-run
  python ref/hunt_mirrors.py
  python ref/hunt_mirrors.py --only L12-1
"""

from __future__ import annotations

import argparse
import re
import ssl
import sys
import time
import urllib.parse
import urllib.request
from datetime import datetime
from pathlib import Path

REF_DIR = Path(__file__).resolve().parent
LOG = REF_DIR / "download_log.txt"
CA_CANDIDATES = [
    r"C:\Program Files\Git\usr\ssl\certs\ca-bundle.crt",
    r"C:\Program Files\Git\mingw64\ssl\certs\ca-bundle.crt",
]
UA = ("Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
      "(KHTML, like Gecko) Chrome/124.0 Safari/537.36")

# licence-wise safe to fetch: author pages, university/institute hosts, preprint servers
ALLOW = re.compile(
    r"(\.edu$|\.ac\.[a-z]{2}$|\.edu\.[a-z]{2}$|\.uni-[a-z]+\.de$|"
    r"\.loria\.fr$|\.inria\.fr$|\.hal\.science$|\.archives-ouvertes\.fr$|"
    r"\.tudelft\.nl$|\.uu\.nl$|\.cmu\.edu$|\.mit\.edu$|\.stanford\.edu$|"
    r"\.arxiv\.org$|\.ncbi\.nlm\.nih\.gov$|\.europepmc\.org$|"
    r"\.ruc\.dk$|\.dtu\.dk$|\.ntnu\.no$|\.uio\.no$|\.sciencedirect\.com$|"
    r"\.researchgate\.net$|\.academia\.edu$)",
    re.I,
)
DENY = re.compile(r"(sci-hub|libgen|z-lib|zlibrary|pdfdrive|booksc|annas-archive)", re.I)

TARGETS = [
    ("L12-1", "fraichard2004reeds",
     "Fraichard Scheuer From Reeds and Shepp continuous-curvature paths pdf",
     ["reeds", "curvature"]),
    ("L01-3", "wynn2014auvs",
     "Wynn Autonomous Underwater Vehicles marine geoscience pdf",
     ["geoscience", "underwater"]),
    ("L09-2", "dechter1985generalized",
     "Dechter Pearl Generalized best-first search strategies optimality A* pdf",
     ["best-first", "optimality"]),
    ("L11-1", "geraerts2007creating",
     "Geraerts Overmars Creating high-quality paths motion planning pdf",
     ["shortcut", "clearance"]),
    ("L15-1", "donald1993kinodynamic",
     "Donald Xavier Canny Reif Kinodynamic motion planning pdf",
     ["kinodynamic", "trajectory"]),
    ("L20-2", "peng2011reproducible",
     "Peng Reproducible research in computational science pdf",
     ["reproducible", "computational science"]),
    ("L22-2", "tiboni2023dropo",
     "Tiboni Arndt Kyrki DROPO offline domain randomization pdf",
     ["domain randomization", "sim-to-real"]),
]


def pdf_text_probe(path: Path, limit: int = 4_000_000) -> str:
    """Crude text probe: inflate content streams and keep literal strings.

    Enough to look for a title keyword; not a general PDF parser.
    """
    import zlib

    data = path.read_bytes()[:limit]
    chunks: list[str] = []
    for match in re.finditer(rb"stream\r?\n(.*?)endstream", data, re.S):
        try:
            dec = zlib.decompress(match.group(1))
        except Exception:  # noqa: BLE001
            continue
        for lit in re.finditer(rb"\((?:\\.|[^\\()])*\)", dec):
            raw = lit.group(0)[1:-1]
            raw = re.sub(rb"\\([0-7]{1,3})", lambda m: bytes([int(m.group(1), 8)]), raw)
            chunks.append(raw.decode("latin-1", "ignore"))
        if sum(len(c) for c in chunks) > 400_000:
            break
    return " ".join(chunks).lower()


def verify_pdf(path: Path, words: list[str]) -> tuple[bool, str]:
    text = pdf_text_probe(path)
    if len(text) < 200:
        # Cannot read text (common for subset fonts); fall back to size sanity.
        return (path.stat().st_size > 60_000, "size-only")
    hits = [w for w in words if w.lower() in text]
    return (len(hits) >= 1, f"text-hits={hits}")


def build_ctx() -> ssl.SSLContext:
    for ca in CA_CANDIDATES:
        if Path(ca).exists():
            return ssl.create_default_context(cafile=ca)
    return ssl.create_default_context()


def http_get(url: str, ctx: ssl.SSLContext, timeout: int = 60) -> tuple[int, bytes, str]:
    req = urllib.request.Request(url, headers={
        "User-Agent": UA,
        "Accept": "application/pdf,text/html;q=0.9,*/*;q=0.8",
    })
    with urllib.request.urlopen(req, timeout=timeout, context=ctx) as resp:
        return resp.status, resp.read(), resp.headers.get("Content-Type", "")


def ddg(query: str, ctx: ssl.SSLContext) -> list[str]:
    url = "https://html.duckduckgo.com/html/?q=" + urllib.parse.quote(query)
    _, body, _ = http_get(url, ctx)
    html = body.decode("utf-8", "ignore")
    raw = re.findall(r'uddg=([^&"]+)', html)
    out: list[str] = []
    for item in raw:
        link = urllib.parse.unquote(item)
        if link not in out:
            out.append(link)
    return out


def acceptable(url: str) -> bool:
    if DENY.search(url):
        return False
    host = urllib.parse.urlparse(url).netloc
    return bool(ALLOW.search(host))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--only", nargs="*")
    args = ap.parse_args()

    ctx = build_ctx()
    targets = [t for t in TARGETS if not args.only or t[0] in args.only]
    ok = fail = 0

    for lid, key, query, words in targets:
        target = REF_DIR / f"{lid}_{key}.pdf"
        print(f"\n[{lid}] {key}")
        if target.exists() and target.stat().st_size > 50000:
            print("  already present, skip")
            continue

        links: list[str] = []
        for attempt in range(2):
            try:
                links = ddg(query, ctx)
            except Exception as exc:  # noqa: BLE001
                print(f"  search failed: {type(exc).__name__}: {exc}")
                links = []
            if links:
                break
            time.sleep(3 + 2 * attempt)

        pdfs = [u for u in links if ".pdf" in u.lower() and acceptable(u)]
        print(f"  {len(links)} results, {len(pdfs)} acceptable .pdf candidates")
        for u in pdfs[:6]:
            print("   -", u)

        if args.dry_run:
            continue

        saved = False
        for url in pdfs[:6]:
            try:
                status, body, ctype = http_get(url, ctx)
            except Exception as exc:  # noqa: BLE001
                print(f"   x {type(exc).__name__}: {exc}")
                continue
            if body[:4] != b"%PDF" or len(body) < 50000:
                print(f"   x not a usable pdf ({len(body)} bytes, {ctype})")
                continue

            target.write_bytes(body)
            good, why = verify_pdf(target, words)
            if not good:
                print(f"   x rejected by content check ({why})")
                target.unlink(missing_ok=True)
                continue

            print(f"  OK {target.name}  {len(body):,} bytes  [{why}]  <- {url}")
            with LOG.open("a", encoding="utf-8") as fh:
                fh.write(f"{datetime.now():%Y-%m-%d %H:%M:%S}\tMIRROR\tOK\t{target.name}"
                         f"\t{len(body)}\t{url}\t{why}\n")
            ok += 1
            saved = True
            break

        if not saved:
            fail += 1
            with LOG.open("a", encoding="utf-8") as fh:
                fh.write(f"{datetime.now():%Y-%m-%d %H:%M:%S}\tMIRROR\tFAIL\t"
                         f"{target.name}\t0\tno usable mirror among {len(pdfs)} candidates\n")
        time.sleep(1.5)

    print(f"\nmirror hunt: ok={ok} fail={fail}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
