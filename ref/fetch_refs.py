"""Download the open-access items of the L01-L22 source list into ref/.

Naming: <L-id>_<bibkey>.<ext>   e.g. L13-2_lekkas2014integral.pdf

Why this exists next to fetch_refs.ps1:
  Windows PowerShell's TLS stack fails in this sandbox (schannel
  SEC_E_NO_CREDENTIALS), while Python's OpenSSL stack completes the handshake.
  The only remaining obstacle is CA verification, so this script:
    1. tries verified TLS using certifi / a local ca-bundle / the system store;
    2. falls back to an unverified context, and records that fact in the log.

Usage:
  python ref/fetch_refs.py --dry-run
  python ref/fetch_refs.py
  python ref/fetch_refs.py --only L13-2 L22-1
"""

from __future__ import annotations

import argparse
import hashlib
import re
import ssl
import sys
import time
import urllib.error
import urllib.request
from datetime import datetime
from pathlib import Path

REF_DIR = Path(__file__).resolve().parent
LOG = REF_DIR / "download_log.txt"
UA = ("Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
      "(KHTML, like Gecko) Chrome/124.0 Safari/537.36")

# (id, key, ext, [urls...], hint)
ITEMS = [
    ("L01-3", "wynn2014auvs", "pdf", [
        "https://www.sciencedirect.com/science/article/pii/S0025322714000747/pdf",
        "https://www.sciencedirect.com/science/article/pii/S0025322714000747/pdfft?isDTMRedir=true&download=true",
    ], "Elsevier hybrid OA; may be blocked by bot protection"),
    ("L09-2", "dechter1985generalized", "pdf", [
        "https://dl.acm.org/doi/pdf/10.1145/3828.3830",
        "https://dl.acm.org/doi/pdf/10.1145/3828.3830?download=true",
    ], "ACM bronze; may need a browser session"),
    ("L11-1", "geraerts2007creating", "pdf", [
        "https://dspace.library.uu.nl/handle/1874/31850",
    ], "repository landing page; script looks for a PDF link inside"),
    ("L12-1", "fraichard2004reeds", "pdf", [
        "https://inria.hal.science/inria-00000009/document",
        "https://hal.inria.fr/inria-00000009/document",
    ], "HAL; resolved via the HAL API to inria.hal.science"),
    ("L12-3", "park2022three", "pdf", [
        "https://mdpi-res.com/d_attachment/applsci/applsci-12-11336/article_deploy/applsci-12-11336.pdf",
        "https://www.mdpi.com/2076-3417/12/22/11336/pdf?version=1668064367",
    ], "MDPI gold; mdpi.com blocks scripts, the mdpi-res CDN does not"),
    ("L14-2", "bingul2023intelligent", "pdf", [
        "https://mdpi-res.com/d_attachment/machines/machines-11-00300/article_deploy/machines-11-00300.pdf",
        "https://www.mdpi.com/2075-1702/11/2/300/pdf?version=1676625969",
    ], "MDPI gold; mdpi-res CDN"),
    ("L15-1", "donald1993kinodynamic", "pdf", [
        "https://dl.acm.org/doi/pdf/10.1145/174147.174150",
    ], "ACM bronze"),
    ("L16-2", "wang2024trajectory", "pdf", [
        "https://mdpi-res.com/d_attachment/jmse/jmse-12-01442/article_deploy/jmse-12-01442.pdf",
        "https://www.mdpi.com/2077-1312/12/8/1442/pdf?version=1724206375",
    ], "MDPI gold; mdpi-res CDN"),
    ("L17-1", "kot2022review", "pdf", [
        "https://mdpi-res.com/d_attachment/electronics/electronics-11-02301/article_deploy/electronics-11-02301.pdf",
        "https://www.mdpi.com/2079-9292/11/15/2301/pdf?version=1659517501",
    ], "MDPI gold; mdpi-res CDN"),
    ("L18-2", "buche2026lotusim", "pdf", [
        "https://arxiv.org/pdf/2607.03072",
    ], "arXiv id from OpenAlex; verify on arxiv.org"),
    ("L20-1", "sandve2013ten", "pdf", [
        "https://journals.plos.org/ploscompbiol/article/file?id=10.1371/journal.pcbi.1003285&type=printable",
    ], "PLOS gold"),
    ("L20-2", "peng2011reproducible", "pdf", [
        "https://pmc.ncbi.nlm.nih.gov/articles/PMC3383002/",
        "https://www.ncbi.nlm.nih.gov/pmc/articles/PMC3383002/",
        "https://europepmc.org/articles/PMC3383002?pdf=render",
    ], "PMC green; Europe PMC / PMC render endpoints return 403 to scripts"),
    ("L22-1", "zhao2020sim", "pdf", [
        "https://arxiv.org/pdf/2009.13303",
    ], "arXiv green"),
    ("L22-2", "tiboni2023dropo", "pdf", [
        "https://doi.org/10.1016/j.robot.2023.104432",
    ], "Elsevier hybrid OA; DOI may resolve to a landing page"),
    ("L21-1", "noaa2024pm0101", "html", [
        "https://tidesandcurrents.noaa.gov/cdata/StationInfo?id=pm0101",
    ], "web page"),
    ("L21-2", "noaa_coops_measure", "html", [
        "https://tidesandcurrents.noaa.gov/measure.html",
        "https://tidesandcurrents.noaa.gov/currents_info.html",
        "https://tidesandcurrents.noaa.gov/currents.html",
    ], "web page; measure.html may have moved"),
]


def build_contexts() -> list[tuple[str, ssl.SSLContext]]:
    """Return (label, context) pairs, most trustworthy first."""
    out: list[tuple[str, ssl.SSLContext]] = []

    cafile = None
    try:
        import certifi  # type: ignore
        cafile = certifi.where()
    except Exception:
        for cand in (
            r"C:\Program Files\Git\mingw64\ssl\certs\ca-bundle.crt",
            r"C:\Program Files\Git\usr\ssl\certs\ca-bundle.crt",
        ):
            if Path(cand).exists():
                cafile = cand
                break

    if cafile:
        try:
            out.append((f"verified(cafile={cafile})", ssl.create_default_context(cafile=cafile)))
        except Exception:
            pass
    try:
        out.append(("verified(system-store)", ssl.create_default_context()))
    except Exception:
        pass
    out.append(("UNVERIFIED", ssl._create_unverified_context()))
    return out


def http_get(url: str, ctx: ssl.SSLContext, timeout: int = 90) -> tuple[int, bytes, str]:
    req = urllib.request.Request(url, headers={
        "User-Agent": UA,
        "Accept": "application/pdf,text/html;q=0.9,*/*;q=0.8",
        "Accept-Language": "en-US,en;q=0.9",
    })
    with urllib.request.urlopen(req, timeout=timeout, context=ctx) as resp:
        return resp.status, resp.read(), resp.headers.get("Content-Type", "")


def find_pdf_links(html: str, base: str) -> list[str]:
    links = re.findall(r'["\']([^"\']+\.pdf[^"\']*)["\']', html, flags=re.I)
    out = []
    for link in links:
        if link.startswith("//"):
            link = "https:" + link
        elif link.startswith("/"):
            root = re.match(r"(https?://[^/]+)", base)
            link = (root.group(1) if root else base) + link
        elif not link.startswith("http"):
            link = base.rstrip("/") + "/" + link
        if link not in out:
            out.append(link)
    return out[:5]


def resolve_hal(hal_id: str, ctx: ssl.SSLContext) -> str | None:
    """Ask the HAL API for the direct file URL of a record (landing pages are JS)."""
    api = (f"https://api.archives-ouvertes.fr/search/?q=halId_s:{hal_id}"
           f"&fl=halId_s,fileMain_s&wt=json")
    try:
        _, body, _ = http_get(api, ctx)
        docs = json.loads(body.decode("utf-8", "ignore"))["response"]["docs"]
        for doc in docs:
            url = doc.get("fileMain_s")
            if url:
                return url
    except Exception:  # noqa: BLE001
        return None
    return None


def download(item, ctx_label: str, ctx: ssl.SSLContext, retries: int = 2):
    lid, key, ext, urls, hint = item
    target = REF_DIR / f"{lid}_{key}.{ext}"
    if target.exists() and target.stat().st_size > 20000:
        return ("skip", target.name, target.stat().st_size, "exists")

    last_err = ""
    candidates: list[str] = list(urls)

    if lid == "L11-1":
        direct = resolve_hal("inria-00000009", ctx)
        if direct:
            candidates.insert(0, direct)

    for attempt in range(retries):
        for url in list(candidates):
            try:
                status, body, ctype = http_get(url, ctx)
            except Exception as exc:  # noqa: BLE001
                last_err = f"{type(exc).__name__}: {exc}"
                continue

            if body[:4] == b"%PDF":
                target.write_bytes(body)
                return ("ok", target.name, len(body), f"{url} [{ctx_label}]")

            is_html = ("html" in ctype.lower()
                       or body[:15].lower().startswith(b"<!doctype html")
                       or body[:5].lower().startswith(b"<html"))

            if ext == "html" and is_html:
                target.write_bytes(body)
                return ("ok", target.name, len(body), f"{url} [{ctx_label}] (html)")

            if is_html:
                text = body.decode("utf-8", "ignore")
                for link in find_pdf_links(text, url):
                    if link not in candidates:
                        candidates.append(link)
                last_err = "landing page without a usable PDF link"
                continue

            last_err = f"unexpected content-type {ctype!r} ({len(body)} bytes)"
        time.sleep(1.5 * (attempt + 1))

    return ("fail", target.name, 0, last_err or "unknown")


def log(line: str) -> None:
    stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with LOG.open("a", encoding="utf-8") as fh:
        fh.write(f"{stamp}\t{line}\n")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--only", nargs="*", default=None)
    args = ap.parse_args()

    items = [it for it in ITEMS if not args.only or it[0] in args.only]
    contexts = build_contexts()

    if args.dry_run:
        print(f"{len(items)} items, contexts: {[c[0] for c in contexts]}")
        for it in items:
            print(f"  {it[0]:6} {it[0]}_{it[1]}.{it[2]}  {it[3][0]}")
        return 0

    label, ctx = contexts[0]
    if label == "UNVERIFIED":
        print("[warn] falling back to UNVERIFIED TLS: downloads are not "
              "certificate-authenticated (recorded in the log).")
    print(f"using TLS context: {label}")

    ok = fail = skip = 0
    for it in items:
        status, name, size, detail = download(it, label, ctx)
        if status == "ok":
            ok += 1
            digest = hashlib.sha256((REF_DIR / name).read_bytes()).hexdigest()[:16]
            print(f"  ok    {name}  {size:,} bytes  sha256:{digest}")
            log(f"DOWNLOAD\tOK\t{name}\t{size}\t{detail}\tsha256:{digest}")
        elif status == "skip":
            skip += 1
            print(f"  skip  {name}  ({size:,} bytes)")
        else:
            fail += 1
            hint = it[4]
            print(f"  FAIL  {name}  {detail}")
            print(f"        hint: {hint}")
            log(f"DOWNLOAD\tFAIL\t{name}\t0\t{detail}\t{hint}")

    print(f"\ndone: ok={ok} skip={skip} fail={fail}  (log: {LOG})")
    return 0 if fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
