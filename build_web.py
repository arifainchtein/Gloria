#!/usr/bin/env python3
"""Builds data/ (the www LittleFS image) from the page sources in web/.

Gloria's copy of Daffodil's build_web.py (2026-11-18), unchanged apart from this note: every
<link rel="stylesheet">, <script src> and <img src> in web/index.html is inlined, fonts inside the CSS
become data: URIs, and the page is written as one gzipped data/index.html.gz. The web server sends it
for /index.html with Content-Encoding: gzip (ESPAsyncWebServer falls back to the .gz on its own), so a
page load over the soft AP is one request plus the page's /LangleyServlet and /GloriaServlet calls.

Edit the files in web/, run  python3 build_web.py , then flash with ./uploadData.sh. A factory release
(ProjectWallLabelMaker, WwwImageBuilder) runs this script itself before packing data/.
"""
import base64
import gzip
import pathlib
import re
import shutil

ROOT = pathlib.Path(__file__).resolve().parent
WEB = ROOT / "web"
DATA = ROOT / "data"

MIME = {
    ".svg": "image/svg+xml", ".png": "image/png", ".jpg": "image/jpeg", ".gif": "image/gif",
    ".woff2": "font/woff2", ".woff": "font/woff", ".ttf": "font/ttf", ".ico": "image/x-icon",
}


def source(ref):
    """The web/ file a page reference points to, by file name (assets/js/x.js -> web/x.js)."""
    name = ref.split("?")[0].split("#")[0].rsplit("/", 1)[-1]
    path = WEB / name
    return path if path.is_file() else None


def data_uri(path):
    mime = MIME.get(path.suffix.lower())
    if not mime:
        raise SystemExit("no MIME type for %s" % path.name)
    return "data:%s;base64,%s" % (mime, base64.b64encode(path.read_bytes()).decode("ascii"))


def inline_css(css, from_name):
    def url(m):
        ref = m.group(2)
        if ref.startswith("data:"):
            return m.group(0)
        path = source(ref)
        if path is None:
            return m.group(0)  # a font fallback that isn't shipped - see the module docstring
        return "url(%s)" % data_uri(path)
    return re.sub(r"""url\((['"]?)([^'")]+)\1\)""", url, css)


def must(ref, what):
    path = source(ref)
    if path is None:
        raise SystemExit("index.html: %s %r is not in web/" % (what, ref))
    return path


def main():
    html = (WEB / "index.html").read_text(encoding="utf-8")
    counts = {"css": 0, "js": 0, "img": 0}

    def style(m):
        path = must(m.group(1), "stylesheet")
        counts["css"] += 1
        return "<style>\n" + inline_css(path.read_text(encoding="utf-8"), path.name) + "\n</style>"
    html = re.sub(r"""<link\s+rel=["']stylesheet["']\s+href=["']([^"']+)["']\s*/?>""", style, html)

    def script(m):
        path = must(m.group(1), "script")
        counts["js"] += 1
        # "</script" inside a script would end the inline block early.
        return "<script>\n" + path.read_text(encoding="utf-8").replace("</script", "<\\/script") + "\n</script>"
    html = re.sub(r"""<script\s+src=["']([^"']+)["']\s*>\s*</script>""", script, html)

    def img(m):
        path = must(m.group(2), "image")
        counts["img"] += 1
        return m.group(1) + data_uri(path) + m.group(3)
    html = re.sub(r"""(<img\b[^>]*?\ssrc=["'])(?!data:)([^"']+)(["'])""", img, html)

    left = re.findall(r"""<(?:link|script)[^>]+(?:href|src)=["'](?!data:|#)[^"']+["']""", html)
    if left:
        raise SystemExit("index.html: references not inlined: %s" % left)

    if DATA.exists():
        shutil.rmtree(DATA)
    DATA.mkdir()
    raw = html.encode("utf-8")
    packed = gzip.compress(raw, compresslevel=9, mtime=0)
    (DATA / "index.html.gz").write_bytes(packed)
    print("inlined %d stylesheets, %d scripts, %d images" % (counts["css"], counts["js"], counts["img"]))
    print("data/index.html.gz: %d KB (%d KB before gzip)" % (len(packed) // 1024, len(raw) // 1024))


if __name__ == "__main__":
    main()
