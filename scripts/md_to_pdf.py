#!/usr/bin/env python3
"""Render a Markdown document to PDF using headless Chrome.

Written rather than installed: the machine has no pandoc, no wkhtmltopdf and no
python-markdown, and a submission deadline is the wrong moment to add a
toolchain. Chrome is already present and prints PDFs from the command line.

Supports exactly what these documents use -- headings, tables, fenced code,
lists, bold, inline code, links, rules -- and nothing else.

    python3 md_to_pdf.py in.md out.pdf ["Header text"]
"""
import html
import os
import re
import subprocess
import sys
import tempfile

CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"

CSS = """
@page { size: A4; margin: 16mm 16mm 16mm 16mm; }
body { font: 10pt/1.45 "Calibri", "Helvetica Neue", Arial, sans-serif;
       color: #1a1a1a; margin: 0; }
h1 { font-size: 19pt; font-weight: 700; margin: 0 0 10pt; }
h2 { font-size: 13pt; font-weight: 700; margin: 16pt 0 6pt;
     page-break-after: avoid; }
h3 { font-size: 11pt; font-weight: 700; margin: 11pt 0 4pt;
     page-break-after: avoid; }
p, li { margin: 0 0 6pt; }
ul, ol { margin: 0 0 8pt; padding-left: 18pt; }
li { margin-bottom: 3pt; }
strong { font-weight: 700; }
code { font: 9pt "Consolas", Menlo, monospace; }
pre { background: #fafafa; border: 1px solid #dcdcdc; padding: 8pt 10pt;
      overflow-x: auto; page-break-inside: avoid; margin: 0 0 9pt; }
pre code { font-size: 8.6pt; line-height: 1.4; }
/* Plain Word-style tables: no fills, single hairline rules. */
table { border-collapse: collapse; width: 100%; margin: 0 0 10pt;
        font-size: 9.2pt; page-break-inside: avoid; }
th { text-align: left; font-weight: 700; border: 1px solid #9a9a9a;
     padding: 4pt 6pt; }
td { border: 1px solid #9a9a9a; padding: 4pt 6pt; vertical-align: top; }
blockquote { margin: 0 0 8pt; padding-left: 10pt; border-left: 2px solid #c8c8c8;
             color: #444; }
a { color: #1a1a1a; text-decoration: none; }
.pending { color: #c00000; font-size: 9pt; font-style: italic;
           margin: 0 0 8pt; }
svg { display: block; margin: 4pt auto 12pt; max-width: 100%; }
"""


def inline(t):
    t = html.escape(t, quote=False)
    t = re.sub(r"`([^`]+)`", r"<code>\1</code>", t)
    t = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", t)
    t = re.sub(r"(?<![\w*])\*([^*\n]+)\*(?![\w*])", r"<em>\1</em>", t)
    t = re.sub(r"\[([^\]]+)\]\(([^)]+)\)", r'<a href="\2">\1</a>', t)
    return t


def cells(line):
    return [c.strip() for c in line.strip().strip("|").split("|")]


def convert(md):
    out, i, lines = [], 0, md.split("\n")
    while i < len(lines):
        ln = lines[i]

        if ln.startswith("```"):
            buf, i = [], i + 1
            while i < len(lines) and not lines[i].startswith("```"):
                buf.append(html.escape(lines[i], quote=False))
                i += 1
            out.append("<pre><code>%s</code></pre>" % "\n".join(buf))
            i += 1
            continue

        # table: a header row followed by a |---| separator
        if (ln.strip().startswith("|") and i + 1 < len(lines)
                and re.match(r"^\s*\|[\s:|-]+\|\s*$", lines[i + 1])):
            head = cells(ln)
            i += 2
            rows = []
            while i < len(lines) and lines[i].strip().startswith("|"):
                rows.append(cells(lines[i]))
                i += 1
            t = ["<table><thead><tr>"]
            t += ["<th>%s</th>" % inline(c) for c in head]
            t.append("</tr></thead><tbody>")
            for r in rows:
                t.append("<tr>" + "".join("<td>%s</td>" % inline(c)
                                          for c in r) + "</tr>")
            t.append("</tbody></table>")
            out.append("".join(t))
            continue

        # Raw HTML block (used for inline SVG diagrams and the red pending
        # notes). Copy it through untouched.
        if ln.lstrip().startswith(("<svg", "<div", "<p class=")):
            tag = ln.lstrip().split()[0].lstrip("<").rstrip(">")
            buf = []
            while i < len(lines):
                buf.append(lines[i])
                if ("</%s>" % tag) in lines[i]:
                    i += 1
                    break
                i += 1
            out.append("\n".join(buf))
            continue

        m = re.match(r"^(#{1,4})\s+(.*)$", ln)
        if m:
            lvl = len(m.group(1))
            out.append("<h%d>%s</h%d>" % (lvl, inline(m.group(2)), lvl))
            i += 1
            continue

        # Separator rules are dropped: headings already delimit sections and
        # the printed documents read better without them.
        if re.match(r"^\s*---+\s*$", ln):
            i += 1
            continue

        if re.match(r"^\s*[-*]\s+", ln) or re.match(r"^\s*\d+\.\s+", ln):
            ordered = bool(re.match(r"^\s*\d+\.\s+", ln))
            items = []
            while i < len(lines) and (re.match(r"^\s*[-*]\s+", lines[i])
                                      or re.match(r"^\s*\d+\.\s+", lines[i])
                                      or (lines[i].startswith("   ")
                                          and lines[i].strip() and items)):
                s = lines[i]
                if re.match(r"^\s*[-*]\s+", s) or re.match(r"^\s*\d+\.\s+", s):
                    items.append(re.sub(r"^\s*(?:[-*]|\d+\.)\s+", "", s))
                else:
                    items[-1] += " " + s.strip()   # continuation line
                i += 1
            tag = "ol" if ordered else "ul"
            out.append("<%s>%s</%s>"
                       % (tag, "".join("<li>%s</li>" % inline(x)
                                       for x in items), tag))
            continue

        if not ln.strip():
            i += 1
            continue

        para = []
        while i < len(lines) and lines[i].strip() \
                and not lines[i].startswith(("#", "```", "|", "- ", "* ")) \
                and not lines[i].lstrip().startswith(("<svg", "<div", "<p class=")) \
                and not re.match(r"^\s*\d+\.\s+", lines[i]) \
                and not re.match(r"^\s*---+\s*$", lines[i]):
            para.append(lines[i].strip())
            i += 1
        if para:
            out.append("<p>%s</p>" % inline(" ".join(para)))
    return "\n".join(out)


def main():
    src, dst = sys.argv[1], sys.argv[2]
    with open(src) as fh:
        body = convert(fh.read())
    page = ("<!doctype html><html><head><meta charset='utf-8'>"
            "<style>%s</style></head><body>%s</body></html>" % (CSS, body))

    tmp = tempfile.NamedTemporaryFile("w", suffix=".html", delete=False)
    tmp.write(page)
    tmp.close()

    subprocess.run([CHROME, "--headless", "--disable-gpu", "--no-pdf-header-footer",
                    "--print-to-pdf=%s" % os.path.abspath(dst),
                    "file://%s" % tmp.name],
                   capture_output=True, timeout=120)
    os.unlink(tmp.name)
    ok = os.path.exists(dst) and os.path.getsize(dst) > 2000
    print("%s -> %s  (%s)" % (src, dst,
                              "%.0f KB" % (os.path.getsize(dst) / 1024.0)
                              if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
