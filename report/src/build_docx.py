"""Builds report/Report.docx (editable Word file, native Word equations) from report_template.html.

Needs pandoc (>= 2.x). Usage:  python report/src/build_docx.py
"""
import os
import re
import subprocess
import tempfile
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPORT = os.path.dirname(HERE)
OUT = os.path.join(REPORT, "Report.docx")

FONT = "Calibri"
ACCENT = "2457A6"
MUTED = "5F6368"


def prepare_html(src):
    src = re.sub(r"<style>[\s\S]*?</style>", "", src)
    src = re.sub(r"<title>[\s\S]*?</title>", "", src)       # else pandoc adds it as a second title
    # inline math \( .. \) -> $..$ (pandoc's tex_math_dollars); $$..$$ stays display math
    src = re.sub(r"\\\(\s*([\s\S]+?)\s*\\\)", lambda m: "$" + m.group(1) + "$", src)
    # KaTeX-only macro
    src = src.replace(r"\dddot x", r"\overset{\cdots}{x}")
    src = src.replace(r"\Big|1 - ", r"\left|1 - ").replace(r"\Big| &lt; 1", r"\right| &lt; 1")
    # the PDF splits the Catmull-Rom spline over two lines; Word wraps long equations itself
    src = re.sub(r"\\Big\[(.*?)\$\$</div>\s*<div class=\"eq\">\$\$\\qquad\\qquad \+(.*?)\\Big\]",
                 r"\\left[\1 +\2\\right]", src, flags=re.S)

    # summary cards -> a 4-column table
    def glance(m):
        cells = re.findall(r'<div class="v">(.*?)</div><div class="l">(.*?)</div>', m.group(0))
        tds = "".join(f"<td><b>{v}</b><br>{l}</td>" for v, l in cells)
        return f"<table><tr>{tds}</tr></table>"
    src = re.sub(r'<div class="glance">[\s\S]*?\n</div>', glance, src)

    # figure widths (in px at 96 dpi): one image 16 cm, two side by side 7.7 cm each, three 5.2 cm each
    def figure(m):
        cls = m.group(1) or ""
        width = 195 if "row3" in cls else 290 if "row" in cls else 600
        imgs = re.sub(r"<img ", f'<img width="{width}" ', m.group(2))
        return f"<p>{imgs}</p>"
    src = re.sub(r'<figure(?: class="([^"]*)")?>([\s\S]*?)</figure>', figure, src)

    # paragraph classes -> Word paragraph styles
    for cls, style in [("words", "In Words"), ("cap", "Figure Caption"), ("note", "Note")]:
        src = re.sub(rf'<p class="{cls}">([\s\S]*?)</p>',
                     rf'<div custom-style="{style}"><p>\1</p></div>', src)
    src = src.replace('<p class="sub">', '<div custom-style="Subtitle"><p>').replace(
        "KUKA iiwa in PyBullet, 1 kHz simulation</p>", "KUKA iiwa in PyBullet, 1 kHz simulation</p></div>")
    src = re.sub(r'<span class="n">(\d)</span>', r"\1  ", src)
    return src


def style_xml(name, style_id, ppr, rpr, based="Normal"):
    return (f'<w:style w:type="paragraph" w:customStyle="1" w:styleId="{style_id}"><w:name w:val="{name}"/>'
            f'<w:basedOn w:val="{based}"/><w:qFormat/><w:pPr>{ppr}</w:pPr><w:rPr>{rpr}</w:rPr></w:style>')


def make_reference(path):
    """pandoc's default reference.docx with a simple, clean look."""
    with open(path, "wb") as f:
        f.write(subprocess.run(["pandoc", "--print-default-data-file", "reference.docx"],
                               check=True, capture_output=True).stdout)
    with zipfile.ZipFile(path) as z:
        files = {n: z.read(n) for n in z.namelist()}
    s = files["word/styles.xml"].decode()

    # fonts and sizes
    s = re.sub(r'(<w:docDefaults>[\s\S]*?)<w:rFonts [^>]*/>',
               rf'\1<w:rFonts w:ascii="{FONT}" w:hAnsi="{FONT}" w:eastAsia="{FONT}" w:cs="{FONT}"/>', s, count=1)
    s = re.sub(r'(<w:docDefaults>[\s\S]*?<w:sz w:val=")\d+(")', r"\g<1>21\2", s)
    s = re.sub(r'(<w:docDefaults>[\s\S]*?<w:szCs w:val=")\d+(")', r"\g<1>21\2", s)

    def restyle(style_id, rpr, ppr=None):
        nonlocal s
        m = re.search(rf'<w:style [^>]*w:styleId="{style_id}"[^>]*>[\s\S]*?</w:style>', s)
        if not m:
            return
        block = re.sub(r"<w:rPr>[\s\S]*?</w:rPr>", f"<w:rPr>{rpr}</w:rPr>", m.group(0))
        if ppr is not None:
            block = re.sub(r"<w:pPr>[\s\S]*?</w:pPr>", f"<w:pPr>{ppr}</w:pPr>", block)
        s = s.replace(m.group(0), block)

    keep = "<w:keepNext/><w:keepLines/>"
    restyle("Title", f'<w:b/><w:color w:val="1D1D1F"/><w:sz w:val="40"/><w:szCs w:val="40"/>',
            '<w:spacing w:before="0" w:after="80"/>')
    restyle("Heading1", f'<w:b/><w:color w:val="1D1D1F"/><w:sz w:val="40"/><w:szCs w:val="40"/>',
            f'{keep}<w:spacing w:before="0" w:after="80"/><w:outlineLvl w:val="0"/>')
    restyle("Heading2", f'<w:b/><w:color w:val="{ACCENT}"/><w:sz w:val="28"/><w:szCs w:val="28"/>',
            f'{keep}<w:pBdr><w:top w:val="single" w:sz="12" w:space="6" w:color="1D1D1F"/></w:pBdr>'
            f'<w:spacing w:before="360" w:after="120"/><w:outlineLvl w:val="1"/>')
    restyle("Heading3", f'<w:b/><w:color w:val="1D1D1F"/><w:sz w:val="22"/><w:szCs w:val="22"/>',
            f'{keep}<w:spacing w:before="200" w:after="60"/><w:outlineLvl w:val="2"/>')
    for sid in ("BodyText", "FirstParagraph", "Compact"):
        restyle(sid, "", '<w:spacing w:before="60" w:after="60" w:line="276" w:lineRule="auto"/>')

    extra = (
        style_xml("Subtitle", "Subtitle", '<w:spacing w:after="200"/>',
                  f'<w:color w:val="{MUTED}"/><w:sz w:val="20"/>')
        + style_xml("In Words", "InWords",
                    '<w:pBdr><w:left w:val="single" w:sz="18" w:space="8" w:color="D9DCE1"/></w:pBdr>'
                    '<w:spacing w:before="40" w:after="140"/><w:ind w:left="200"/>',
                    f'<w:color w:val="{MUTED}"/>')
        + style_xml("Figure Caption", "FigureCaption", '<w:jc w:val="center"/><w:spacing w:before="40" w:after="200"/>',
                    f'<w:color w:val="{MUTED}"/><w:sz w:val="18"/>')
        + style_xml("Note", "Note", '<w:spacing w:before="40" w:after="100"/>',
                    f'<w:color w:val="{MUTED}"/><w:sz w:val="19"/>')
    )
    s = re.sub(r'<w:style [^>]*w:styleId="Subtitle"[^>]*>[\s\S]*?</w:style>', "", s)
    s = s.replace("</w:styles>", extra + "</w:styles>")

    # tables: thin horizontal rules, shaded header row
    table_style = (
        '<w:tblPr><w:tblBorders><w:top w:val="single" w:sz="4" w:color="D9DCE1"/>'
        '<w:bottom w:val="single" w:sz="4" w:color="D9DCE1"/>'
        '<w:insideH w:val="single" w:sz="4" w:color="D9DCE1"/></w:tblBorders>'
        '<w:tblCellMar><w:top w:w="40" w:type="dxa"/><w:left w:w="100" w:type="dxa"/>'
        '<w:bottom w:w="40" w:type="dxa"/><w:right w:w="100" w:type="dxa"/></w:tblCellMar></w:tblPr>'
        '<w:tblStylePr w:type="firstRow"><w:rPr><w:b/></w:rPr>'
        '<w:tcPr><w:shd w:val="clear" w:color="auto" w:fill="F5F6F8"/></w:tcPr></w:tblStylePr>')
    s = re.sub(r'(<w:style [^>]*w:styleId="Table"[^>]*>[\s\S]*?)<w:tblPr>[\s\S]*?</w:tblPr>',
               r"\1" + table_style.replace("\\", "\\\\"), s, count=1)
    files["word/styles.xml"] = s.encode()

    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        for n, data in files.items():
            z.writestr(n, data)


def postprocess(path):
    """A4 page with 1.5 cm margins, full-width tables, no autofit."""
    with zipfile.ZipFile(path) as z:
        files = {n: z.read(n) for n in z.namelist()}
    d = files["word/document.xml"].decode()
    sect = ('<w:sectPr><w:pgSz w:w="11906" w:h="16838"/><w:pgMar w:top="907" w:right="851" w:bottom="907" '
            'w:left="851" w:header="567" w:footer="567" w:gutter="0"/></w:sectPr>')
    d = re.sub(r"<w:sectPr\s*/>|<w:sectPr[\s\S]*?</w:sectPr>", "", d).replace("</w:body>", sect + "</w:body>")
    d = re.sub(r'<w:tblW w:type="\w+" w:w="\d+" />', '<w:tblW w:type="pct" w:w="5000" />', d)
    d = d.replace('<w:tblLayout w:type="autofit"/>', "")
    # pandoc's math converter escapes brackets inside \text{..} as "\(" "\)"
    d = re.sub(r"(<m:t[^>]*>)([^<]*)", lambda m: m.group(1) + m.group(2).replace("\\(", "(").replace("\\)", ")"), d)
    files["word/document.xml"] = d.encode()
    # strict-schema tidy-ups: declare png by extension, drop pandoc's out-of-order zoom setting
    ct = files["[Content_Types].xml"].decode()
    if 'Extension="png"' not in ct:
        ct = ct.replace("<Default ", '<Default Extension="png" ContentType="image/png" /><Default ', 1)
    files["[Content_Types].xml"] = ct.encode()
    files["word/settings.xml"] = order_settings(files["word/settings.xml"].decode()).encode()
    for name in ("word/document.xml", "word/styles.xml"):
        files[name] = schema_order(files[name])
    files["word/numbering.xml"] = re.sub(rb'(<w:nsid w:val=")([0-9A-Fa-f]{1,7})(")',
                                         lambda m: m.group(1) + m.group(2).rjust(8, b"0") + m.group(3),
                                         files["word/numbering.xml"])
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        for n, data in files.items():
            z.writestr(n, data)


# element order required by the WordprocessingML schema (CT_Settings); pandoc writes them out of order
SETTINGS_ORDER = ["embedSystemFonts", "proofState", "stylePaneFormatFilter", "doNotTrackMoves", "defaultTabStop",
                  "drawingGridHorizontalSpacing", "drawingGridVerticalSpacing", "displayHorizontalDrawingGridEvery",
                  "displayVerticalDrawingGridEvery", "characterSpacingControl", "savePreviewPicture", "footnotePr",
                  "m:mathPr", "themeFontLang", "clrSchemeMapping", "decimalSymbol", "listSeparator"]


def order_settings(xml):
    head, body = re.match(r"([\s\S]*?<w:settings[^>]*>)([\s\S]*)</w:settings>", xml).groups()
    parts = []
    for name in SETTINGS_ORDER:
        tag = name if ":" in name else "w:" + name
        m = re.search(rf"<{tag}\b[^>]*/>|<{tag}\b[^>]*>[\s\S]*?</{tag}>", body)
        if m:
            parts.append(m.group(0))
    return head + "".join(parts) + "</w:settings>"


W = "{http://schemas.openxmlformats.org/wordprocessingml/2006/main}"
M = "{http://schemas.openxmlformats.org/officeDocument/2006/math}"
# child order required by the schema for the elements pandoc writes out of order
CHILD_ORDER = {
    W + "style": ["name", "aliases", "basedOn", "next", "link", "autoRedefine", "uiPriority", "semiHidden",
                  "unhideWhenUsed", "qFormat", "locked", "personal", "personalCompose", "personalReply", "rsid",
                  "pPr", "rPr", "tblPr", "trPr", "tcPr", "tblStylePr"],
    W + "pPr": ["pStyle", "keepNext", "keepLines", "pageBreakBefore", "framePr", "widowControl", "numPr",
                "suppressLineNumbers", "pBdr", "shd", "tabs", "suppressAutoHyphens", "kinsoku", "wordWrap",
                "overflowPunct", "topLinePunct", "autoSpaceDE", "autoSpaceDN", "bidi", "adjustRightInd",
                "snapToGrid", "spacing", "ind", "contextualSpacing", "mirrorIndents", "suppressOverlap", "jc",
                "textDirection", "textAlignment", "textboxTightWrap", "outlineLvl", "divId", "cnfStyle", "rPr",
                "sectPr", "pPrChange"],
    W + "tblPr": ["tblStyle", "tblpPr", "tblOverlap", "bidiVisual", "tblStyleRowBandSize", "tblStyleColBandSize",
                  "tblW", "jc", "tblCellSpacing", "tblInd", "tblBorders", "shd", "tblLayout", "tblCellMar",
                  "tblLook", "tblCaption", "tblDescription"],
    W + "tcPr": ["cnfStyle", "tcW", "gridSpan", "hMerge", "vMerge", "tcBorders", "shd", "noWrap", "tcMar",
                 "textDirection", "tcFitText", "vAlign", "hideMark"],
    M + "dPr": ["begChr", "sepChr", "endChr", "grow", "shp", "ctrlPr"],
    M + "mcPr": ["count", "mcJc"],
}


def schema_order(data):
    """Put children of the elements in CHILD_ORDER into schema order and fix two other pandoc quirks."""
    from lxml import etree
    root = etree.fromstring(data)
    for parent_tag, order in CHILD_ORDER.items():
        rank = {name: i for i, name in enumerate(order)}
        for el in root.iter(parent_tag):
            kids = list(el)
            kids.sort(key=lambda k: rank.get(etree.QName(k).localname, len(order)))   # stable sort
            for k in kids:
                el.append(k)
    for el in root.iter(W + "style"):                 # keep only the first tblStylePr of each type
        seen = set()
        for t in el.findall(W + "tblStylePr"):
            if t.get(W + "type") in seen:
                el.remove(t)
            seen.add(t.get(W + "type"))
    for el in root.iter(W + "rPr"):                   # stray text inside run properties
        el.text = None
        for k in el:
            k.tail = None
    for el in root.iter(M + "rPr"):                   # m:nor and m:sty are alternatives
        if el.find(M + "nor") is not None:
            for s in el.findall(M + "sty"):
                el.remove(s)
    return etree.tostring(root, xml_declaration=True, encoding="UTF-8", standalone=True)


def main():
    html = prepare_html(open(os.path.join(HERE, "report_template.html"), encoding="utf8").read())
    with tempfile.TemporaryDirectory() as tmp:
        src = os.path.join(tmp, "report.html")
        ref = os.path.join(tmp, "reference.docx")
        open(src, "w", encoding="utf8").write(html)
        make_reference(ref)
        subprocess.run(["pandoc", "-f", "html+tex_math_dollars", src, "-o", OUT,
                        "--reference-doc", ref, "--resource-path", REPORT], check=True)
    postprocess(OUT)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
