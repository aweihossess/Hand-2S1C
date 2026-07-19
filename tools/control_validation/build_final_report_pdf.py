#!/usr/bin/env python3
"""Render the final Markdown control report to a polished, self-contained PDF."""

from __future__ import annotations

import html
import re
from pathlib import Path

from PIL import Image as PILImage
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    BaseDocTemplate,
    Frame,
    Image,
    KeepTogether,
    ListFlowable,
    ListItem,
    PageBreak,
    PageTemplate,
    Paragraph,
    Preformatted,
    Spacer,
    Table,
    TableStyle,
)


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "docs" / "final_control_and_validation_report_20260719.md"
OUTPUT = ROOT / "output" / "pdf" / "five_tendon_control_final_report_20260719.pdf"

PAGE_W, PAGE_H = A4
LEFT = 18 * mm
RIGHT = 18 * mm
TOP = 18 * mm
BOTTOM = 16 * mm
CONTENT_W = PAGE_W - LEFT - RIGHT


def register_fonts() -> None:
    regular = Path(r"C:\Windows\Fonts\msyh.ttc")
    bold = Path(r"C:\Windows\Fonts\msyhbd.ttc")
    if regular.exists() and bold.exists():
        pdfmetrics.registerFont(TTFont("CJK", str(regular)))
        pdfmetrics.registerFont(TTFont("CJK-Bold", str(bold)))
        pdfmetrics.registerFontFamily(
            "CJK", normal="CJK", bold="CJK-Bold", italic="CJK", boldItalic="CJK-Bold"
        )
    else:
        from reportlab.pdfbase.cidfonts import UnicodeCIDFont

        pdfmetrics.registerFont(UnicodeCIDFont("STSong-Light"))


def font_name(bold: bool = False) -> str:
    names = pdfmetrics.getRegisteredFontNames()
    if "CJK" in names:
        return "CJK-Bold" if bold else "CJK"
    return "STSong-Light"


def inline_markup(text: str) -> str:
    text = html.escape(text, quote=False)
    text = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", text)
    text = re.sub(
        r"`([^`]+)`",
        r'<font name="CJK" color="#17476e">\1</font>',
        text,
    )
    text = re.sub(r"\[([^\]]+)\]\(([^)]+)\)", r'<link href="\2" color="#1769aa">\1</link>', text)
    return text


class NumberedDocTemplate(BaseDocTemplate):
    def __init__(self, filename: str):
        super().__init__(
            filename,
            pagesize=A4,
            leftMargin=LEFT,
            rightMargin=RIGHT,
            topMargin=TOP,
            bottomMargin=BOTTOM,
            title="五腱四关节手指控制系统最终实现与验证报告",
            author="Codex / mmhand project",
        )
        frame = Frame(LEFT, BOTTOM, CONTENT_W, PAGE_H - TOP - BOTTOM, id="normal")
        self.addPageTemplates(PageTemplate(id="main", frames=frame, onPage=self._on_page))

    @staticmethod
    def _on_page(canvas, doc) -> None:
        canvas.saveState()
        canvas.setStrokeColor(colors.HexColor("#d9e2e8"))
        canvas.setLineWidth(0.5)
        canvas.line(LEFT, 12 * mm, PAGE_W - RIGHT, 12 * mm)
        canvas.setFillColor(colors.HexColor("#667681"))
        canvas.setFont(font_name(), 7.5)
        canvas.drawString(LEFT, 7.2 * mm, "五腱四关节控制系统 - 最终实现与验证报告")
        canvas.drawRightString(PAGE_W - RIGHT, 7.2 * mm, f"第 {doc.page} 页")
        canvas.restoreState()


def build_styles():
    base = getSampleStyleSheet()
    body = ParagraphStyle(
        "BodyCJK",
        parent=base["BodyText"],
        fontName=font_name(),
        fontSize=9.2,
        leading=14.2,
        textColor=colors.HexColor("#26343d"),
        alignment=TA_LEFT,
        wordWrap="CJK",
        spaceAfter=2.5 * mm,
    )
    styles = {
        "body": body,
        "title": ParagraphStyle(
            "TitleCJK",
            parent=body,
            fontName=font_name(True),
            fontSize=24,
            leading=31,
            textColor=colors.HexColor("#123d57"),
            alignment=TA_CENTER,
            spaceAfter=10 * mm,
        ),
        "h2": ParagraphStyle(
            "H2CJK",
            parent=body,
            fontName=font_name(True),
            fontSize=15,
            leading=20,
            textColor=colors.HexColor("#126a8f"),
            spaceBefore=5 * mm,
            spaceAfter=3 * mm,
            keepWithNext=True,
        ),
        "h3": ParagraphStyle(
            "H3CJK",
            parent=body,
            fontName=font_name(True),
            fontSize=11.5,
            leading=16,
            textColor=colors.HexColor("#17476e"),
            spaceBefore=3.5 * mm,
            spaceAfter=2 * mm,
            keepWithNext=True,
        ),
        "h4": ParagraphStyle(
            "H4CJK",
            parent=body,
            fontName=font_name(True),
            fontSize=10,
            leading=14,
            textColor=colors.HexColor("#38596c"),
            spaceBefore=2.5 * mm,
            spaceAfter=1.5 * mm,
            keepWithNext=True,
        ),
        "meta": ParagraphStyle(
            "MetaCJK",
            parent=body,
            fontSize=9,
            leading=15,
            textColor=colors.HexColor("#4d6471"),
            leftIndent=10 * mm,
            rightIndent=10 * mm,
            spaceAfter=1 * mm,
        ),
        "caption": ParagraphStyle(
            "CaptionCJK",
            parent=body,
            fontSize=8,
            leading=11,
            textColor=colors.HexColor("#60717b"),
            alignment=TA_CENTER,
            spaceAfter=3 * mm,
        ),
        "code": ParagraphStyle(
            "CodeCJK",
            parent=body,
            fontName=font_name(),
            fontSize=7.4,
            leading=10.2,
            leftIndent=4 * mm,
            rightIndent=4 * mm,
            backColor=colors.HexColor("#f1f5f7"),
            borderColor=colors.HexColor("#d5e1e7"),
            borderWidth=0.5,
            borderPadding=5,
            spaceBefore=1.5 * mm,
            spaceAfter=3 * mm,
        ),
    }
    return styles


def image_flowable(path: Path, caption: str, styles) -> list:
    with PILImage.open(path) as img:
        width, height = img.size
    scale = min(CONTENT_W / width, 118 * mm / height, 1.0)
    flow = Image(str(path), width=width * scale, height=height * scale)
    flow.hAlign = "CENTER"
    return [flow, Paragraph(inline_markup(caption), styles["caption"])]


def table_flowable(rows: list[list[str]], styles) -> Table:
    cells = [[Paragraph(inline_markup(cell.strip()), styles["body"]) for cell in row] for row in rows]
    cols = max(len(row) for row in rows)
    col_width = CONTENT_W / cols
    table = Table(cells, colWidths=[col_width] * cols, repeatRows=1, hAlign="LEFT")
    table.setStyle(
        TableStyle(
            [
                ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#176b87")),
                ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
                ("FONTNAME", (0, 0), (-1, 0), font_name(True)),
                ("BACKGROUND", (0, 1), (-1, -1), colors.HexColor("#f7fafb")),
                ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#f3f7f9")]),
                ("GRID", (0, 0), (-1, -1), 0.35, colors.HexColor("#c9d6dc")),
                ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                ("LEFTPADDING", (0, 0), (-1, -1), 5),
                ("RIGHTPADDING", (0, 0), (-1, -1), 5),
                ("TOPPADDING", (0, 0), (-1, -1), 4),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
            ]
        )
    )
    return table


def parse_markdown(source: Path, styles) -> list:
    lines = source.read_text(encoding="utf-8").splitlines()
    story: list = []
    paragraph: list[str] = []
    list_items: list[str] = []
    numbered = False
    in_code = False
    code_lines: list[str] = []
    title_seen = False

    def flush_paragraph() -> None:
        nonlocal paragraph
        if paragraph:
            story.append(Paragraph(inline_markup(" ".join(x.strip() for x in paragraph)), styles["body"]))
            paragraph = []

    def flush_list() -> None:
        nonlocal list_items, numbered
        if list_items:
            bullets = [ListItem(Paragraph(inline_markup(item), styles["body"]), leftIndent=4 * mm) for item in list_items]
            story.append(
                ListFlowable(
                    bullets,
                    bulletType="1" if numbered else "bullet",
                    leftIndent=7 * mm,
                    bulletFontName=font_name(),
                    bulletFontSize=8,
                    spaceAfter=2 * mm,
                )
            )
            list_items = []
            numbered = False

    i = 0
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()

        if stripped.startswith("```"):
            flush_paragraph()
            flush_list()
            if in_code:
                story.append(Preformatted("\n".join(code_lines), styles["code"])); code_lines = []
                in_code = False
            else:
                in_code = True
            i += 1
            continue
        if in_code:
            code_lines.append(line)
            i += 1
            continue

        image_match = re.fullmatch(r"!\[([^\]]*)\]\(([^)]+)\)", stripped)
        if image_match:
            flush_paragraph(); flush_list()
            rel = image_match.group(2)
            path = (source.parent / rel).resolve()
            if path.exists():
                story.extend(image_flowable(path, image_match.group(1), styles))
            i += 1
            continue

        if stripped.startswith("|") and stripped.endswith("|"):
            flush_paragraph(); flush_list()
            rows = []
            while i < len(lines) and lines[i].strip().startswith("|"):
                row = [x for x in lines[i].strip().strip("|").split("|")]
                if not all(re.fullmatch(r"\s*:?-+:?\s*", cell) for cell in row):
                    rows.append(row)
                i += 1
            story.append(table_flowable(rows, styles))
            story.append(Spacer(1, 3 * mm))
            continue

        heading = re.match(r"^(#{1,4})\s+(.+)$", stripped)
        if heading:
            flush_paragraph(); flush_list()
            level = len(heading.group(1)); text = heading.group(2)
            if level == 1 and not title_seen:
                title_seen = True
                story.append(Spacer(1, 32 * mm))
                story.append(Paragraph(inline_markup(text), styles["title"]))
            else:
                story.append(Paragraph(inline_markup(text), styles[f"h{min(level,4)}"]))
            i += 1
            continue

        if stripped == "---":
            flush_paragraph(); flush_list()
            story.append(Spacer(1, 3 * mm))
            i += 1
            continue

        bullet = re.match(r"^[-*]\s+(.+)$", stripped)
        number = re.match(r"^\d+\.\s+(.+)$", stripped)
        if bullet or number:
            flush_paragraph()
            this_numbered = bool(number)
            if list_items and numbered != this_numbered:
                flush_list()
            numbered = this_numbered
            list_items.append((number or bullet).group(1))
            i += 1
            continue

        if not stripped:
            flush_paragraph(); flush_list()
            i += 1
            continue

        if title_seen and len(story) <= 6 and stripped.startswith("**"):
            flush_paragraph(); flush_list()
            story.append(Paragraph(inline_markup(stripped), styles["meta"]))
            i += 1
            continue

        paragraph.append(line)
        i += 1

    flush_paragraph(); flush_list()
    return story


def main() -> None:
    register_fonts()
    styles = build_styles()
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    doc = NumberedDocTemplate(str(OUTPUT))
    story = parse_markdown(SOURCE, styles)
    doc.build(story)
    print(OUTPUT)


if __name__ == "__main__":
    main()
