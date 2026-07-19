#!/usr/bin/env python3
"""Render the final control theory and experiment Markdown reports to PDF."""

from __future__ import annotations

import argparse
from html import escape
from pathlib import Path
import re

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    BaseDocTemplate,
    Frame,
    Image,
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
from reportlab.platypus.tableofcontents import TableOfContents


FONT_REGULAR = "MicrosoftYaHei"
FONT_BOLD = "MicrosoftYaHeiBold"
PAGE_W, PAGE_H = A4


def register_fonts() -> None:
    regular = Path(r"C:\Windows\Fonts\msyh.ttc")
    bold = Path(r"C:\Windows\Fonts\msyhbd.ttc")
    if not regular.exists() or not bold.exists():
        raise FileNotFoundError("Microsoft YaHei fonts are required for Chinese PDF output")
    pdfmetrics.registerFont(TTFont(FONT_REGULAR, str(regular)))
    pdfmetrics.registerFont(TTFont(FONT_BOLD, str(bold)))


def inline_markup(text: str) -> str:
    value = escape(text.strip())
    value = re.sub(r"\*\*(.+?)\*\*", r"<b>\1</b>", value)
    value = re.sub(r"`([^`]+)`", rf'<font name="{FONT_REGULAR}">\1</font>', value)
    return value


class FinalReportDoc(BaseDocTemplate):
    def __init__(self, filename: str, *, report_title: str):
        super().__init__(
            filename,
            pagesize=A4,
            leftMargin=18 * mm,
            rightMargin=18 * mm,
            topMargin=20 * mm,
            bottomMargin=18 * mm,
            title=report_title,
            author="mmhand control validation",
            subject="Five-tendon four-joint control",
        )
        self.report_title = report_title
        frame = Frame(
            self.leftMargin,
            self.bottomMargin,
            self.width,
            self.height,
            id="normal",
        )
        self.addPageTemplates(PageTemplate(id="report", frames=[frame], onPage=self._draw_page))

    def _draw_page(self, canvas, doc) -> None:
        canvas.saveState()
        if doc.page > 1:
            canvas.setStrokeColor(colors.HexColor("#B8C1CC"))
            canvas.setLineWidth(0.45)
            canvas.line(18 * mm, PAGE_H - 13 * mm, PAGE_W - 18 * mm, PAGE_H - 13 * mm)
            canvas.setFont(FONT_REGULAR, 8)
            canvas.setFillColor(colors.HexColor("#56606B"))
            canvas.drawString(18 * mm, PAGE_H - 10.5 * mm, self.report_title[:31])
            canvas.drawRightString(PAGE_W - 18 * mm, 10.5 * mm, f"第 {doc.page - 1} 页")
        canvas.restoreState()

    def afterFlowable(self, flowable) -> None:
        if isinstance(flowable, Paragraph):
            style_name = flowable.style.name
            if style_name in ("H1", "H2", "H3"):
                level = {"H1": 0, "H2": 1, "H3": 2}[style_name]
                text = flowable.getPlainText()
                key = f"section-{self.seq.nextf('section')}"
                self.canv.bookmarkPage(key)
                self.canv.addOutlineEntry(text, key, level=level, closed=False)
                self.notify("TOCEntry", (level, text, self.page - 1, key))


def build_styles() -> dict[str, ParagraphStyle]:
    base = getSampleStyleSheet()
    return {
        "cover_title": ParagraphStyle(
            "CoverTitle",
            parent=base["Title"],
            fontName=FONT_BOLD,
            fontSize=25,
            leading=35,
            alignment=TA_CENTER,
            textColor=colors.HexColor("#17365D"),
            spaceAfter=16,
        ),
        "cover_subtitle": ParagraphStyle(
            "CoverSubtitle",
            parent=base["Normal"],
            fontName=FONT_REGULAR,
            fontSize=12,
            leading=21,
            alignment=TA_CENTER,
            textColor=colors.HexColor("#56606B"),
        ),
        "H1": ParagraphStyle(
            "H1",
            parent=base["Heading1"],
            fontName=FONT_BOLD,
            fontSize=17,
            leading=24,
            textColor=colors.HexColor("#17365D"),
            spaceBefore=10,
            spaceAfter=8,
            keepWithNext=True,
        ),
        "H2": ParagraphStyle(
            "H2",
            parent=base["Heading2"],
            fontName=FONT_BOLD,
            fontSize=13.5,
            leading=20,
            textColor=colors.HexColor("#244F75"),
            spaceBefore=9,
            spaceAfter=6,
            keepWithNext=True,
        ),
        "H3": ParagraphStyle(
            "H3",
            parent=base["Heading3"],
            fontName=FONT_BOLD,
            fontSize=11.5,
            leading=17,
            textColor=colors.HexColor("#315E83"),
            spaceBefore=7,
            spaceAfter=4,
            keepWithNext=True,
        ),
        "body": ParagraphStyle(
            "Body",
            parent=base["BodyText"],
            fontName=FONT_REGULAR,
            fontSize=9.6,
            leading=16.2,
            alignment=TA_JUSTIFY,
            firstLineIndent=2 * 9.6,
            textColor=colors.HexColor("#20252A"),
            wordWrap="CJK",
            spaceAfter=5,
        ),
        "quote": ParagraphStyle(
            "Quote",
            parent=base["BodyText"],
            fontName=FONT_REGULAR,
            fontSize=9.5,
            leading=16,
            leftIndent=8 * mm,
            rightIndent=8 * mm,
            borderColor=colors.HexColor("#8AA6BF"),
            borderWidth=0.6,
            borderPadding=7,
            backColor=colors.HexColor("#F3F7FA"),
            wordWrap="CJK",
            spaceBefore=5,
            spaceAfter=7,
        ),
        "code": ParagraphStyle(
            "Code",
            parent=base["Code"],
            fontName=FONT_REGULAR,
            fontSize=7.8,
            leading=12.0,
            leftIndent=4 * mm,
            rightIndent=4 * mm,
            borderColor=colors.HexColor("#CCD5DE"),
            borderWidth=0.5,
            borderPadding=7,
            backColor=colors.HexColor("#F7F8FA"),
            textColor=colors.HexColor("#26313B"),
            spaceBefore=4,
            spaceAfter=7,
        ),
        "bullet": ParagraphStyle(
            "Bullet",
            parent=base["BodyText"],
            fontName=FONT_REGULAR,
            fontSize=9.4,
            leading=15.5,
            wordWrap="CJK",
        ),
        "caption": ParagraphStyle(
            "Caption",
            parent=base["BodyText"],
            fontName=FONT_REGULAR,
            fontSize=8.2,
            leading=12,
            alignment=TA_CENTER,
            textColor=colors.HexColor("#56606B"),
            spaceAfter=7,
        ),
        "toc_title": ParagraphStyle(
            "TocTitle",
            parent=base["Heading1"],
            fontName=FONT_BOLD,
            fontSize=18,
            leading=25,
            alignment=TA_CENTER,
            textColor=colors.HexColor("#17365D"),
            spaceAfter=15,
        ),
        "table": ParagraphStyle(
            "TableCell",
            parent=base["BodyText"],
            fontName=FONT_REGULAR,
            fontSize=8.2,
            leading=11.5,
            alignment=TA_LEFT,
            wordWrap="CJK",
        ),
        "table_head": ParagraphStyle(
            "TableHead",
            parent=base["BodyText"],
            fontName=FONT_BOLD,
            fontSize=8.3,
            leading=11.5,
            alignment=TA_CENTER,
            textColor=colors.white,
            wordWrap="CJK",
        ),
    }


def scaled_image(path: Path, max_width: float, max_height: float) -> Image:
    image = Image(str(path))
    scale = min(max_width / image.imageWidth, max_height / image.imageHeight, 1.0)
    image.drawWidth = image.imageWidth * scale
    image.drawHeight = image.imageHeight * scale
    image.hAlign = "CENTER"
    return image


def markdown_story(path: Path, styles: dict[str, ParagraphStyle]) -> tuple[str, list]:
    lines = path.read_text(encoding="utf-8").splitlines()
    title = lines[0].lstrip("# ").strip()
    story: list = []
    index = 1
    paragraph_lines: list[str] = []

    def flush_paragraph() -> None:
        nonlocal paragraph_lines
        if paragraph_lines:
            story.append(Paragraph(inline_markup(" ".join(paragraph_lines)), styles["body"]))
            paragraph_lines = []

    while index < len(lines):
        line = lines[index]
        stripped = line.strip()
        if not stripped:
            flush_paragraph()
            index += 1
            continue
        if stripped.startswith("```"):
            flush_paragraph()
            code_lines: list[str] = []
            index += 1
            while index < len(lines) and not lines[index].strip().startswith("```"):
                code_lines.append(lines[index])
                index += 1
            story.append(Preformatted("\n".join(code_lines), styles["code"]))
            index += 1
            continue
        heading_match = re.match(r"^(#{2,4})\s+(.+)$", stripped)
        if heading_match:
            flush_paragraph()
            level = min(len(heading_match.group(1)) - 1, 3)
            story.append(Paragraph(inline_markup(heading_match.group(2)), styles[f"H{level}"]))
            index += 1
            continue
        if stripped.startswith("| ") or (stripped.startswith("|") and stripped.endswith("|")):
            flush_paragraph()
            table_lines: list[str] = []
            while index < len(lines):
                current = lines[index].strip()
                if not (current.startswith("|") and current.endswith("|")):
                    break
                table_lines.append(current)
                index += 1
            raw_rows = [[cell.strip() for cell in row.strip("|").split("|")] for row in table_lines]
            raw_rows = [
                row
                for row in raw_rows
                if not all(re.fullmatch(r":?-{3,}:?", cell.replace(" ", "")) for cell in row)
            ]
            cell_rows = []
            for row_index, row in enumerate(raw_rows):
                style = styles["table_head"] if row_index == 0 else styles["table"]
                cell_rows.append([Paragraph(inline_markup(cell), style) for cell in row])
            if cell_rows:
                table = Table(cell_rows, repeatRows=1, hAlign="CENTER")
                table.setStyle(
                    TableStyle(
                        [
                            ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#315E83")),
                            ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
                            ("GRID", (0, 0), (-1, -1), 0.35, colors.HexColor("#B8C1CC")),
                            ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
                            ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#F5F7F9")]),
                            ("LEFTPADDING", (0, 0), (-1, -1), 5),
                            ("RIGHTPADDING", (0, 0), (-1, -1), 5),
                            ("TOPPADDING", (0, 0), (-1, -1), 5),
                            ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
                        ]
                    )
                )
                story.extend([table, Spacer(1, 6)])
            continue
        image_match = re.match(r"^!\[(.*?)\]\((.*?)\)$", stripped)
        if image_match:
            flush_paragraph()
            image_path = (path.parent / image_match.group(2)).resolve()
            if not image_path.exists():
                raise FileNotFoundError(f"report image not found: {image_path}")
            story.append(scaled_image(image_path, 172 * mm, 110 * mm))
            story.append(Paragraph(inline_markup(image_match.group(1)), styles["caption"]))
            index += 1
            continue
        if stripped.startswith(">"):
            flush_paragraph()
            story.append(Paragraph(inline_markup(stripped.lstrip("> ")), styles["quote"]))
            index += 1
            continue
        if re.match(r"^[-*]\s+", stripped) or re.match(r"^\d+\.\s+", stripped):
            flush_paragraph()
            ordered = bool(re.match(r"^\d+\.\s+", stripped))
            items: list[ListItem] = []
            while index < len(lines):
                current = lines[index].strip()
                match = re.match(r"^(?:[-*]|\d+\.)\s+(.+)$", current)
                if not match:
                    break
                items.append(ListItem(Paragraph(inline_markup(match.group(1)), styles["bullet"])))
                index += 1
            story.append(
                ListFlowable(
                    items,
                    bulletType="1" if ordered else "bullet",
                    start="1",
                    leftIndent=16,
                    bulletFontName=FONT_REGULAR,
                    bulletFontSize=8,
                    spaceAfter=6,
                )
            )
            continue
        paragraph_lines.append(stripped)
        index += 1
    flush_paragraph()
    return title, story


def render_report(markdown: Path, destination: Path) -> None:
    styles = build_styles()
    title, body = markdown_story(markdown, styles)
    destination.parent.mkdir(parents=True, exist_ok=True)
    doc = FinalReportDoc(str(destination), report_title=title)

    toc = TableOfContents()
    toc.levelStyles = [
        ParagraphStyle(
            "TOC0",
            fontName=FONT_BOLD,
            fontSize=10.2,
            leading=16,
            leftIndent=0,
            textColor=colors.HexColor("#17365D"),
        ),
        ParagraphStyle(
            "TOC1",
            fontName=FONT_REGULAR,
            fontSize=9.4,
            leading=15,
            leftIndent=12,
            textColor=colors.HexColor("#26313B"),
        ),
        ParagraphStyle(
            "TOC2",
            fontName=FONT_REGULAR,
            fontSize=8.7,
            leading=14,
            leftIndent=24,
            textColor=colors.HexColor("#56606B"),
        ),
    ]

    cover = [
        Spacer(1, 48 * mm),
        Paragraph(inline_markup(title), styles["cover_title"]),
        Spacer(1, 10 * mm),
        Paragraph("五腱绳四自由度手指控制项目", styles["cover_subtitle"]),
        Paragraph("最终控制框架与实验归档", styles["cover_subtitle"]),
        Spacer(1, 24 * mm),
        Paragraph("版本日期：2026-07-19", styles["cover_subtitle"]),
        Paragraph("分支：codex/five-tendon", styles["cover_subtitle"]),
        PageBreak(),
        Paragraph("目录", styles["toc_title"]),
        toc,
        PageBreak(),
    ]
    doc.multiBuild(cover + body)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--theory-md", type=Path, required=True)
    parser.add_argument("--experiment-md", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    register_fonts()
    render_report(args.theory_md, args.output_dir / "five_tendon_control_theory_final.pdf")
    render_report(args.experiment_md, args.output_dir / "fd_threshold_and_validation_final.pdf")
    print(args.output_dir / "five_tendon_control_theory_final.pdf")
    print(args.output_dir / "fd_threshold_and_validation_final.pdf")


if __name__ == "__main__":
    main()
