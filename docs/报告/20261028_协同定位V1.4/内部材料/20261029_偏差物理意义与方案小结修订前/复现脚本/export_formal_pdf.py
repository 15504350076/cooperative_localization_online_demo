#!/usr/bin/env python3
"""从正式版 Word 导出可读取的 PDF。

每次使用独立的 LibreOffice 用户配置，避免旧配置中的字体缓存导致 PDF
出现空白页或损坏的嵌入字体流。Word 源文件保持不变。
"""
from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REPORT_DIR = ROOT / "docs/报告/20261028_协同定位V1.4"
DEFAULT_DOCX = REPORT_DIR / "多平台协同定位硬件仿真测试报告与联调方案_V1.4_设备分级与锚点仿真版.docx"
DEFAULT_PDF = DEFAULT_DOCX.with_suffix('.pdf')


def add_toc_links(source: Path, destination: Path) -> int:
    """按PDF目录文字的实际位置建立内部链接，目标取自导出文件的章节书签。"""
    try:
        from pypdf import PdfReader, PdfWriter
        from pypdf.annotations import Link
    except ModuleNotFoundError:
        sys.path.append('/tmp/cooperative_report_deps')
        try:
            from pypdf import PdfReader, PdfWriter
            from pypdf.annotations import Link
        except ModuleNotFoundError as exc:
            raise RuntimeError('生成PDF目录链接需要pypdf：python3 -m pip install pypdf') from exc

    reader = PdfReader(source)
    roots = [item for item in reader.outline if not isinstance(item, list)]
    toc = next((item for item in roots if item.title == '目录'), None)
    chapters = [item for item in roots if re.match(r'^\d+\s+', item.title) or item.title.startswith('附录 ')]
    if toc is None or not chapters:
        raise RuntimeError('PDF缺少目录或章节书签，无法建立目录链接')
    toc_page = reader.get_destination_page_number(toc)
    bbox = subprocess.run(['pdftotext', '-f', str(toc_page + 1), '-l', str(toc_page + 1),
                           '-bbox-layout', str(source), '-'], check=True, capture_output=True).stdout
    root = ET.fromstring(bbox)
    ns = {'h': 'http://www.w3.org/1999/xhtml'}
    page = root.find('.//h:page', ns)
    if page is None:
        raise RuntimeError('无法读取PDF目录页的位置')
    height = float(page.attrib['height'])
    lines = page.findall('.//h:line', ns)
    writer = PdfWriter(clone_from=reader)
    first_chapter = next(item for item in chapters if re.match(r'^1\s+', item.title))
    body_start = reader.get_destination_page_number(first_chapter)
    if body_start:
        writer.set_page_label(0, body_start - 1, prefix='')
    writer.set_page_label(body_start, len(reader.pages) - 1, style='/D', start=1)
    for chapter in chapters:
        title = re.sub(r'^\d+\s+|^附录\s+', '', chapter.title)
        matches = [line for line in lines if ''.join(line.itertext()).strip() == title]
        if len(matches) != 1:
            raise RuntimeError(f'PDF目录标题未唯一匹配：{title}')
        line = matches[0]
        number = re.match(r'^(\d+|附录)', chapter.title).group(1)
        number_lines = [item for item in lines if ''.join(item.itertext()).strip() == number
                        and abs(float(item.attrib['yMin']) - float(line.attrib['yMin'])) < 3]
        left = min([float(line.attrib['xMin'])] + [float(item.attrib['xMin']) for item in number_lines])
        rect = (left - 3, height - float(line.attrib['yMax']) - 2,
                float(line.attrib['xMax']) + 3, height - float(line.attrib['yMin']) + 2)
        writer.add_annotation(toc_page, Link(rect=rect,
                              target_page_index=reader.get_destination_page_number(chapter)))
    with destination.open('wb') as stream:
        writer.write(stream)
    return len(chapters)


def export_pdf(docx: Path, pdf: Path) -> Path:
    docx = docx.resolve()
    pdf = pdf.resolve()
    if not docx.is_file():
        raise FileNotFoundError(f"Word源文件不存在：{docx}")

    with tempfile.TemporaryDirectory(prefix="cooperative_report_lo_") as work:
        work_dir = Path(work)
        profile = work_dir / "profile"
        # LibreOffice 7.3支持的本地PDF导出配置：曲线和文字保持无损，不转JPEG。
        user_profile = profile / 'user'
        user_profile.mkdir(parents=True)
        (user_profile / 'registrymodifications.xcu').write_text('''<?xml version="1.0" encoding="UTF-8"?>
<oor:items xmlns:oor="http://openoffice.org/2001/registry">
<item oor:path="/org.openoffice.Office.Common/Filter/PDF/Export">
<prop oor:name="UseLosslessCompression" oor:op="fuse"><value>true</value></prop>
<prop oor:name="ReduceImageResolution" oor:op="fuse"><value>false</value></prop>
</item></oor:items>''', encoding='utf-8')
        output = work_dir / "pdf"
        output.mkdir()
        command = [
            "libreoffice",
            f"-env:UserInstallation=file://{profile}",
            "--headless",
            "--convert-to",
            "pdf:writer_pdf_Export",
            "--outdir",
            str(output),
            str(docx),
        ]
        subprocess.run(command, check=True)
        generated = output / f"{docx.stem}.pdf"
        if not generated.is_file():
            raise RuntimeError(f"LibreOffice未生成PDF：{generated}")
        linked = work_dir / 'with_toc_links.pdf'
        count = add_toc_links(generated, linked)
        pdf.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(linked, pdf)
        print(f'已建立 {count} 个PDF目录跳转链接')
    return pdf


def main() -> None:
    parser = argparse.ArgumentParser(description="使用独立LibreOffice配置导出正式报告PDF")
    parser.add_argument("--docx", type=Path, default=DEFAULT_DOCX)
    parser.add_argument("--pdf", type=Path, default=DEFAULT_PDF)
    args = parser.parse_args()
    result = export_pdf(args.docx, args.pdf)
    print(f"已生成：{result}")


if __name__ == "__main__":
    main()
