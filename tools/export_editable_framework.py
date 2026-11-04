#!/usr/bin/env python3
"""将现有图1的HTML文字、方框和连线导出为可编辑PPTX和文本SVG。"""
from html import escape, unescape
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_CONNECTOR, MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.oxml.xmlchemy import OxmlElement
from pptx.util import Inches, Pt

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / 'docs/报告/20261023_协同定位阶段报告/figures'


def main():
    html = (ROOT / 'docs/visualization/system_framework.html').read_text()
    boxes = []
    for number, content in re.findall(r'<div class="box b(\d+)">(.*?)</div>', html):
        css = re.search(r'\.b' + number + r'\{([^}]+)\}', html)[1]
        geometry = {k: int(v) for k, v in re.findall(r'(left|top|width|height):(\d+)px', css)}
        text = unescape(re.sub(r'<br\s*/?>', '\n', content))
        boxes.append((geometry, text))
    paths = re.findall(r'<path class="line" d="([^"]+)"', html)
    assert len(boxes) == 6 and len(paths) == 5, '图1结构变化，请核对导出规则'
    DEST.mkdir(parents=True, exist_ok=True)

    # SVG保留真实文本节点，文字和方框可在矢量绘图软件中分别选择。
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1600" height="900" viewBox="0 0 1600 900">',
           '<rect width="1600" height="900" fill="white"/>',
           '<defs><marker id="arrow" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="8" markerHeight="8" orient="auto"><path d="M0 0L10 5L0 10Z" fill="#59636c"/></marker></defs>']
    for i, (g, text) in enumerate(boxes, 1):
        x, y, w, h = (g[k] for k in ('left', 'top', 'width', 'height'))
        lines = text.splitlines()
        svg.append(f'<g id="box_{i}"><rect x="{x}" y="{y}" width="{w}" height="{h}" fill="#eef2f5" stroke="#59636c" stroke-width="3"/>')
        for j, line in enumerate(lines):
            baseline = y + h / 2 + (j - (len(lines) - 1) / 2) * 41.6 + 9
            svg.append(f'<text x="{x+w/2}" y="{baseline}" text-anchor="middle" font-family="Times New Roman,SimHei,Noto Sans CJK SC,sans-serif" font-size="26">{escape(line)}</text>')
        svg.append('</g>')
    for i, path in enumerate(paths, 1):
        svg.append(f'<path id="connection_{i}" d="{path}" stroke="#59636c" stroke-width="3" fill="none" marker-end="url(#arrow)"/>')
    svg.append('</svg>')
    svg_text = '\n'.join(svg) + '\n'
    (DEST / '图1_协同定位系统组成_可编辑.svg').write_text(svg_text)
    (DEST / '01_系统与联调边界.svg').write_text(svg_text)

    prs = Presentation()
    prs.slide_width, prs.slide_height = Inches(1600/120), Inches(900/120)
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    slide.background.fill.solid(); slide.background.fill.fore_color.rgb = RGBColor(255,255,255)
    scale = lambda value: Inches(value/120)
    for i, (g, text) in enumerate(boxes, 1):
        shape = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, *(scale(g[k]) for k in ('left','top','width','height')))
        shape.name = f'模块{i}：' + text.splitlines()[0]
        for effect in shape._element.xpath('.//a:effectRef'):
            effect.set('idx', '0')
        shape.fill.solid(); shape.fill.fore_color.rgb = RGBColor.from_string('EEF2F5')
        shape.line.color.rgb = RGBColor.from_string('59636C'); shape.line.width = Pt(1.8)
        tf = shape.text_frame; tf.clear(); tf.word_wrap = True; tf.vertical_anchor = MSO_ANCHOR.MIDDLE
        tf.margin_left = tf.margin_right = scale(12)
        for j, line in enumerate(text.splitlines()):
            p = tf.paragraphs[0] if j == 0 else tf.add_paragraph()
            p.alignment = PP_ALIGN.CENTER; p.space_before = p.space_after = Pt(0); p.line_spacing = 1.6
            r = p.add_run(); r.text = line; r.font.name = 'Times New Roman'; r.font.size = Pt(15.6)
            r.font.color.rgb = RGBColor(17,17,17)
            ea = OxmlElement('a:ea'); ea.set('typeface','黑体'); r._r.get_or_add_rPr().append(ea)
    # 原图只有绝对坐标M/H/V/L折线，各段保留为PPT原生可编辑连线。
    segment_count = 0
    for i, path in enumerate(paths, 1):
        tokens = re.findall(r'[MHVL]|-?\d+(?:\.\d+)?', path)
        points=[]; pos=0; x=y=0
        while pos < len(tokens):
            command=tokens[pos]; pos+=1
            if command in ('M','L'):
                x,y=float(tokens[pos]),float(tokens[pos+1]);pos+=2
            elif command=='H': x=float(tokens[pos]);pos+=1
            elif command=='V': y=float(tokens[pos]);pos+=1
            points.append((x,y))
        for j,(start,end) in enumerate(zip(points,points[1:])):
            line=slide.shapes.add_connector(MSO_CONNECTOR.STRAIGHT,scale(start[0]),scale(start[1]),scale(end[0]),scale(end[1]))
            line.name=f'连线{i}第{j+1}段'; line.line.color.rgb=RGBColor.from_string('59636C'); line.line.width=Pt(1.8)
            if j==len(points)-2:
                arrow=OxmlElement('a:tailEnd');arrow.set('type','triangle');line.line._get_or_add_ln().append(arrow)
            segment_count+=1
    slide.notes_slide.notes_text_frame.text = '图1可编辑源文件。六个方框内文字和各段连线均为PowerPoint原生对象。修改后导出PNG，替换报告figures/01_系统与联调边界.png，再运行render_formal_report.py和export_formal_pdf.py。'
    target=DEST/'图1_协同定位系统组成_可编辑.pptx';prs.save(target)
    check=Presentation(target)
    assert sum(shape.has_text_frame for shape in check.slides[0].shapes)==6
    assert len(check.slides[0].shapes)==6+segment_count
    assert len(ET.fromstring(svg_text).findall('.//{http://www.w3.org/2000/svg}text'))==sum(len(text.splitlines()) for _,text in boxes)
    print(f'已生成：{target}；6个可编辑文本方框、{segment_count}段原生连线；另附文本SVG。')


if __name__ == '__main__':
    main()
