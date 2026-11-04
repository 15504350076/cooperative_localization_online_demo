"""报告图线采用简体中文字形，避免Matplotlib把CJK字体集首项当作中文。"""
from pathlib import Path

import matplotlib
from matplotlib import font_manager as fm


def chinese_font_family():
    # 文泉驿微米黑的首个字面就是简体中文，且TrueType可嵌入PDF并保留文字。
    # Noto CJK TTC的首项是JP；仅指定SC家族名不能修复旧版Matplotlib的加载行为。
    for filename in ('/usr/share/fonts/truetype/msttcorefonts/simhei.ttf',
                     '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc'):
        if Path(filename).is_file():
            if filename.endswith('.ttc'):
                # Matplotlib 3.5的PDF子集工具不能直接嵌入TTC，提取正确的中文首项。
                from fontTools.ttLib import TTFont
                cache = Path(matplotlib.get_cachedir()) / 'report-wqy-microhei.ttf'
                if not cache.exists() or cache.stat().st_mtime < Path(filename).stat().st_mtime:
                    with TTFont(filename, fontNumber=0) as font:
                        font.save(cache)
                filename = str(cache)
            family = fm.FontProperties(fname=filename).get_name()
            fm.fontManager.ttflist = [font for font in fm.fontManager.ttflist if font.name != family]
            fm.fontManager.addfont(filename)
            return family
    raise RuntimeError('缺少报告简体中文字体，请安装fonts-wqy-microhei或提供SimHei字体')


def numeric_ticks(fig):
    """纯数字刻度用新罗马；未安装时使用其度量兼容字体Liberation Serif。"""
    latin = fm.findfont(fm.FontProperties(family=['Times New Roman', 'Liberation Serif']))
    for ax in fig.axes:
        for axis, labels in ((ax.xaxis, ax.get_xticklabels()), (ax.yaxis, ax.get_yticklabels())):
            # 分类轴包含空白占位刻度，后续绘制会复用它们；整条分类轴保留中文字体。
            if any(not label.get_text().isascii() for label in labels):
                continue
            for label in labels + [axis.get_offset_text()]:
                if label.get_text():
                    label.set_fontproperties(fm.FontProperties(fname=latin, size=label.get_fontsize()))
