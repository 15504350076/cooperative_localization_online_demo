#!/usr/bin/env python3
"""将手工修改后的正式版 Markdown 排版为 Word；不重建或覆盖 Markdown。"""
import argparse
from pathlib import Path

from build_stage_report import DEST, render_word


MARKDOWN = DEST.parent / '20261028_协同定位V1.4/多平台协同定位硬件仿真测试报告与联调方案_V1.4_设备分级与锚点仿真版.md'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--markdown',type=Path,default=MARKDOWN)
    args=parser.parse_args()
    word=args.markdown.with_suffix('.docx')
    render_word(
        args.markdown.read_text(),
        word,
        title='多平台协同定位硬件仿真测试报告与联调方案',
        header='',
    )
    print(f'已生成：{word}')


if __name__ == '__main__':
    main()
