#!/usr/bin/env python3
"""将手工修改后的正式版 Markdown 排版为 Word；不重建或覆盖 Markdown。"""
from pathlib import Path

from build_stage_report import DEST, render_word


MARKDOWN = DEST / '多平台协同定位硬件仿真测试报告与联调方案_V1.3_正式报告版.md'
WORD = MARKDOWN.with_suffix('.docx')


def main():
    render_word(
        MARKDOWN.read_text(),
        WORD,
        title='多平台协同定位硬件仿真测试报告与联调方案',
        header='',
    )
    print(f'已生成：{WORD}')


if __name__ == '__main__':
    main()
