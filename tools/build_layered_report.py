#!/usr/bin/env python3
"""兼容旧命令：当前报告已统一为一册正式报告。"""

from build_formal_report import build_formal_report


def build_layered_reports():
    return build_formal_report()


if __name__ == '__main__':
    build_formal_report()
