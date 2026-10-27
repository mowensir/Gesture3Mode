# -*- coding: utf-8 -*-
"""
draw.io 导出的 SVG 瘦身脚本。

原因：draw.io 导出 SVG 时，每个 HTML 文字标签（foreignObject）都会附带一张
4 倍尺寸的 base64 PNG 兜底图（<switch> 的 <image>），导致单文件 1~2MB。

处理：把每张兜底图重采样为 2 倍显示尺寸（Retina 标准）；
单色文字标签转为"透明+单色"二值图，多色标签退回 16 色调色板。
体积约降为原来的 1/13，浏览器中的矢量文字(foreignObject)不受影响。

用法（每次用 draw.io 重新导出 SVG 后运行一次）：
  python shrink_svg.py circuit.svg circuit_internal.svg bluetooth_mindmap.svg arduino_freertos_flow.svg
依赖：pip install pillow
"""
import re, sys, base64, io
from collections import Counter
from PIL import Image

PAT = re.compile(r'(<image x="[\d.]+" y="[\d.]+" width="[\d.]+" height="[\d.]+" '
                 r'xlink:href="data:image/png;base64,)([A-Za-z0-9+/=]+)(")')


def shrink(m):
    raw = base64.b64decode(m.group(2))
    im = Image.open(io.BytesIO(raw)).convert('RGBA')
    tag = m.group(1)
    dw = float(re.search(r'width="([\d.]+)"', tag).group(1))
    dh = float(re.search(r'height="([\d.]+)"', tag).group(1))
    tw, th = max(1, round(dw * 2)), max(1, round(dh * 2))
    if im.width > tw:
        im = im.resize((tw, th), Image.LANCZOS)
    px = im.load()
    cnt = Counter()
    for yy in range(im.height):
        for xx in range(im.width):
            r, g, b, a = px[xx, yy]
            if a > 200:
                cnt[(r, g, b)] += 1
    out = None
    if cnt:
        ink, n1 = cnt.most_common(1)[0]
        far = sum(n for c, n in cnt.items()
                  if max(abs(c[0] - ink[0]), abs(c[1] - ink[1]), abs(c[2] - ink[2])) > 24)
        if far < n1 * 0.05:  # 单色标签 → 二值化
            bilevel = Image.new('RGBA', im.size, (0, 0, 0, 0))
            bp = bilevel.load()
            for yy in range(im.height):
                for xx in range(im.width):
                    if px[xx, yy][3] >= 110:
                        bp[xx, yy] = (*ink, 255)
            out = bilevel.quantize(2, method=Image.FASTOCTREE)
    if out is None:  # 多色标签 → 16 色调色板
        out = im.quantize(16, method=Image.FASTOCTREE)
    buf = io.BytesIO()
    out.save(buf, 'PNG', optimize=True)
    return m.group(1) + base64.b64encode(buf.getvalue()).decode() + m.group(3)


if __name__ == '__main__':
    for f in sys.argv[1:]:
        s = open(f, encoding='utf-8').read()
        before = len(s)
        s2, n = PAT.subn(shrink, s)
        open(f, 'w', encoding='utf-8').write(s2)
        print(f'{f}: {before:,} -> {len(s2):,} B  ({n} 张兜底图)')
