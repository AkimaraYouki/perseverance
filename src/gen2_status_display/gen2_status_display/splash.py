"""Boot animation (black & white, ~3.5 s). frame(t) is pure: same t -> same image."""
from PIL import Image, ImageDraw

from .render import W, H, _font, _FB, _MB

F_TITLE = _font(_FB, 26)
F_SUB = _font(_FB, 13)
F_BY = _font(_MB, 12)
TITLE = 'C-WANG'
DURATION_S = 3.6


def _centre(d, y, text, font, fill):
    d.text(((W - d.textlength(text, font=font)) / 2, y), text, font=font, fill=fill)


def _ease(x):
    x = min(1.0, max(0.0, x))
    return x * x * (3 - 2 * x)


def frame(t):
    img = Image.new('RGB', (W, H), (0, 0, 0))
    d = ImageDraw.Draw(img)
    # 2) title types in letter by letter with a block cursor (0.4-1.6 s)
    n = int(len(TITLE) * min(1.0, max(0.0, (t - 0.4) / 1.2)))
    tw = d.textlength(TITLE, font=F_TITLE)
    tx = (W - tw) / 2
    shown = TITLE[:n]
    d.text((tx, 62), shown, font=F_TITLE, fill=(255, 255, 255))
    if 0.4 <= t < 2.0 and int(t * 6) % 2 == 0:
        cx = tx + d.textlength(shown, font=F_TITLE) + 2
        d.rectangle([cx, 66, cx + 12, 92], fill=(255, 255, 255))
    # 3) underline grows from the centre (1.6-2.0 s)
    u = _ease((t - 1.6) / 0.4)
    if u > 0:
        half = (W - 120) / 2 * u
        d.line([W / 2 - half, 102, W / 2 + half, 102], fill=(255, 255, 255))
    # 4) institute fades in (1.9-2.5 s)
    g = int(255 * _ease((t - 1.9) / 0.6))
    if g > 0:
        _centre(d, 114, 'Kumoh National Institute', F_SUB, (g, g, g))
        _centre(d, 132, 'of Technology', F_SUB, (g, g, g))
    # 5) author + progress bar (2.4-3.6 s)
    g2 = int(170 * _ease((t - 2.4) / 0.4))
    if g2 > 0:
        _centre(d, 164, 'by suhopark', F_BY, (g2, g2, g2))
    p = _ease((t - 2.5) / 1.0)
    if t > 2.5:
        bx0, bx1, by = 90, W - 90, 196
        d.rectangle([bx0, by, bx1, by + 5], outline=(120, 120, 120))
        d.rectangle([bx0 + 1, by + 1, bx0 + 1 + (bx1 - bx0 - 2) * p, by + 4], fill=(255, 255, 255))
    return img
