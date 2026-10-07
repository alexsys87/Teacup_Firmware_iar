"""Draws the Teacup Host icon: AppIcon.ico (16…256 px) and AppIcon.png.

Usage: python3 make_icon.py [output directory]   (needs Pillow)
"""
import math, os, sys
from PIL import Image, ImageDraw, ImageFilter

N = 1024
out = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))

def lerp(a, b, t): return tuple(int(a[i] + (b[i] - a[i]) * t) for i in range(len(a)))
def hexc(h, a=255): h = h.lstrip('#'); return (int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16), a)

img = Image.new('RGBA', (N, N), (0, 0, 0, 0))

# Background: rounded square, diagonal gradient blue → violet.
grad = Image.new('RGBA', (N, N))
gp = grad.load()
c0, c1 = hexc('#1E88E5'), hexc('#6D28D9')
for y in range(N):
    for x in range(N):
        t = (x * 0.35 + y * 0.65) / N
        gp[x, y] = lerp(c0, c1, min(1, max(0, t)))
mask = Image.new('L', (N, N), 0)
m = 40
ImageDraw.Draw(mask).rounded_rectangle((m, m, N - m, N - m), radius=210, fill=255)
img.paste(grad, (0, 0), mask)

# Soft light in the upper left.
glow = Image.new('RGBA', (N, N), (0, 0, 0, 0))
ImageDraw.Draw(glow).ellipse((-200, -300, 700, 500), fill=(255, 255, 255, 60))
glow = glow.filter(ImageFilter.GaussianBlur(120))
img = Image.alpha_composite(img, Image.composite(glow, Image.new('RGBA', (N, N), (0, 0, 0, 0)), mask))

d = ImageDraw.Draw(img)

# Isometric cube.
cx, cy, s = 512, 560, 290
c30 = math.cos(math.radians(30))
def P(x, y, z): return (cx + (x - y) * s * c30, cy + (x + y) * s * 0.5 - z * s)
def poly(pts, fill): d.polygon([P(*p) for p in pts], fill=fill)

cy += int(s * 0.32)        # Room for the nozzle above the cube.

# Shadow on the "bed".
sh = Image.new('RGBA', (N, N), (0, 0, 0, 0))
sd = ImageDraw.Draw(sh)
sd.polygon([P(0, 0, 0), P(1.15, 0, 0), P(1.15, 1.15, 0), P(0, 1.15, 0)], fill=(0, 0, 0, 90))
sh = sh.filter(ImageFilter.GaussianBlur(18))
sh.putalpha(Image.composite(sh.getchannel('A'), Image.new('L', (N, N), 0), mask))
img = Image.alpha_composite(img, sh)
d = ImageDraw.Draw(img)

top, left, right = hexc('#FFB74D'), hexc('#F57C00'), hexc('#E65100')
poly([(1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1)], right)   # x = 1 face (right)
poly([(0, 1, 0), (1, 1, 0), (1, 1, 1), (0, 1, 1)], left)    # y = 1 face (left)
poly([(0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)], top)     # top

# Layer lines on the sides.
for i in range(1, 8):
    z = i / 8
    d.line([P(1, 0, z), P(1, 1, z)], fill=hexc('#BF360C', 150), width=6)
    d.line([P(0, 1, z), P(1, 1, z)], fill=hexc('#E65100', 150), width=6)
# Infill hatching on the top face.
for i in range(1, 9):
    t = i / 9
    d.line([P(t, 0, 1), P(t, 1, 1)], fill=hexc('#FFE0B2', 150), width=5)
# Edge highlight.
d.line([P(0, 1, 1), P(1, 1, 1), P(1, 0, 1)], fill=hexc('#FFF3E0', 230), width=8)
d.line([P(1, 1, 1), P(1, 1, 0)], fill=hexc('#FFCC80', 200), width=6)

# Nozzle above the top face: heater block and a cone, tip at the face centre.
tip = P(0.5, 0.5, 1.0)
tx, ty = tip[0], tip[1] - 18
block_w, block_h = 170, 100
bx0, by0 = tx - block_w / 2, ty - 95 - block_h
d.rounded_rectangle((tx - 30, by0 - 110, tx + 30, by0 + 10), radius=10, fill=hexc('#B0BEC5'))   # heat break
d.rounded_rectangle((bx0, by0, bx0 + block_w, by0 + block_h), radius=22, fill=hexc('#ECEFF1'))  # block
d.rounded_rectangle((bx0, by0 + block_h - 28, bx0 + block_w, by0 + block_h), radius=14, fill=hexc('#CFD8DC'))
d.polygon([(tx - 50, by0 + block_h), (tx + 50, by0 + block_h), (tx + 12, ty - 10), (tx - 12, ty - 10)],
          fill=hexc('#FFD54F'))                                                                    # brass nozzle
d.polygon([(tx - 12, ty - 10), (tx + 12, ty - 10), (tx, ty + 6)], fill=hexc('#FFB300'))
# Hot glow at the tip.
g2 = Image.new('RGBA', (N, N), (0, 0, 0, 0))
ImageDraw.Draw(g2).ellipse((tx - 60, ty - 30, tx + 60, ty + 40), fill=(255, 220, 120, 150))
g2 = g2.filter(ImageFilter.GaussianBlur(22))
img = Image.alpha_composite(img, g2)

sizes = [16, 24, 32, 48, 64, 128, 256]
img.resize((256, 256), Image.LANCZOS).save(os.path.join(out, 'AppIcon.ico'), sizes=[(z, z) for z in sizes])
img.resize((64, 64), Image.LANCZOS).save(os.path.join(out, 'AppIcon.png'))
