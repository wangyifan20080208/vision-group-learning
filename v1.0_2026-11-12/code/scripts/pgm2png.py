import sys
from PIL import Image

src, dst = sys.argv[1], sys.argv[2]
im = Image.open(src).convert("L")
print("size:", im.size)
lo = int(sys.argv[3]) if len(sys.argv) > 3 else 1

# 统计像素分布，判断地图内容
h = im.histogram()
print("dark(<60):", sum(h[:60]))
print("mid(60-200):", sum(h[60:200]))
print("light(>=200):", sum(h[200:]))

im2 = im.point(lambda p: 255 if p > 200 else (0 if p < 60 else 128))
scale = 4
im2.resize((im2.width * scale, im2.height * scale), Image.NEAREST).save(dst)
print("saved", dst)
