#!/usr/bin/env bash
F=/mnt/c/Users/wangyifan/Downloads/RMUL2026.step
echo "=== units ==="
grep -o "SI_UNIT([^)]*)" "$F" | head -5
echo "=== products ==="
grep -o "PRODUCT ('[^']*'" "$F" | head -10
echo "=== entity counts ==="
for k in CARTESIAN_POINT ADVANCED_FACE EDGE_CURVE VERTEX_POINT PLANE CYLINDRICAL_SURFACE MANIFOLD_SOLID_BREP; do
  printf "%-24s %s\n" "$k" "$(grep -c "$k" "$F")"
done
echo "=== tail ==="
tail -c 120 "$F"
