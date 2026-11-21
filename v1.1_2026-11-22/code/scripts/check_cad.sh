#!/usr/bin/env bash
echo "=== cadvenv ==="
if [ -x "$HOME/cadvenv/bin/python" ]; then
  "$HOME/cadvenv/bin/python" -c "import cadquery; print('cadquery OK', cadquery.__version__)" 2>&1 | head -2
  "$HOME/cadvenv/bin/python" -c "import ezdxf; print('ezdxf OK', ezdxf.__version__)" 2>&1 | head -2
else
  echo "cadvenv 不存在"
fi
echo "=== disk ==="
df -h / | tail -1
