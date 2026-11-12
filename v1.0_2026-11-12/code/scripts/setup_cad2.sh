#!/usr/bin/env bash
set -e
VENV="$HOME/cadvenv"
IDX="${PIP_INDEX:-https://pypi.org/simple}"

echo "==> index: $IDX"
echo "==> 1/4 virtualenv"
python3 -m pip install --user -q -i "$IDX" --timeout 60 virtualenv

echo "==> 2/4 创建 venv"
rm -rf "$VENV"
python3 -m virtualenv -q "$VENV"

echo "==> 3/4 cadquery + ezdxf（约 300MB，慢）"
"$VENV/bin/pip" install -q -i "$IDX" --timeout 120 cadquery ezdxf

echo "==> 4/4 验证"
"$VENV/bin/python" -c "import cadquery; print('cadquery', cadquery.__version__)"
"$VENV/bin/python" -c "import ezdxf; print('ezdxf', ezdxf.__version__)"
echo "ALL_OK"
