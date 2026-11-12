#!/usr/bin/env bash
set -e
VENV="$HOME/cadvenv"

echo "==> 1/4 安装 virtualenv"
python3 -m pip install --user -q virtualenv

echo "==> 2/4 创建 venv"
rm -rf "$VENV"
python3 -m virtualenv -q "$VENV"

echo "==> 3/4 安装 cadquery + ezdxf"
"$VENV/bin/pip" install -q --upgrade pip
"$VENV/bin/pip" install -q cadquery ezdxf

echo "==> 4/4 验证"
"$VENV/bin/python" -c "import cadquery; print('cadquery', cadquery.__version__)"
"$VENV/bin/python" -c "import ezdxf; print('ezdxf', ezdxf.__version__)"
echo "ALL_OK"
