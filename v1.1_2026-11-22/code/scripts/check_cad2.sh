#!/usr/bin/env bash
"$HOME/cadvenv/bin/python" -c "import cadquery" 2>&1 | tail -15
echo "-----"
"$HOME/cadvenv/bin/pip" list 2>/dev/null | grep -iE "cadquery|ocp|ezdxf|numpy|vtk" | head
