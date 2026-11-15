#!/usr/bin/env bash
set -e

echo "=========================================================="
echo " Running Robotics Phase 2 Full Test Suite"
echo "=========================================================="

echo ""
echo ">>> [1/2] Building and running C++ unit tests (CTest)..."
cmake -B build -S .
cmake --build build
ctest --test-dir build --output-on-failure
./build/test_rover_logic

echo ""
echo ">>> [2/2] Running Python unit tests (pytest)..."
if [ ! -d ".venv" ]; then
    echo "Creating virtual environment in .venv..."
    python3 -m venv .venv
    .venv/bin/pip install --upgrade pip
    .venv/bin/pip install -r requirements.txt
fi

export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
.venv/bin/pytest -v tests/test_dashboard.py

echo ""
echo "=========================================================="
echo " All C++ and Python tests passed successfully!"
echo "=========================================================="
