#!/usr/bin/env python3

from __future__ import annotations

import os
import sys


PYTHON_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT = os.path.dirname(PYTHON_ROOT)
if PYTHON_ROOT not in sys.path:
    sys.path.insert(0, PYTHON_ROOT)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from usbfdcan_sim.app import main


if __name__ == "__main__":
    main()
