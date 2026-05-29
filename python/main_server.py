#!/usr/bin/env python3
"""Compatibility wrapper for the simulation entrypoint."""

from sim.main_server import *  # noqa: F401,F403

if __name__ == "__main__":
    from sim.main_server import main

    main()
