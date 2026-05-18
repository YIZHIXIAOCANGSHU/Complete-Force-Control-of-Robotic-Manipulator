"""Lifecycle utilities for mode startup and shutdown."""

from __future__ import annotations

from contextlib import contextmanager
from typing import Iterator


@contextmanager
def closing_runtime(*resources) -> Iterator[None]:
    try:
        yield
    finally:
        for resource in reversed(resources):
            close = getattr(resource, "close", None)
            if close is None:
                continue
            close()

