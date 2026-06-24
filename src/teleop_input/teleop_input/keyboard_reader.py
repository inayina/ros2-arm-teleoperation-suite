#!/usr/bin/env python3
"""Nonblocking terminal keyboard reader for teleop_input."""

import select
import sys
import termios
import tty


class KeyboardReader:
    """Read single keys without blocking when stdin is an interactive terminal."""

    def __init__(self):
        self._stream = sys.stdin
        self._enabled = self._stream.isatty()
        self._old_settings = None
        if self._enabled:
            self._old_settings = termios.tcgetattr(self._stream)
            tty.setcbreak(self._stream.fileno())

    @property
    def enabled(self) -> bool:
        return self._enabled

    def get_key(self):
        if not self._enabled:
            return None
        readable, _, _ = select.select([self._stream], [], [], 0.0)
        if not readable:
            return None
        return self._stream.read(1)

    def close(self):
        if self._enabled and self._old_settings is not None:
            termios.tcsetattr(self._stream, termios.TCSADRAIN, self._old_settings)
            self._old_settings = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
