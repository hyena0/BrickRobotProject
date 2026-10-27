"""LDraw parser: Studio-exported model to structured parts (Stage 1.1)."""

from .ldr import parse_file, parse_text

__all__ = ["parse_file", "parse_text"]
