from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional


def get_logger(
    name: str, level: int = logging.INFO, log_file: Optional[str] = None
) -> logging.Logger:
    """Return a logger configured for console (and file).

    The returned logger will have the requested *level* and will
    always have a stream handler that writes to ``sys.stderr``.  If
    *log_file* is provided the logger will also write to that file; any
    intermediate directories are created automatically.

    Duplicate handlers are avoided by checking ``logger.handlers`` so
    the function is safe to call multiple times with the same name.
    """

    logger = logging.getLogger(name)
    logger.setLevel(level)

    # If the logger already has handlers we assume it was already
    # configured by an earlier call.  This keeps messages from being
    # duplicated when modules import the factory repeatedly.
    if not logger.handlers:
        fmt = logging.Formatter("%(asctime)s | %(levelname)s | %(message)s")

        stream_h = logging.StreamHandler()
        stream_h.setFormatter(fmt)
        logger.addHandler(stream_h)

        if log_file:
            log_path = Path(log_file)
            if log_path.parent and not log_path.parent.exists():
                log_path.parent.mkdir(parents=True, exist_ok=True)

            file_h = logging.FileHandler(log_path)
            file_h.setFormatter(fmt)
            logger.addHandler(file_h)

    return logger
