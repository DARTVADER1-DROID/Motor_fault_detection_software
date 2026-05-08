"""
Motor Control System v3.0 - Application Entry Point
====================================================
Starts the FastAPI server via uvicorn.

All service initialisation (cache, DB, motor fleet, monitoring)
is handled by the @app.on_event("startup") hook in
backend.api.endpoints — no duplicate init here.

Usage:
    python -m backend.main
"""

import os
import logging
from dotenv import load_dotenv
import uvicorn

# Load .env before anything else
load_dotenv()

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


def main() -> None:
    host    = os.getenv("SERVER_HOST", "0.0.0.0")
    port    = int(os.getenv("SERVER_PORT", "8000"))
    workers = int(os.getenv("WORKERS", "1"))
    debug   = os.getenv("DEBUG", "false").lower() == "true"

    logger.info("=" * 60)
    logger.info("MOTOR CONTROL SYSTEM v3.0")
    logger.info(f"  Host    : {host}:{port}")
    logger.info(f"  Workers : {workers}")
    logger.info(f"  Reload  : {debug}")
    logger.info(f"  Docs    : http://localhost:{port}/docs")
    logger.info("=" * 60)

    uvicorn.run(
        "backend.api.endpoints:app",   # ← correct dotted module path
        host=host,
        port=port,
        workers=workers,
        log_level="info",
        reload=debug,
    )


if __name__ == "__main__":
    main()
