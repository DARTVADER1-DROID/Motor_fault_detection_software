# Build Troubleshooting & Fix Notes

## ✅ Fixed: pydantic-core build failure on Python 3.13

### Root Cause
Railway's build was using **Python 3.13** (via Nixpacks auto-detection) instead of the
Python 3.11 specified in the Dockerfile. `pydantic-core==2.14.1` (bundled with pydantic
2.5.x) is incompatible with Python 3.13 due to a `ForwardRef._evaluate()` API change.

### Symptoms
```
TypeError: ForwardRef._evaluate() missing 1 required keyword-only argument: 'recursive_guard'
ERROR: Failed building wheel for pydantic-core
```

### Fixes Applied

1. **`railway.json`** — Now explicitly sets `"builder": "DOCKERFILE"` with the path.
   Previously it just said `"dockerfile"` (lowercase) which Railway may have ignored,
   falling back to Nixpacks and Python 3.13.

2. **`nixpacks.toml`** — Added as belt-and-suspenders: if Railway ever uses Nixpacks,
   it is now told to use `python311` explicitly.

3. **`requirements.txt`** — Upgraded pydantic to `2.6.3` (compatible with Python 3.11+)
   and updated other packages to stable recent versions.

4. **`Dockerfile`** — Added `ENV SERVER_PORT=${PORT:-8000}` so Railway's injected `$PORT`
   is respected, and fixed the healthcheck to use `$SERVER_PORT`.

5. **`backend/main.py`** — Port now reads `$PORT` first (Railway's env var), then
   `$SERVER_PORT`, then defaults to `8000`. This prevents Railway from routing to the
   wrong port.

### Verified Working Versions
- Python: 3.11 (pinned via Dockerfile FROM and nixpacks.toml)
- fastapi: 0.109.0
- uvicorn[standard]: 0.27.0
- pydantic: 2.6.3
- python-dotenv: 1.0.1
- sqlalchemy: 2.0.25
- psycopg2-binary: 2.9.9
