# Build & Dependency Troubleshooting Guide

## Issue: `pydantic-core` Build Error

**Symptoms:**
```
error: failed to run custom build command for `pydantic-core`
TypeError: ForwardRef._evaluate() missing 1 required keyword-only argument: 'recursive_guard'
```

**Root Cause:**
Python 3.13 has breaking changes in the `ForwardRef` API that aren't compatible with older pydantic versions.

**Solution:**
✅ **Already Fixed!** Updated to compatible versions:
- `pydantic==2.6.3` (was 2.5.0)
- `fastapi==0.109.0` (was 0.104.1)
- `uvicorn[standard]==0.27.0` (was 0.24.0)

These versions have pre-built wheels, so no compilation needed!

---

## Issue: Railway Build Fails

**Problem:** Docker build works locally but fails on Railway

**Solutions:**

### 1. Check Python Version
Railway defaults to Python 3.11, which we're using ✅

### 2. Clear Cache
```bash
# If using Railway CLI
railway rebuild --skip-cache

# If using GitHub:
# Go to Railway dashboard → Services → Settings → Rebuild
```

### 3. Check Dependencies
Ensure `requirements.txt` has compatible versions (already done ✅)

---

## Issue: ImportError or ModuleNotFoundError

**Problem:** `ModuleNotFoundError: No module named 'backend'`

**Solutions:**

1. **Ensure WORKDIR is correct** in Dockerfile (it is: `/app`)
2. **Check file structure:**
   ```
   motor_fixed/
   ├── backend/
   │   ├── __init__.py      ← Must exist!
   │   ├── main.py
   │   ├── api/
   │   │   ├── __init__.py  ← Must exist!
   │   │   └── endpoints.py
   │   └── ...
   └── requirements.txt
   ```
3. **Verify `__init__.py` files exist** in all directories

---

## Issue: Port Binding Error

**Problem:** `Address already in use` or `Cannot bind to port`

**Solution:**
Railway automatically sets `$PORT` environment variable. Your app reads from `SERVER_PORT`:
- Add to Railway variables: `SERVER_PORT=${PORT}` 
- OR add to environment: `PORT=8000`

Actually, the app defaults to port 8000 if `SERVER_PORT` isn't set, so this should be fine.

---

## Issue: Requirements.txt Install Fails

**Problem:** Specific package fails to install

**Solution Options:**

### Option 1: Use Pre-built Wheels Only
```txt
# Minimal dependencies with only pre-built wheels
fastapi==0.109.0
uvicorn[standard]==0.27.0
pydantic==2.6.3
python-dotenv==1.0.0
sqlalchemy==2.0.25
psycopg2-binary==2.9.9
```

### Option 2: Add Extra Build Dependencies
Update Dockerfile to install more build tools:
```dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    libpq-dev \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*
```

### Option 3: Use Alpine Linux (Smaller but More Issues)
Not recommended for this project. Stick with `python:3.11-slim`.

---

## Issue: Docker Image Too Large

**Problem:** Image size > 500MB

**Solutions:**

1. **Multi-stage build** (not needed for your size)
2. **Remove unnecessary files** (.dockerignore - already done ✅)
3. **Use slim image** (already using python:3.11-slim ✅)

Current expected size: **~150-200MB** (reasonable)

---

## Issue: Health Check Failing

**Problem:** 
```
Health check status: unhealthy
```

**Solutions:**

1. **Wait for startup** - Health checks start after 5s. If app takes longer, increase `--start-period`:
   ```dockerfile
   HEALTHCHECK --interval=30s --timeout=10s --start-period=15s --retries=3 \
       CMD python -c "import urllib.request; urllib.request.urlopen('http://localhost:8000/docs').read()" || exit 1
   ```

2. **Disable health check temporarily** (for testing):
   Remove or comment out the `HEALTHCHECK` line in Dockerfile

3. **Check logs** - App might be crashing before health check runs

---

## Issue: Database Connection Fails

**Problem:** 
```
could not connect to server: No such file or directory
```

**Solutions:**

1. **You're using cache mode** (no database required)
   - Just don't set `DATABASE_URL` ✅

2. **If you want persistent database:**
   - Add PostgreSQL plugin in Railway dashboard
   - Railway auto-provides `DATABASE_URL` as environment variable
   - Uncomment in `config/.env.example`

3. **Test connection:**
   ```bash
   # In Railway terminal or logs, check:
   echo $DATABASE_URL
   ```

---

## Version Compatibility Matrix

| Component | Recommended | Python 3.11 | Python 3.12 | Notes |
|-----------|------------|-------------|-------------|-------|
| Python | 3.11 | ✅ | ✅ | 3.13 has breaking changes |
| FastAPI | 0.109.0 | ✅ | ✅ | Supports Python 3.8+ |
| Pydantic | 2.6.3 | ✅ | ✅ | Fixed Python 3.13 issues in 2.7+ |
| Uvicorn | 0.27.0 | ✅ | ✅ | Latest stable |
| SQLAlchemy | 2.0.25 | ✅ | ✅ | Optional, for database |

---

## Quick Fixes Checklist

- [ ] Updated `requirements.txt` to compatible versions
- [ ] Using `python:3.11-slim` in Dockerfile
- [ ] All `__init__.py` files exist in package directories
- [ ] `Procfile` points to correct entry point
- [ ] Environment variables set in Railway dashboard
- [ ] `.dockerignore` configured to skip unnecessary files
- [ ] Health check timeout sufficient for your startup time

---

## Testing Locally

Before deploying to Railway, test locally:

```bash
# Build image
docker build -t motor-fleet:test .

# Run with proper environment
docker run -p 8000:8000 \
  -e SERVER_HOST=0.0.0.0 \
  -e SERVER_PORT=8000 \
  -e WORKERS=1 \
  motor-fleet:test

# Test in another terminal
curl http://localhost:8000/docs
```

Expected: FastAPI Swagger UI loads successfully

---

## Need Help?

1. **Check Railway logs:** Railway dashboard → Deployments → View logs
2. **Check Docker build logs:** Same dashboard
3. **Local debugging:** Run Docker image locally first
4. **Minimal test:** Start with `WORKERS=1` and `DEBUG=false`

---

## Common Success Indicators

✅ Docker builds without errors  
✅ Container starts and stays running  
✅ Health checks pass  
✅ API responds to requests  
✅ Logs appear in Railway dashboard  

Once you see these, your deployment is successful! 🚀
