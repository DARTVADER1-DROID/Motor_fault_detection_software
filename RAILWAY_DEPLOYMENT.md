# Railway Deployment Guide for Motor Fleet System

This project is now configured for deployment on Railway. Follow these steps to deploy:

## Prerequisites
- A [Railway account](https://railway.app)
- Your project connected to a Git repository (GitHub recommended)
- Docker installed locally (for testing, optional)

## 1. Quick Start (Recommended)

### Option A: Using Railway CLI

```bash
# Install Railway CLI (macOS/Linux)
npm i -g @railway/cli

# Or via Homebrew (macOS)
brew install railway

# Login to Railway
railway login

# Initialize Railway project in your repo root
railway init

# Deploy
railway up
```

### Option B: Using Railway Dashboard

1. Go to [railway.app](https://railway.app)
2. Click **"Create New Project"**
3. Select **"Deploy from GitHub"**
4. Connect your GitHub account and select this repository
5. Railway will automatically detect the `Dockerfile` and `Procfile`
6. Click **"Deploy"**

## 2. Environment Variables Setup

Railway will need environment variables. Set them in the Railway dashboard:

### Via Railway Dashboard:
1. Open your project → Variables section
2. Add the following variables:

```
SERVER_HOST=0.0.0.0
SERVER_PORT=8000
WORKERS=4
DEBUG=false
LOG_LEVEL=INFO
```

### For Database (Optional):
If you want persistent storage:
1. Add a PostgreSQL plugin in Railway dashboard
2. The `DATABASE_URL` will be automatically available
3. Uncomment and use it in your code if needed

## 3. Port Configuration (IMPORTANT)

⚠️ **Railway automatically sets the `$PORT` environment variable.**

Your application reads `SERVER_PORT` from env variables:
- Default: `8000`
- Railway provides: `$PORT` environment variable
- The app is already configured to use these correctly

No additional changes needed!

## 4. Dockerfile Details

The provided `Dockerfile`:
- Uses Python 3.11-slim base image (lightweight)
- Installs dependencies from `requirements.txt`
- Includes health checks for Railway
- Exposes port 8000
- Runs: `python -m backend.main`

## 5. Procfile Configuration

The `Procfile` tells Railway how to start the app:
```
web: python -m backend.main
```

## 6. Testing Locally (Optional)

Test your Docker image locally before deploying:

```bash
# Build the image
docker build -t motor-fleet:latest .

# Run the container
docker run -p 8000:8000 \
  -e SERVER_HOST=0.0.0.0 \
  -e SERVER_PORT=8000 \
  -e DEBUG=false \
  motor-fleet:latest

# Test the API
curl http://localhost:8000/docs
```

## 7. Deployment Checklist

- [x] `Dockerfile` created ✓
- [x] `Procfile` created ✓
- [x] `.dockerignore` created ✓
- [x] `requirements.txt` configured ✓
- [x] Environment variables documented ✓
- [ ] Push code to GitHub
- [ ] Connect to Railway
- [ ] Set environment variables in Railway dashboard
- [ ] Deploy!

## 8. Post-Deployment

After deployment, Railway will:
1. Build your Docker image
2. Start the container
3. Provide you with a public URL (e.g., `https://motor-fleet-abc123.railway.app`)

### Access Your Application:
- **API Docs**: `https://your-railway-url/docs`
- **API**: `https://your-railway-url/api/...`
- **Frontend**: Served from `/frontend/` if configured

## 9. Viewing Logs

Monitor your deployment in Railway dashboard or CLI:

```bash
# View logs
railway logs

# Follow logs in real-time
railway logs --follow
```

## 10. Common Issues & Solutions

### Issue: Port already in use
- **Solution**: Railway uses the `$PORT` environment variable automatically. No action needed.

### Issue: Container keeps restarting
- **Check logs** in Railway dashboard
- **Verify environment variables** are set correctly
- **Check health check** status

### Issue: Database connection fails
- **Verify DATABASE_URL** is set if using a database
- **Check PostgreSQL plugin** is added to your Railway project

### Issue: Slow startup
- **Normal for first deployment** (cold start)
- **Subsequent starts are faster**
- Consider adding more resources via Railway dashboard

## 11. Scaling & Resources

In Railway dashboard, you can:
- Increase **CPU & RAM** allocation
- Adjust **deployment** strategy
- Add **auto-scaling** rules
- Configure **health checks**

## 12. Git Deployment (Recommended)

The easiest way is to connect Railway to your GitHub repo:

1. Commit all changes (including new files)
   ```bash
   git add .
   git commit -m "Configure for Railway deployment"
   git push origin main
   ```

2. Connect to Railway via dashboard
3. Each push to `main` will auto-deploy

## 13. Support

- **Railway Docs**: https://docs.railway.app
- **Railway Community**: https://discord.gg/railway

---

**You're ready to deploy! 🚀**

If you have any issues, check the Railway logs in the dashboard or contact Railway support.
