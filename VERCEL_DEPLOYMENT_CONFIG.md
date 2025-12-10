# Vercel Deployment Configuration

## Setting up Vercel Project

1. **Install Vercel CLI** (optional, for command-line deployment):
   ```bash
   npm install -g vercel
   ```

2. **Login to Vercel**:
   ```bash
   vercel login
   ```

## Manual Vercel Dashboard Setup

### Step 1: Import Project from GitHub
1. Go to https://vercel.com/dashboard
2. Click "Add New..." → "Project"
3. Select your GitHub account and repository
4. Click "Import"

### Step 2: Configure Project Settings
1. **Build & Development Settings**:
   - Framework Preset: `Other` (already set in vercel.json)
   - Build Command: `yarn build` or `npm run build`
   - Output Directory: `build`
   - Root Directory: `.` (root)

2. **Environment Variables** (already configured in vercel.json):
   - `NEXT_PUBLIC_API_BASE_URL`: Your backend API URL
   - `NEXT_PUBLIC_CHATBOT_ENABLED`: true

3. **Build Command Override**: `yarn build` (or `npm run build`)
4. **Output Directory Override**: `build`

## Vercel Project Configuration File (vercel.json)

The `vercel.json` file is already configured with:

```json
{
  "buildCommand": "npm run build",
  "outputDirectory": "build",
  "framework": "other",
  "env": {
    "NEXT_PUBLIC_API_BASE_URL": "https://your-backend-project-name-production.up.railway.app",
    "NEXT_PUBLIC_CHATBOT_ENABLED": "true"
  },
  "regions": ["sfo1"],
  "headers": [
    {
      "source": "/api/:path*",
      "headers": [
        {
          "key": "Cache-Control",
          "value": "no-cache, no-store, max-age=0, must-revalidate"
        }
      ]
    },
    {
      "source": "/(.*)",
      "headers": [
        {
          "key": "X-Content-Type-Options",
          "value": "nosniff"
        },
        {
          "key": "X-Frame-Options",
          "value": "DENY"
        },
        {
          "key": "X-XSS-Protection",
          "value": "1; mode=block"
        }
      ]
    }
  ],
  "rewrites": [
    {
      "source": "/api/:path*",
      "destination": "https://your-backend-project-name-production.up.railway.app/:path*"
    }
  ],
  "redirects": [
    {
      "source": "/chat",
      "destination": "/chatbot",
      "permanent": true
    }
  ]
}
```

## API Rewrites Configuration

The rewrites in vercel.json ensure that frontend API requests are properly forwarded to your backend service:

- All requests to `/api/*` from the frontend will be rewritten to your backend API
- This handles CORS issues by making the API calls appear to come from the same domain

## Deployment Process

1. **Automatic Deployment**: Once connected to GitHub, Vercel will automatically deploy on every push to the main branch
2. **Manual Deployment**: Use `vercel --prod` to deploy manually from the command line
3. **Preview Deployments**: Pull requests will automatically generate preview deployments

## Environment Variables Setup

### For Production:
- `NEXT_PUBLIC_API_BASE_URL`: Set to your production backend URL
- `NEXT_PUBLIC_CHATBOT_ENABLED`: Set to "true" to enable the chatbot UI

### For Preview Deployments:
- Same variables will be available but may point to staging backend if configured

## Custom Domains (Optional)

1. Go to your project settings in Vercel dashboard
2. Navigate to "Domains" section
3. Add your custom domain
4. Update DNS settings as instructed by Vercel