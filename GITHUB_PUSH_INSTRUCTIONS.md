# GitHub Push Instructions

## Before Pushing to GitHub

1. **Verify all changes are committed**:
   ```bash
   git status
   ```

2. **Install dependencies and build locally to test**:
   ```bash
   npm install
   npm run build
   ```

3. **Commit all changes**:
   ```bash
   git add .
   git commit -m "feat: configure Vercel deployment for Docusaurus frontend"
   ```

4. **Push to GitHub**:
   ```bash
   git push origin main
   ```

## GitHub Repository Setup

If this is the first time deploying:

1. **Create/verify GitHub repository**:
   - Go to GitHub.com and create a new repository if it doesn't exist
   - Name it appropriately (e.g., `docusaurus-rag-chatbot`)
   - Add the remote if not already configured:
     ```bash
     git remote add origin https://github.com/your-username/your-repository-name.git
     ```

2. **Push initial code**:
   ```bash
   git branch -M main
   git push -u origin main
   ```