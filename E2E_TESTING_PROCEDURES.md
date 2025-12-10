# End-to-End Testing Procedures

## Pre-Deployment Testing

### 1. Local Build Test
```bash
# Install dependencies
npm install

# Build the project
npm run build

# Serve locally to test
npm run serve
```

### 2. API Connectivity Test
```bash
# Test backend health endpoint
curl http://localhost:8000/health

# Expected response:
# {"status": "healthy", "service": "rag-backend"}
```

## Post-Deployment Testing

### 1. Frontend Deployment Verification
1. **Access the deployed site**:
   - Visit: `https://your-project-name.vercel.app`
   - Verify the Docusaurus site loads completely
   - Check that all documentation pages are accessible
   - Verify the chatbot UI is present and functional

2. **Check browser console**:
   - Open browser developer tools (F12)
   - Look for any JavaScript errors
   - Verify no CORS errors are present

### 2. Backend API Testing
1. **Health check**:
   - Endpoint: `GET /health`
   - Expected: 200 status with health information

2. **API functionality test**:
   ```bash
   curl -X POST \
     -H "Content-Type: application/json" \
     -d '{"question": "What is this project about?"}' \
     https://your-backend-project-name-production.up.railway.app/ask
   ```

### 3. Chatbot End-to-End Testing
1. **Basic functionality**:
   - Navigate to the chatbot interface
   - Enter a simple question related to your documentation
   - Verify a response is returned
   - Check that sources are properly cited

2. **Response quality verification**:
   - Ask questions about different topics in your documentation
   - Verify responses are accurate and relevant
   - Check that the chatbot is using the knowledge base properly

3. **Error handling**:
   - Test with empty questions
   - Test with very long questions
   - Verify proper error messages are displayed

### 4. Integration Testing
1. **Frontend-Backend Connection**:
   - Verify API calls from frontend to backend work correctly
   - Check that the rewrite rules in vercel.json are working
   - Confirm no CORS errors occur

2. **Environment variables**:
   - Verify NEXT_PUBLIC_API_BASE_URL is correctly set
   - Confirm the frontend is connecting to the correct backend

### 5. Performance Testing
1. **Load time**:
   - Measure how long the frontend takes to load
   - Check that the chatbot interface loads quickly

2. **Response time**:
   - Time how long it takes for the backend to respond to queries
   - Verify responses come back within acceptable timeframes (under 5-10 seconds)

### 6. Verification Checklist

#### Frontend Tests:
- [ ] Site loads completely on Vercel
- [ ] All documentation pages accessible
- [ ] Chatbot UI is present and styled correctly
- [ ] No console errors
- [ ] Responsive design works on mobile/desktop

#### Backend Tests:
- [ ] Health endpoint returns 200 OK
- [ ] API endpoint accepts questions and returns answers
- [ ] Responses include proper sources
- [ ] Error handling works appropriately

#### Integration Tests:
- [ ] Frontend successfully calls backend API
- [ ] No CORS errors in browser console
- [ ] API rewrites are working properly
- [ ] Environment variables are correctly configured

#### Functional Tests:
- [ ] Chatbot returns relevant answers
- [ ] Sources are properly cited
- [ ] Different types of questions work
- [ ] Error states are handled gracefully

#### Performance Tests:
- [ ] Frontend loads within 3 seconds
- [ ] Backend responds within 5-10 seconds
- [ ] No timeout issues
- [ ] Smooth user experience

## Troubleshooting Common Issues

### CORS Issues
- Verify ALLOWED_ORIGINS in backend includes your Vercel domain
- Check that vercel.json rewrites are properly configured

### API Connection Failures
- Confirm NEXT_PUBLIC_API_BASE_URL is set correctly
- Verify backend service is running and accessible

### Build Failures
- Check that all dependencies are properly specified
- Verify the build command in vercel.json matches package.json

### Chatbot Not Responding
- Verify the backend API is accessible
- Check that Qdrant and OpenAI services are properly configured
- Confirm environment variables are set correctly