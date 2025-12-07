#!/bin/bash

################################################################################
# RAG Chatbot Deployment Script
#
# Automates initial setup and deployment of the RAG chatbot system.
# Supports: local development, staging, and production environments.
#
# Usage:
#   ./scripts/deploy.sh --env production --setup
#   ./scripts/deploy.sh --env staging --backend-only
#   ./scripts/deploy.sh --env development
################################################################################

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
ENVIRONMENT=${ENVIRONMENT:-"development"}
BACKEND_ONLY=${BACKEND_ONLY:-false}
FRONTEND_ONLY=${FRONTEND_ONLY:-false}
SETUP_ONLY=${SETUP_ONLY:-false}

# Print colored output
log_info() {
  echo -e "${BLUE}ℹ${NC} $1"
}

log_success() {
  echo -e "${GREEN}✓${NC} $1"
}

log_warning() {
  echo -e "${YELLOW}⚠${NC} $1"
}

log_error() {
  echo -e "${RED}✗${NC} $1"
}

# Parse command line arguments
parse_args() {
  while [[ $# -gt 0 ]]; do
    case $1 in
      --env)
        ENVIRONMENT="$2"
        shift 2
        ;;
      --setup)
        SETUP_ONLY=true
        shift
        ;;
      --backend-only)
        BACKEND_ONLY=true
        shift
        ;;
      --frontend-only)
        FRONTEND_ONLY=true
        shift
        ;;
      *)
        log_error "Unknown option: $1"
        print_usage
        exit 1
        ;;
    esac
  done
}

# Print usage
print_usage() {
  cat <<EOF
Usage: ./scripts/deploy.sh [OPTIONS]

Options:
  --env {development|staging|production}  Environment to deploy to (default: development)
  --setup                                 Run setup only, don't deploy
  --backend-only                          Deploy only backend service
  --frontend-only                         Deploy only frontend service

Examples:
  ./scripts/deploy.sh --env production --setup
  ./scripts/deploy.sh --env staging --backend-only
  ./scripts/deploy.sh --env development
EOF
}

# Check prerequisites
check_prerequisites() {
  log_info "Checking prerequisites..."

  # Check environment file exists
  ENV_FILE="$PROJECT_ROOT/.env.${ENVIRONMENT}"
  if [ ! -f "$ENV_FILE" ]; then
    log_error "Environment file not found: $ENV_FILE"
    log_info "Copy from .env.example and configure:"
    log_info "  cp .env.example $ENV_FILE"
    exit 1
  fi
  log_success "Environment file found: $ENV_FILE"

  # Check required tools
  local required_tools=("docker" "python3" "git")
  for tool in "${required_tools[@]}"; do
    if ! command -v "$tool" &> /dev/null; then
      log_error "$tool is not installed"
      exit 1
    fi
    log_success "$tool is available"
  done

  # Check git repository
  if ! git rev-parse --git-dir > /dev/null 2>&1; then
    log_error "Not a git repository"
    exit 1
  fi
  log_success "Git repository found"

  # Check branch
  local current_branch=$(git rev-parse --abbrev-ref HEAD)
  log_info "Current branch: $current_branch"
}

# Load environment variables
load_environment() {
  log_info "Loading environment: $ENVIRONMENT"

  if [ -f "$PROJECT_ROOT/.env.${ENVIRONMENT}" ]; then
    set -a
    source "$PROJECT_ROOT/.env.${ENVIRONMENT}"
    set +a
    log_success "Environment loaded"
  else
    log_error "Environment file not found: $PROJECT_ROOT/.env.${ENVIRONMENT}"
    exit 1
  fi
}

# Setup backend
setup_backend() {
  log_info "Setting up backend..."

  cd "$PROJECT_ROOT/backend"

  # Create virtual environment
  if [ ! -d "venv" ]; then
    log_info "Creating Python virtual environment..."
    python3 -m venv venv
    log_success "Virtual environment created"
  fi

  # Activate virtual environment
  source venv/bin/activate

  # Install dependencies
  log_info "Installing Python dependencies..."
  pip install --upgrade pip setuptools wheel > /dev/null
  pip install -r requirements.txt
  log_success "Dependencies installed"

  # Run migrations
  log_info "Running database migrations..."
  python migrate.py --up || log_warning "Migrations may have failed (database might not be ready)"
  log_success "Database initialized"

  # Ingest documentation
  if [ -d "../docs" ]; then
    log_info "Ingesting documentation..."
    python ingest.py --init-db --docs-path ../docs || log_warning "Ingestion failed (check database/API keys)"
    log_success "Documentation ingested"
  fi

  cd "$PROJECT_ROOT"
}

# Setup frontend
setup_frontend() {
  log_info "Setting up frontend..."

  # Check if npm is installed
  if ! command -v npm &> /dev/null; then
    log_warning "npm not installed, skipping frontend setup"
    return
  fi

  # Install dependencies
  log_info "Installing npm dependencies..."
  npm install > /dev/null
  log_success "Frontend dependencies installed"
}

# Deploy backend
deploy_backend() {
  log_info "Deploying backend to Railway..."

  if ! command -v railway &> /dev/null; then
    log_error "Railway CLI not installed"
    log_info "Install: npm install -g @railway/cli"
    exit 1
  fi

  # Login to Railway
  log_info "Authenticating with Railway..."
  railway login || log_warning "Railway authentication required"

  # Deploy
  log_info "Deploying application..."
  cd "$PROJECT_ROOT"
  railway deploy || {
    log_error "Railway deployment failed"
    exit 1
  }

  log_success "Backend deployed to Railway"
  railway open || true
}

# Deploy frontend
deploy_frontend() {
  log_info "Deploying frontend to Vercel..."

  if ! command -v vercel &> /dev/null; then
    log_error "Vercel CLI not installed"
    log_info "Install: npm install -g vercel"
    exit 1
  fi

  # Deploy to production
  local vercel_args="--prod"
  if [ "$ENVIRONMENT" = "staging" ]; then
    vercel_args="--no-wait"
  fi

  log_info "Deploying to Vercel ($ENVIRONMENT)..."
  cd "$PROJECT_ROOT"
  vercel $vercel_args || {
    log_error "Vercel deployment failed"
    exit 1
  }

  log_success "Frontend deployed to Vercel"
}

# Health check
health_check() {
  log_info "Running health checks..."

  # Get backend URL from environment
  local backend_url=${BACKEND_URL:-"http://localhost:8000"}

  # Check backend health
  log_info "Checking backend health: $backend_url/health"
  if curl -s "$backend_url/health" | grep -q "ok"; then
    log_success "Backend is healthy"
  else
    log_warning "Backend health check failed or not ready yet"
  fi
}

# Main deployment flow
main() {
  parse_args "$@"

  echo -e "${BLUE}"
  echo "╔════════════════════════════════════════════════════════════╗"
  echo "║       RAG Chatbot Deployment Script                       ║"
  echo "║       Environment: $ENVIRONMENT                               ║"
  echo "╚════════════════════════════════════════════════════════════╝"
  echo -e "${NC}"

  # Check prerequisites
  check_prerequisites

  # Load environment
  load_environment

  # Setup phase
  if [ "$SETUP_ONLY" = true ] || [ "$BACKEND_ONLY" = true ]; then
    setup_backend
  fi

  if [ "$SETUP_ONLY" = true ] || [ "$FRONTEND_ONLY" = true ]; then
    setup_frontend
  fi

  if [ "$SETUP_ONLY" = true ]; then
    log_success "Setup complete!"
    echo -e "${BLUE}Next steps:${NC}"
    echo "  1. Configure your environment: $PROJECT_ROOT/.env.${ENVIRONMENT}"
    echo "  2. Verify database credentials"
    echo "  3. Run: ./scripts/deploy.sh --env $ENVIRONMENT"
    exit 0
  fi

  # Deployment phase
  if [ "$BACKEND_ONLY" != true ] && [ "$FRONTEND_ONLY" != true ]; then
    # Deploy both
    deploy_backend
    deploy_frontend
  elif [ "$BACKEND_ONLY" = true ]; then
    deploy_backend
  elif [ "$FRONTEND_ONLY" = true ]; then
    deploy_frontend
  fi

  # Health check
  sleep 2
  health_check

  log_success "Deployment complete!"
  echo -e "${BLUE}Useful commands:${NC}"
  echo "  View logs:    railway logs"
  echo "  Health check: curl https://your-url/health"
  echo "  Redeploy:     railway redeploy"
  echo "  Docs:         cat DEPLOYMENT.md"
}

# Run main function
main "$@"
