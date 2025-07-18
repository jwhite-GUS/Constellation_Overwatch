# Galaxy Unmanned Systems - PowerShell Setup Script
# This script sets up the ROS 2 development environment on Windows

Write-Host "🌟 Galaxy Unmanned Systems - ROS 2 Environment Setup (Windows)" -ForegroundColor Yellow
Write-Host "=============================================================" -ForegroundColor Yellow

# Check if Docker is running
try {
    docker info | Out-Null
    Write-Host "✅ Docker is running" -ForegroundColor Green
} catch {
    Write-Host "❌ Docker is not running. Please start Docker Desktop and try again." -ForegroundColor Red
    exit 1
}

# Check if docker-compose is available
try {
    docker-compose --version | Out-Null
    Write-Host "✅ docker-compose is available" -ForegroundColor Green
} catch {
    Write-Host "❌ docker-compose is not installed. Please install Docker Desktop." -ForegroundColor Red
    exit 1
}

# Build the Docker images
Write-Host "🔨 Building Docker images..." -ForegroundColor Cyan
docker-compose build

if ($LASTEXITCODE -eq 0) {
    Write-Host "🚀 Starting ROS 2 development environment..." -ForegroundColor Cyan
    docker-compose up -d
    
    if ($LASTEXITCODE -eq 0) {
        Write-Host "✅ Environment setup complete!" -ForegroundColor Green
        Write-Host ""
        Write-Host "💡 Next steps:" -ForegroundColor Yellow
        Write-Host "1. Open VS Code in this directory" -ForegroundColor White
        Write-Host "2. Install the recommended VS Code extensions" -ForegroundColor White
        Write-Host "3. Use 'docker-compose exec ros-dev bash' to enter the container" -ForegroundColor White
        Write-Host "4. Run './scripts/build.sh' to build the workspace" -ForegroundColor White
        Write-Host ""
        Write-Host "🎯 Happy coding!" -ForegroundColor Green
    } else {
        Write-Host "❌ Failed to start the environment" -ForegroundColor Red
        exit 1
    }
} else {
    Write-Host "❌ Failed to build Docker images" -ForegroundColor Red
    exit 1
}
