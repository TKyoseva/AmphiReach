#!/bin/bash
# Script to set up and commit AmphiReach project to GitHub
# Run this from the project root directory (after migration)

set -e

echo "=========================================="
echo "  Setting up AmphiReach GitHub Repository"
echo "=========================================="
echo ""

# Check if git is installed
if ! command -v git &> /dev/null; then
    echo "ERROR: git is not installed. Please install git first."
    exit 1
fi

# Check if we're in a git repository
if [ ! -d ".git" ]; then
    echo "Initializing git repository..."
    git init
    echo "✓ Git repository initialized"
else
    echo "Git repository already exists"
fi

# Check if .gitignore exists
if [ ! -f ".gitignore" ]; then
    echo "WARNING: .gitignore not found. Creating default .gitignore..."
    cat > .gitignore << 'EOF'
# Python
__pycache__/
*.py[cod]
*$py.class
*.so
.Python
build/
dist/
*.egg-info/
venv/
env/
.venv

# ROS
build/
install/
log/
.ros/

# IDE
.vscode/
.idea/
*.swp
*.swo

# OS
.DS_Store
Thumbs.db

# Data/Logs
data/logs/
*.log

# Fusion 360
*.f3d.bak
*.f3d.lock

# Temporary files
*.tmp
*.bak
EOF
    echo "✓ Created .gitignore"
fi

# Add all files
echo ""
echo "Staging files..."
git add .

# Check if there are changes to commit
if git diff --cached --quiet; then
    echo "No changes to commit. Repository is up to date."
    exit 0
fi

# Show status
echo ""
echo "Files to be committed:"
git status --short

# Create initial commit
echo ""
read -p "Create initial commit? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    git commit -m "Initial commit: AmphiReach project

- Robot model (URDF) with dual-hull catamaran design
- Motor control system (ROS2 and standalone)
- Mission controller with multi-segment missions
- Gazebo simulation worlds with hydrodynamics
- Documentation and research materials
- Test scripts and utilities"
    echo ""
    echo "✓ Initial commit created"
else
    echo "Commit cancelled. Run 'git commit' manually when ready."
    exit 0
fi

# Ask about remote repository
echo ""
read -p "Do you want to add a remote GitHub repository? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    read -p "Enter GitHub repository URL (e.g., https://github.com/username/AmphiReach.git): " repo_url
    
    if [ -n "$repo_url" ]; then
        # Check if remote already exists
        if git remote | grep -q "origin"; then
            echo "Remote 'origin' already exists. Updating..."
            git remote set-url origin "$repo_url"
        else
            git remote add origin "$repo_url"
        fi
        echo "✓ Remote repository added: $repo_url"
        
        # Ask about pushing
        echo ""
        read -p "Push to GitHub now? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            # Determine default branch
            default_branch=$(git branch --show-current 2>/dev/null || echo "main")
            
            echo "Pushing to GitHub..."
            git push -u origin "$default_branch"
            echo ""
            echo "✓ Successfully pushed to GitHub!"
            echo "Repository: $repo_url"
        else
            echo "To push later, run: git push -u origin main"
        fi
    else
        echo "No URL provided. Skipping remote setup."
        echo "To add remote later, run: git remote add origin <url>"
    fi
else
    echo "Skipping remote setup."
    echo "To add remote later, run: git remote add origin <url>"
fi

echo ""
echo "=========================================="
echo "  GitHub Setup Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Review your commit: git log"
echo "2. If you haven't added remote, add it: git remote add origin <url>"
echo "3. Push to GitHub: git push -u origin main"
echo "4. Verify on GitHub: Check your repository online"
echo ""
