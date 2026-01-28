# GitHub Commit Guide for AmphiReach

This guide will help you commit and push the AmphiReach project to GitHub.

## Prerequisites

1. GitHub account created
2. GitHub repository created (name: `AmphiReach`)
3. Git installed on your system

## Quick Setup (Automated)

Run the setup script:

```bash
cd /mnt/c/Users/kyose/Documents/AR2.6/ar_robot

# Fix line endings first (if needed)
python3 -c "data = open('setup_github_repo.sh', 'rb').read().replace(b'\r\n', b'\n').replace(b'\r', b'\n'); open('setup_github_repo.sh', 'wb').write(data)"

# Make executable and run
chmod +x setup_github_repo.sh
bash setup_github_repo.sh
```

## Manual Setup

### Step 1: Initialize Git Repository

```bash
cd /mnt/c/Users/kyose/Documents/AR2.6/ar_robot

# Initialize git (if not already done)
git init

# Set default branch to main
git branch -M main
```

### Step 2: Create/Verify .gitignore

Make sure `.gitignore` exists and includes:

```bash
# Check if .gitignore exists
ls -la .gitignore

# If it doesn't exist, copy it from ar_robot/.gitignore
cp ar_robot/.gitignore .
```

### Step 3: Add Files

```bash
# Add all files
git add .

# Check what will be committed
git status
```

### Step 4: Create Initial Commit

```bash
git commit -m "Initial commit: AmphiReach project

- Robot model (URDF) with dual-hull catamaran design
- Motor control system (ROS2 and standalone)
- Mission controller with multi-segment missions
- Gazebo simulation worlds with hydrodynamics
- Documentation and research materials
- Test scripts and utilities"
```

### Step 5: Add Remote Repository

```bash
# Replace 'yourusername' with your GitHub username
git remote add origin https://github.com/yourusername/AmphiReach.git

# Verify remote
git remote -v
```

### Step 6: Push to GitHub

```bash
# Push to GitHub
git push -u origin main
```

If you get authentication errors, you may need to:
- Use a Personal Access Token instead of password
- Set up SSH keys
- Use GitHub CLI

## Alternative: Using GitHub CLI

If you have GitHub CLI installed:

```bash
# Create repository and push in one command
gh repo create AmphiReach --public --source=. --remote=origin --push
```

## Commit Message Examples

### Initial Commit
```
Initial commit: AmphiReach project
```

### Adding Features
```
feat: Add motor control system with differential thrust
```

### Documentation
```
docs: Add motor control documentation and examples
```

### Bug Fixes
```
fix: Resolve line ending issues in migration script
```

### Updates
```
update: Improve mission controller with better speed control
```

## File Organization Before Commit

Make sure your project structure follows the recommended layout:

```
AmphiReach/
├── README.md
├── LICENSE
├── .gitignore
├── docs/
├── models/
├── simulation/
├── control/
├── tests/
├── scripts/
├── media/
└── data/
```

## Common Issues and Solutions

### Issue: Authentication Failed

**Solution**: Use Personal Access Token
1. Go to GitHub → Settings → Developer settings → Personal access tokens
2. Generate new token with `repo` permissions
3. Use token as password when pushing

### Issue: Large Files

**Solution**: Use Git LFS for large files
```bash
# Install Git LFS
git lfs install

# Track large files
git lfs track "*.f3d"
git lfs track "*.xlsx"
git lfs track "*.pdf"

# Add .gitattributes
git add .gitattributes
```

### Issue: Line Endings

**Solution**: Configure git to handle line endings
```bash
# Set git config for line endings
git config core.autocrlf input  # Linux/WSL
git config core.autocrlf true   # Windows
```

## Verify Your Commit

After pushing, verify on GitHub:

1. Go to https://github.com/yourusername/AmphiReach
2. Check that all files are present
3. Verify README.md displays correctly
4. Check that LICENSE file is visible

## Next Steps After First Commit

1. **Add repository description** on GitHub
2. **Add topics/tags** (e.g., robotics, gazebo, underwater, ros2)
3. **Create releases** for major milestones
4. **Set up GitHub Pages** for documentation (optional)
5. **Add collaborators** if working in a team

## Useful Git Commands

```bash
# Check status
git status

# View commit history
git log --oneline

# View changes
git diff

# Add specific file
git add path/to/file

# Commit with message
git commit -m "Your message"

# Push changes
git push

# Pull latest changes
git pull

# Create new branch
git checkout -b feature-name

# Switch branches
git checkout main
```

## Repository Settings on GitHub

After pushing, configure on GitHub:

1. **Settings → General**
   - Add description
   - Add website URL (if applicable)
   - Choose default branch (main)

2. **Settings → Pages** (optional)
   - Enable GitHub Pages for documentation

3. **Settings → Collaborators**
   - Add team members

4. **About section**
   - Add description
   - Add topics
   - Add website

## Troubleshooting

### "Repository not found"
- Check repository name matches exactly
- Verify you have push access
- Check remote URL: `git remote -v`

### "Permission denied"
- Use Personal Access Token
- Check SSH keys if using SSH
- Verify GitHub credentials

### "Large file" errors
- Use Git LFS for files > 100MB
- Or remove large files from commit

---

**Ready to commit?** Run `bash setup_github_repo.sh` or follow the manual steps above!
