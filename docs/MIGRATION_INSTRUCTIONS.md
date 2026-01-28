# Migration Instructions - Fix Line Endings Issue

The migration script has Windows line endings that need to be fixed in WSL.

## Quick Fix (Run in WSL)

```bash
cd /mnt/c/Users/kyose/Documents/AR2.6/ar_robot

# Option 1: Use sed to fix line endings
sed -i 's/\r$//' migrate_to_github_structure.sh

# Option 2: Use dos2unix (if installed)
dos2unix migrate_to_github_structure.sh

# Option 3: Use the fix script
bash fix_line_endings_migration.sh

# Then run the migration
bash migrate_to_github_structure.sh
```

## Alternative: Manual Fix

If the script still has issues, you can manually fix line endings:

```bash
# In WSL, convert the file
tr -d '\r' < migrate_to_github_structure.sh > migrate_to_github_structure_fixed.sh
mv migrate_to_github_structure_fixed.sh migrate_to_github_structure.sh
chmod +x migrate_to_github_structure.sh
bash migrate_to_github_structure.sh
```

## Or: Run Commands Directly

Instead of using the script, you can run the commands manually:

```bash
# Create directories
mkdir -p docs/{research,diagrams}
mkdir -p models/{fusion360/components,meshes,urdf,sdf}
mkdir -p simulation/{worlds,scripts,fusion_simulation,config}
mkdir -p control/{motor_control,mission_control/launch,joint_control}
mkdir -p tests/{unit_tests,integration_tests}
mkdir -p scripts/{setup,tools,build}
mkdir -p media/{images/{robot,screenshots,diagrams},videos}
mkdir -p data/{results,logs,config}
mkdir -p .github/workflows

# Then manually move files as needed
# See GITHUB_STRUCTURE.md for the complete structure
```
