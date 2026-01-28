#!/bin/bash
# Fix line endings for migration script
# Run this in WSL/Linux

# Fix the migration script itself
sed -i 's/\r$//' migrate_to_github_structure.sh

# Make it executable
chmod +x migrate_to_github_structure.sh

echo "Fixed line endings for migrate_to_github_structure.sh"
echo "You can now run: bash migrate_to_github_structure.sh"
