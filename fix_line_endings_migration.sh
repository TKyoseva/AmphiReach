#!/bin/bash
# Fix line endings for migration script in WSL

# Remove Windows line endings (CRLF -> LF)
sed -i 's/\r$//' migrate_to_github_structure.sh

# Make executable
chmod +x migrate_to_github_structure.sh

echo "Fixed line endings for migrate_to_github_structure.sh"
echo "Now you can run: bash migrate_to_github_structure.sh"
