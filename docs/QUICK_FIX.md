# Quick Fix for Line Endings

Run this **ONE command** in your WSL terminal to fix the migration script:

```bash
cd /mnt/c/Users/kyose/Documents/AR2.6/ar_robot && sed -i 's/\r$//' migrate_to_github_structure.sh && chmod +x migrate_to_github_structure.sh && echo "Fixed! Now run: bash migrate_to_github_structure.sh"
```

Then run:
```bash
bash migrate_to_github_structure.sh
```

## Alternative: Use Python to Fix

If sed doesn't work, use Python:

```bash
cd /mnt/c/Users/kyose/Documents/AR2.6/ar_robot
python3 -c "
with open('migrate_to_github_structure.sh', 'rb') as f:
    data = f.read()
data = data.replace(b'\r\n', b'\n').replace(b'\r', b'\n')
with open('migrate_to_github_structure.sh', 'wb') as f:
    f.write(data)
print('Fixed line endings!')
"
chmod +x migrate_to_github_structure.sh
bash migrate_to_github_structure.sh
```
