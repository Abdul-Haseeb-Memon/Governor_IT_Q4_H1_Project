# Quickstart: GitHub Push Workflow

## Prerequisites
- Git installed (version 2.0 or higher)
- GitHub account with repository access
- SSH key or personal access token configured for GitHub authentication

## Step 1: Configure Remote Repository
1. Navigate to your local repository:
   ```bash
   cd your-repository-directory
   ```

2. Add the GitHub remote repository:
   ```bash
   git remote add origin https://github.com/username/repository-name.git
   ```
   Or for SSH:
   ```bash
   git remote add origin git@github.com:username/repository-name.git
   ```

3. Verify the remote configuration:
   ```bash
   git remote -v
   ```

## Step 2: Commit Your Changes
1. Check the status of your changes:
   ```bash
   git status
   ```

2. Add files to staging:
   ```bash
   git add .
   ```
   Or add specific files:
   ```bash
   git add file1.js file2.py
   ```

3. Commit the changes:
   ```bash
   git commit -m "Descriptive commit message"
   ```

## Step 3: Push Changes to GitHub
1. For an existing branch:
   ```bash
   git push origin branch-name
   ```

2. For a new branch (sets up tracking):
   ```bash
   git push -u origin new-branch-name
   ```

## Step 4: Verify Push Success
1. Check that your changes appear on GitHub:
   ```bash
   git log --oneline -5
   ```
   Compare with the commits visible on the GitHub repository page.

## Common Issues and Solutions

### Authentication Error
- **Problem**: Permission denied (publickey) or authentication failed
- **Solution**: Verify SSH key setup or personal access token configuration

### Non-fast-forward Error
- **Problem**: Updates were rejected because the remote contains work that you do not have locally
- **Solution**: Pull the latest changes first:
  ```bash
  git pull origin branch-name
  # Resolve any conflicts if necessary
  git push origin branch-name
  ```

### Branch Doesn't Exist Remotely
- **Problem**: The remote branch doesn't exist yet
- **Solution**: Use the `-u` flag to create the remote branch:
  ```bash
  git push -u origin branch-name
  ```