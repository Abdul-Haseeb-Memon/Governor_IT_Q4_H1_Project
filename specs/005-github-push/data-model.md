# Data Model: GitHub Push Workflow

## Git Repository Entity
- **name**: Repository name
- **fields**:
  - local_path (string): Local file system path to the repository
  - remote_url (string): URL of the remote GitHub repository
  - status (enum): Current status (clean, modified, staged, conflicted)
  - branch (string): Current active branch
- **relationships**: Contains multiple Git Branch entities
- **validation**: Must have a valid Git configuration

## Git Branch Entity
- **name**: Branch
- **fields**:
  - name (string): Branch name
  - remote_tracking (string): Remote tracking branch name (e.g., origin/main)
  - commit_hash (string): SHA of the latest commit on this branch
  - is_current (boolean): Whether this is the currently checked out branch
- **relationships**: Belongs to Git Repository, contains multiple Commit entities
- **validation**: Branch name must follow Git naming conventions

## Commit Entity
- **name**: Commit
- **fields**:
  - hash (string): SHA identifier of the commit
  - message (string): Commit message
  - author (string): Author of the commit
  - timestamp (datetime): When the commit was created
  - parent_hashes (array): SHAs of parent commits
- **relationships**: Belongs to Git Branch
- **validation**: Must have a valid commit hash format

## Remote Configuration Entity
- **name**: Remote Config
- **fields**:
  - name (string): Remote name (e.g., origin, upstream)
  - fetch_url (string): URL for fetching from remote
  - push_url (string): URL for pushing to remote
  - last_fetched (datetime): When the remote was last fetched
- **relationships**: Associated with Git Repository
- **validation**: URLs must be valid Git repository URLs

## Push Operation Entity
- **name**: Push Operation
- **fields**:
  - source_branch (string): Local branch being pushed
  - destination_branch (string): Remote branch to push to
  - repository (string): Repository name
  - status (enum): Operation status (pending, success, failed)
  - timestamp (datetime): When the operation was initiated
  - error_message (string, optional): Error details if operation failed
- **relationships**: Associated with Git Branch and Remote Configuration
- **validation**: Source and destination branches must exist