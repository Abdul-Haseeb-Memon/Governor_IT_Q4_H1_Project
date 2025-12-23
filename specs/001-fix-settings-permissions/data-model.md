# Data Model: Fix Settings Permissions

## ClaudeCodeSettings Entity

### Fields
- **permissions.allow**: Array of permission pattern strings that define allowed bash commands
- **permissions.deny**: Array of permission pattern strings that define denied bash commands
- **permissions.ask**: Array of permission pattern strings that require user confirmation

### Relationships
- Each permission pattern string follows Claude Code's security requirements
- Patterns must have `:*` at the end for proper prefix matching

### Validation Rules
- Permission patterns with `:*` in the middle are invalid
- Permission patterns with `:*` at the end are valid
- Patterns must follow Claude Code's documented format

### State Transitions
- Invalid permission patterns → Error state ("Found 2 invalid settings files")
- Valid permission patterns → Normal operation state

## PermissionPattern Entity

### Fields
- **pattern**: String representing the bash command pattern
- **type**: Type of command (e.g., "move", "powershell", "bash")
- **scope**: Scope of the permission (e.g., specific file paths, general commands)

### Validation Rules
- Must end with `:*` for prefix matching
- Must not have wildcards in the middle that violate security requirements
- Must match actual command structure