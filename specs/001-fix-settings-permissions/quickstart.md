# Quickstart: Fix Settings Permissions

## Prerequisites
- Claude Code installed and configured
- Access to `.claude/settings.local.json` file

## Setup
1. Navigate to your project directory
2. Locate the `.claude/settings.local.json` file
3. Verify Claude Code is showing "Found 2 invalid settings files" error

## Fix Permission Patterns
1. Open `.claude/settings.local.json`
2. Find permission patterns with `:*` in the middle
3. Move `:*` to the end of the pattern
4. Save the file

## Remove Urdu Translation
1. Remove any Urdu-related permission patterns from the file
2. Verify all Urdu translation commands are removed
3. Save the file

## Verification
1. Run Claude Code
2. Confirm no "Found 2 invalid settings files" error appears
3. Test that remaining functionality still works as expected