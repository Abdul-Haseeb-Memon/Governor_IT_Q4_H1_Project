# Research: Fix Settings Permissions

## Decision: Claude Code Permission Pattern Requirements
**Rationale**: Claude Code requires permission patterns to have `:*` at the end for proper prefix matching, not in the middle of command patterns.
**Alternatives considered**:
- Keep original patterns (rejected - causes errors)
- Use different wildcard patterns (rejected - doesn't follow Claude Code standards)

## Decision: Removal of Urdu Translation Functionality
**Rationale**: User explicitly requested removal of Urdu translation features to clean up configuration.
**Alternatives considered**:
- Keep Urdu functionality (rejected - goes against user request)
- Disable but keep in config (rejected - user wants complete removal)

## Decision: Configuration File Location
**Rationale**: Claude Code settings are stored in `.claude/settings.local.json` according to Claude Code documentation.
**Alternatives considered**:
- Other configuration files (rejected - not the correct file)
- Environment variables (rejected - Claude Code uses this specific file)

## Decision: Validation Approach
**Rationale**: Will validate fixes by running Claude Code to ensure no "Found 2 invalid settings files" error appears.
**Alternatives considered**:
- Manual JSON validation only (rejected - doesn't test actual functionality)
- External tools (rejected - Claude Code itself is the validator)