---
command: config-clean
description: Remove Claude Code configuration (MCP servers, plugins, marketplaces)
arguments: "[options]"
user_invocable: true
---

# Config Clean Command

Clean up your Claude Code configuration by removing MCP servers, plugins, and/or marketplaces. Supports selective cleanup and dry-run mode.

## Arguments

- `--dry-run` - Show what would be removed without executing
- `--backup` - Backup configs before cleanup (timestamped to `~/.claude/backups/`)
- `--mcp-only` - Only remove MCP servers
- `--plugins-only` - Only uninstall plugins
- `--marketplaces-only` - Only remove marketplaces

Without selective flags, all configuration types are cleaned.

## Instructions

1. Parse the user's arguments to determine which flags to pass to the cleanup script.

2. Run the cleanup script located at `{plugin_dir}/clean_claude_setup.sh` with the appropriate flags.

3. Display the output from the script, which includes:
   - Color-coded status messages
   - What was removed (or would be removed in dry-run mode)
   - Backup location if `--backup` was used

## Example Usage

```
/config-clean --dry-run
/config-clean --backup
/config-clean --mcp-only
/config-clean --plugins-only --backup
```

## Example Output

```
Claude Code Configuration Cleanup
==================================
DRY RUN MODE - No changes will be made

[OK] Dependencies validated (jq, claude)

=== Removing MCP Servers ===
[DRY-RUN] Would: claude mcp remove "context7"
[DRY-RUN] Would: claude mcp remove "playwright"

=== Removing Plugins ===
[DRY-RUN] Would: claude plugin uninstall "superpowers"

=== Removing Marketplaces ===
[DRY-RUN] Would: claude marketplace remove "claude-code-plugins"

Dry run complete. No changes were made.
```

## Error Handling

- If `jq` or `claude` CLI is not installed, the script will exit with an error message
- Failed removals are logged as warnings but don't stop the cleanup process
- In dry-run mode, no changes are made to the configuration
