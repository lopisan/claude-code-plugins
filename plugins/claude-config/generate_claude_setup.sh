#!/bin/bash
# Generates claude_setup.sh from current Claude Code configuration

set -e

OUTPUT="${1:-claude_setup.sh}"
SUMMARY_MODE="${2:-}"

MARKETPLACES_FILE="$HOME/.claude/plugins/known_marketplaces.json"
MCP_FILE="$HOME/.claude.json"
SETTINGS_FILE="$HOME/.claude/settings.json"

# Count items for summary
count_marketplaces() {
    if [[ -f "$MARKETPLACES_FILE" ]]; then
        jq 'to_entries | length' "$MARKETPLACES_FILE" 2>/dev/null || echo "0"
    else
        echo "0"
    fi
}

count_mcp_servers() {
    if [[ -f "$MCP_FILE" ]]; then
        jq '.mcpServers | to_entries | length' "$MCP_FILE" 2>/dev/null || echo "0"
    else
        echo "0"
    fi
}

count_plugins() {
    if [[ -f "$SETTINGS_FILE" ]]; then
        jq '[.enabledPlugins | to_entries[] | select(.value == true)] | length' "$SETTINGS_FILE" 2>/dev/null || echo "0"
    else
        echo "0"
    fi
}

# If summary mode, just output counts
if [[ "$SUMMARY_MODE" == "--summary" ]]; then
    echo "marketplaces:$(count_marketplaces)"
    echo "mcp_servers:$(count_mcp_servers)"
    echo "plugins:$(count_plugins)"
    exit 0
fi

# Check if any config exists
MARKETPLACE_COUNT=$(count_marketplaces)
MCP_COUNT=$(count_mcp_servers)
PLUGIN_COUNT=$(count_plugins)

if [[ "$MARKETPLACE_COUNT" == "0" && "$MCP_COUNT" == "0" && "$PLUGIN_COUNT" == "0" ]]; then
    echo "No configuration found to export."
    exit 1
fi

# Generate the setup script
cat > "$OUTPUT" << 'HEADER'
#!/bin/bash
# Claude Code setup script - Marketplaces, Plugins, and MCP Servers
# Generated on: GENERATED_DATE

set -e

echo "=== Removing existing MCP Servers ==="
for server in $(claude mcp list 2>/dev/null | grep -oP '^\w+(?=:)'); do
    claude mcp remove "$server" || true
done

echo "=== Removing existing Plugins ==="
for plugin in $(claude plugin list 2>/dev/null | grep -oP '^[\w-]+@[\w-]+'); do
    claude plugin uninstall "$plugin" || true
done

echo "=== Removing existing Marketplaces ==="
for marketplace in $(claude plugin marketplace list 2>/dev/null | grep -oP '^\w[\w-]*(?=\s)'); do
    claude plugin marketplace remove "$marketplace" || true
done

HEADER

# Replace date placeholder
sed -i "s/GENERATED_DATE/$(date)/" "$OUTPUT"

# Add marketplaces
echo 'echo "=== Adding Marketplaces ==="' >> "$OUTPUT"
if [[ -f "$MARKETPLACES_FILE" ]]; then
    jq -r '
      to_entries[] |
      if .value.source.source == "git" then
        "claude marketplace add \(.value.source.url)"
      elif .value.source.source == "github" then
        "claude marketplace add \(.value.source.repo)"
      else
        empty
      end
    ' "$MARKETPLACES_FILE" >> "$OUTPUT" 2>/dev/null || true
fi

# Add MCP servers
echo '' >> "$OUTPUT"
echo 'echo "=== Adding MCP Servers ==="' >> "$OUTPUT"
if [[ -f "$MCP_FILE" ]]; then
    jq -r '
      .mcpServers | to_entries[] |
      if .value.type == "http" then
        "claude mcp add \(.key) --type http --url \"\(.value.url)\""
      elif .value.type == "stdio" then
        "claude mcp add \(.key) -- \(.value.command) \(.value.args | join(" "))"
      else
        empty
      end
    ' "$MCP_FILE" >> "$OUTPUT" 2>/dev/null || true
fi

# Add plugins
echo '' >> "$OUTPUT"
echo 'echo "=== Installing Plugins ==="' >> "$OUTPUT"
if [[ -f "$SETTINGS_FILE" ]]; then
    jq -r '
      .enabledPlugins | to_entries[] |
      select(.value == true) |
      "claude plugin install \(.key)"
    ' "$SETTINGS_FILE" >> "$OUTPUT" 2>/dev/null || true
fi

echo '' >> "$OUTPUT"
echo 'echo "=== Done ==="' >> "$OUTPUT"

chmod +x "$OUTPUT"

# Output summary
echo "Generated: $OUTPUT"
echo "marketplaces:$MARKETPLACE_COUNT"
echo "mcp_servers:$MCP_COUNT"
echo "plugins:$PLUGIN_COUNT"
