#!/bin/bash
# Generates claude_setup.sh from current Claude Code configuration

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Config files
MARKETPLACES_FILE="$HOME/.claude/plugins/known_marketplaces.json"
MCP_FILE="$HOME/.claude.json"
SETTINGS_FILE="$HOME/.claude/settings.json"

# Defaults
OUTPUT="claude_setup.sh"
SUMMARY_MODE=false
DRY_RUN=false
NO_CLEANUP=false
MCP_ONLY=false
PLUGINS_ONLY=false
MARKETPLACES_ONLY=false

usage() {
    echo "Usage: $0 [OPTIONS] [output-file]"
    echo ""
    echo "Generate a setup script from current Claude Code configuration."
    echo ""
    echo "Options:"
    echo "  --summary            Only output item counts (for internal use)"
    echo "  --dry-run            Show generated script without writing to file"
    echo "  --no-cleanup         Generate setup without cleanup phase"
    echo "  --mcp-only           Only export MCP servers"
    echo "  --plugins-only       Only export plugins"
    echo "  --marketplaces-only  Only export marketplaces"
    echo "  -h, --help           Show this help message"
    echo ""
    echo "Arguments:"
    echo "  output-file          Output path (default: claude_setup.sh)"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1" >&2
}

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1" >&2
}

# Validate dependencies
validate_dependencies() {
    local missing=()

    if ! command -v jq &> /dev/null; then
        missing+=("jq")
    fi

    if [[ ${#missing[@]} -gt 0 ]]; then
        log_error "Missing required dependencies: ${missing[*]}"
        echo "Please install the missing tools and try again." >&2
        exit 1
    fi
}

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

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --summary)
            SUMMARY_MODE=true
            shift
            ;;
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        --no-cleanup)
            NO_CLEANUP=true
            shift
            ;;
        --mcp-only)
            MCP_ONLY=true
            shift
            ;;
        --plugins-only)
            PLUGINS_ONLY=true
            shift
            ;;
        --marketplaces-only)
            MARKETPLACES_ONLY=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        -*)
            log_error "Unknown option: $1"
            usage
            exit 1
            ;;
        *)
            OUTPUT="$1"
            shift
            ;;
    esac
done

# Validate dependencies
validate_dependencies

# Determine what to export
EXPORT_MCP=true
EXPORT_PLUGINS=true
EXPORT_MARKETPLACES=true

# If any selective flag is set, only export those
if [[ "$MCP_ONLY" == true || "$PLUGINS_ONLY" == true || "$MARKETPLACES_ONLY" == true ]]; then
    EXPORT_MCP=$MCP_ONLY
    EXPORT_PLUGINS=$PLUGINS_ONLY
    EXPORT_MARKETPLACES=$MARKETPLACES_ONLY
fi

# If summary mode, just output counts
if [[ "$SUMMARY_MODE" == true ]]; then
    [[ "$EXPORT_MARKETPLACES" == true ]] && echo "marketplaces:$(count_marketplaces)"
    [[ "$EXPORT_MCP" == true ]] && echo "mcp_servers:$(count_mcp_servers)"
    [[ "$EXPORT_PLUGINS" == true ]] && echo "plugins:$(count_plugins)"
    exit 0
fi

# Check if any config exists
MARKETPLACE_COUNT=0
MCP_COUNT=0
PLUGIN_COUNT=0

[[ "$EXPORT_MARKETPLACES" == true ]] && MARKETPLACE_COUNT=$(count_marketplaces)
[[ "$EXPORT_MCP" == true ]] && MCP_COUNT=$(count_mcp_servers)
[[ "$EXPORT_PLUGINS" == true ]] && PLUGIN_COUNT=$(count_plugins)

if [[ "$MARKETPLACE_COUNT" == "0" && "$MCP_COUNT" == "0" && "$PLUGIN_COUNT" == "0" ]]; then
    log_error "No configuration found to export."
    exit 1
fi

# Generate the setup script content
generate_script() {
    cat << 'HEADER'
#!/bin/bash
# Claude Code setup script - Marketplaces, Plugins, and MCP Servers
# Generated on: GENERATED_DATE

set -e

HEADER

    # Add cleanup section unless --no-cleanup is set
    if [[ "$NO_CLEANUP" == false ]]; then
        if [[ "$EXPORT_MCP" == true ]]; then
            cat << 'MCP_CLEANUP'
echo "=== Removing existing MCP Servers ==="
if [[ -f "$HOME/.claude.json" ]]; then
    for server in $(jq -r '.mcpServers | keys[]' "$HOME/.claude.json" 2>/dev/null); do
        claude mcp remove "$server" || true
    done
fi

MCP_CLEANUP
        fi

        if [[ "$EXPORT_PLUGINS" == true ]]; then
            cat << 'PLUGIN_CLEANUP'
echo "=== Removing existing Plugins ==="
for plugin in $(claude plugin list --json 2>/dev/null | jq -r '.[].id // empty'); do
    claude plugin uninstall "$plugin" || true
done

PLUGIN_CLEANUP
        fi

        if [[ "$EXPORT_MARKETPLACES" == true ]]; then
            cat << 'MARKETPLACE_CLEANUP'
echo "=== Removing existing Marketplaces ==="
for marketplace in $(claude marketplace list --json 2>/dev/null | jq -r '.[].name // empty'); do
    claude marketplace remove "$marketplace" || true
done

MARKETPLACE_CLEANUP
        fi
    fi

    # Add marketplaces
    if [[ "$EXPORT_MARKETPLACES" == true ]]; then
        echo 'echo "=== Adding Marketplaces ==="'
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
            ' "$MARKETPLACES_FILE" 2>/dev/null || true
        fi
        echo ''
    fi

    # Add MCP servers
    if [[ "$EXPORT_MCP" == true ]]; then
        echo 'echo "=== Adding MCP Servers ==="'
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
            ' "$MCP_FILE" 2>/dev/null || true
        fi
        echo ''
    fi

    # Add plugins
    if [[ "$EXPORT_PLUGINS" == true ]]; then
        echo 'echo "=== Installing Plugins ==="'
        if [[ -f "$SETTINGS_FILE" ]]; then
            jq -r '
              .enabledPlugins | to_entries[] |
              select(.value == true) |
              "claude plugin install \(.key)"
            ' "$SETTINGS_FILE" 2>/dev/null || true
        fi
        echo ''
    fi

    echo 'echo "=== Done ==="'
}

# Generate script content with date substitution
script_content=$(generate_script | sed "s/GENERATED_DATE/$(date)/")

# Output or write the script
if [[ "$DRY_RUN" == true ]]; then
    echo -e "${YELLOW}=== Generated Script (dry-run) ===${NC}" >&2
    echo ""
    echo "$script_content"
    echo ""
    echo -e "${YELLOW}=== End of Script ===${NC}" >&2
else
    echo "$script_content" > "$OUTPUT"
    chmod +x "$OUTPUT"
    echo "Generated: $OUTPUT"
fi

# Output summary
[[ "$EXPORT_MARKETPLACES" == true ]] && echo "marketplaces:$MARKETPLACE_COUNT"
[[ "$EXPORT_MCP" == true ]] && echo "mcp_servers:$MCP_COUNT"
[[ "$EXPORT_PLUGINS" == true ]] && echo "plugins:$PLUGIN_COUNT"
