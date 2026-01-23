#!/bin/bash
# Clean up Claude Code configuration - remove MCP servers, plugins, and marketplaces
# Standalone cleanup script with options for selective removal

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Config files
MCP_FILE="$HOME/.claude.json"
SETTINGS_FILE="$HOME/.claude/settings.json"
MARKETPLACES_FILE="$HOME/.claude/plugins/known_marketplaces.json"

# Flags
DRY_RUN=false
BACKUP=false
MCP_ONLY=false
PLUGINS_ONLY=false
MARKETPLACES_ONLY=false

usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Clean up Claude Code configuration by removing MCP servers, plugins, and marketplaces."
    echo ""
    echo "Options:"
    echo "  --dry-run            Show what would be removed without executing"
    echo "  --backup             Backup configs before cleanup (timestamped)"
    echo "  --mcp-only           Only remove MCP servers"
    echo "  --plugins-only       Only uninstall plugins"
    echo "  --marketplaces-only  Only remove marketplaces"
    echo "  -h, --help           Show this help message"
    echo ""
    echo "Without selective flags, all configuration types are cleaned."
}

log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[OK]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

log_dry() {
    echo -e "${YELLOW}[DRY-RUN]${NC} Would: $1"
}

# Validate dependencies
validate_dependencies() {
    local missing=()

    if ! command -v jq &> /dev/null; then
        missing+=("jq")
    fi

    if ! command -v claude &> /dev/null; then
        missing+=("claude")
    fi

    if [[ ${#missing[@]} -gt 0 ]]; then
        log_error "Missing required dependencies: ${missing[*]}"
        echo "Please install the missing tools and try again."
        exit 1
    fi

    log_success "Dependencies validated (jq, claude)"
}

# Backup configuration files
backup_configs() {
    local timestamp=$(date +%Y%m%d_%H%M%S)
    local backup_dir="$HOME/.claude/backups/config_$timestamp"

    log_info "Creating backup in $backup_dir"
    mkdir -p "$backup_dir"

    if [[ -f "$MCP_FILE" ]]; then
        cp "$MCP_FILE" "$backup_dir/claude.json"
        log_success "Backed up MCP config"
    fi

    if [[ -f "$SETTINGS_FILE" ]]; then
        cp "$SETTINGS_FILE" "$backup_dir/settings.json"
        log_success "Backed up settings"
    fi

    if [[ -f "$MARKETPLACES_FILE" ]]; then
        cp "$MARKETPLACES_FILE" "$backup_dir/known_marketplaces.json"
        log_success "Backed up marketplaces"
    fi

    echo "$backup_dir"
}

# Remove MCP servers
clean_mcp_servers() {
    echo ""
    echo -e "${BLUE}=== Removing MCP Servers ===${NC}"

    if [[ ! -f "$MCP_FILE" ]]; then
        log_warn "No MCP config file found at $MCP_FILE"
        return
    fi

    local servers=$(jq -r '.mcpServers | keys[]' "$MCP_FILE" 2>/dev/null)

    if [[ -z "$servers" ]]; then
        log_info "No MCP servers configured"
        return
    fi

    while IFS= read -r server; do
        if [[ -n "$server" ]]; then
            if [[ "$DRY_RUN" == true ]]; then
                log_dry "claude mcp remove \"$server\""
            else
                if claude mcp remove "$server" 2>/dev/null; then
                    log_success "Removed MCP server: $server"
                else
                    log_warn "Failed to remove MCP server: $server"
                fi
            fi
        fi
    done <<< "$servers"
}

# Remove plugins
clean_plugins() {
    echo ""
    echo -e "${BLUE}=== Removing Plugins ===${NC}"

    local plugins=$(claude plugin list --json 2>/dev/null | jq -r '.[].id // empty')

    if [[ -z "$plugins" ]]; then
        log_info "No plugins installed"
        return
    fi

    while IFS= read -r plugin; do
        if [[ -n "$plugin" ]]; then
            if [[ "$DRY_RUN" == true ]]; then
                log_dry "claude plugin uninstall \"$plugin\""
            else
                if claude plugin uninstall "$plugin" 2>/dev/null; then
                    log_success "Uninstalled plugin: $plugin"
                else
                    log_warn "Failed to uninstall plugin: $plugin"
                fi
            fi
        fi
    done <<< "$plugins"
}

# Remove marketplaces
clean_marketplaces() {
    echo ""
    echo -e "${BLUE}=== Removing Marketplaces ===${NC}"

    local marketplaces=$(claude plugin marketplace list --json 2>/dev/null | jq -r '.[].name // empty')

    if [[ -z "$marketplaces" ]]; then
        log_info "No marketplaces registered"
        return
    fi

    while IFS= read -r marketplace; do
        if [[ -n "$marketplace" ]]; then
            if [[ "$DRY_RUN" == true ]]; then
                log_dry "claude plugin marketplace remove \"$marketplace\""
            else
                if claude plugin marketplace remove "$marketplace" 2>/dev/null; then
                    log_success "Removed marketplace: $marketplace"
                else
                    log_warn "Failed to remove marketplace: $marketplace"
                fi
            fi
        fi
    done <<< "$marketplaces"
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        --backup)
            BACKUP=true
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
        *)
            log_error "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

# Determine what to clean
CLEAN_MCP=true
CLEAN_PLUGINS=true
CLEAN_MARKETPLACES=true

# If any selective flag is set, only clean those
if [[ "$MCP_ONLY" == true || "$PLUGINS_ONLY" == true || "$MARKETPLACES_ONLY" == true ]]; then
    CLEAN_MCP=$MCP_ONLY
    CLEAN_PLUGINS=$PLUGINS_ONLY
    CLEAN_MARKETPLACES=$MARKETPLACES_ONLY
fi

# Main execution
echo -e "${BLUE}Claude Code Configuration Cleanup${NC}"
echo "=================================="

if [[ "$DRY_RUN" == true ]]; then
    echo -e "${YELLOW}DRY RUN MODE - No changes will be made${NC}"
    echo ""
fi

validate_dependencies

if [[ "$BACKUP" == true && "$DRY_RUN" == false ]]; then
    backup_dir=$(backup_configs)
    echo -e "${GREEN}Backup created: $backup_dir${NC}"
fi

# Run cleanup based on flags
[[ "$CLEAN_MCP" == true ]] && clean_mcp_servers
[[ "$CLEAN_PLUGINS" == true ]] && clean_plugins
[[ "$CLEAN_MARKETPLACES" == true ]] && clean_marketplaces

echo ""
if [[ "$DRY_RUN" == true ]]; then
    echo -e "${YELLOW}Dry run complete. No changes were made.${NC}"
else
    echo -e "${GREEN}Cleanup complete!${NC}"
fi
