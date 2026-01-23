#!/bin/bash
# ROS Master Discovery - Network scanning version (no sudo required)
# Outputs JSON: {"masters": [...], "external": [...], "local": "..."}
#   - masters: external IPs + one preferred local IP
#   - external: IPs from other machines
#   - local: preferred local IP (192.168.x > 10.x > 127.x)
# Uses parallel bash /dev/tcp connections

PORT=11311
TIMEOUT=0.5

# Get network ranges from ip command (filter Docker/loopback)
NETWORK_RANGES=$(ip -4 addr show 2>/dev/null | \
    grep -oP '(?<=inet\s)\d+\.\d+\.\d+\.\d+/\d+' | \
    grep -v '^172\.' | \
    grep -v '^127\.' | \
    sort -u)

# Get all local IPs for this machine (used to distinguish local vs external masters)
LOCAL_IPS=$(ip -4 addr show 2>/dev/null | \
    grep -oP '(?<=inet\s)\d+\.\d+\.\d+\.\d+' | \
    sort -u)

# Function to extract subnet base (e.g., 192.168.0 from 192.168.0.121/24)
get_subnet_base() {
    echo "$1" | cut -d'/' -f1 | cut -d'.' -f1-3
}

# Check if an IP belongs to this machine
is_local_ip() {
    local ip="$1"
    [[ "$ip" == "127.0.0.1" ]] && return 0
    echo "$LOCAL_IPS" | grep -qx "$ip"
}

# Collect all found masters
ALL_FOUND_IPS=$({
    # Check localhost first
    timeout $TIMEOUT bash -c "echo > /dev/tcp/127.0.0.1/$PORT" 2>/dev/null && echo "127.0.0.1"

    # Scan each network range
    for RANGE in $NETWORK_RANGES; do
        SUBNET=$(get_subnet_base "$RANGE")
        # Scan all 254 hosts in parallel
        for i in {1..254}; do
            (timeout $TIMEOUT bash -c "echo > /dev/tcp/$SUBNET.$i/$PORT" 2>/dev/null && echo "$SUBNET.$i") &
        done
    done
    wait
} 2>/dev/null | sort -u -t. -k1,1n -k2,2n -k3,3n -k4,4n)

# Separate into local vs external and output JSON
{
    LOCAL_MASTERS=()
    EXTERNAL_MASTERS=()

    while IFS= read -r ip; do
        [ -z "$ip" ] && continue
        if is_local_ip "$ip"; then
            LOCAL_MASTERS+=("$ip")
        else
            EXTERNAL_MASTERS+=("$ip")
        fi
    done <<< "$ALL_FOUND_IPS"

    # Select preferred local IP (prefer 192.168.x > 10.x > others)
    PREFERRED_LOCAL=""
    for ip in "${LOCAL_MASTERS[@]}"; do
        if [[ "$ip" =~ ^192\.168\. ]]; then
            PREFERRED_LOCAL="$ip"
            break
        fi
    done
    if [ -z "$PREFERRED_LOCAL" ]; then
        for ip in "${LOCAL_MASTERS[@]}"; do
            if [[ "$ip" =~ ^10\. ]]; then
                PREFERRED_LOCAL="$ip"
                break
            fi
        done
    fi
    if [ -z "$PREFERRED_LOCAL" ] && [ ${#LOCAL_MASTERS[@]} -gt 0 ]; then
        PREFERRED_LOCAL="${LOCAL_MASTERS[0]}"
    fi

    # Build masters array: external + preferred local
    MASTERS=("${EXTERNAL_MASTERS[@]}")
    [ -n "$PREFERRED_LOCAL" ] && MASTERS+=("$PREFERRED_LOCAL")

    # Output JSON
    printf '{'

    # masters array
    printf '"masters": ['
    first=true
    for ip in "${MASTERS[@]}"; do
        [ "$first" = true ] && first=false || printf ','
        printf '"%s"' "$ip"
    done
    printf ']'

    # external array
    printf ', "external": ['
    first=true
    for ip in "${EXTERNAL_MASTERS[@]}"; do
        [ "$first" = true ] && first=false || printf ','
        printf '"%s"' "$ip"
    done
    printf ']'

    # local (single preferred)
    if [ -n "$PREFERRED_LOCAL" ]; then
        printf ', "local": "%s"' "$PREFERRED_LOCAL"
    else
        printf ', "local": null'
    fi

    printf '}\n'
}
