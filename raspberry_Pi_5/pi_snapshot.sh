#!/bin/bash
# ============================================================
# pi_snapshot.sh — capture current state of the Raspberry Pi
# ============================================================
# Records system info, network config, service status, and 
# selected configuration files into a snapshot folder.
# 
# Usage:
#   ./pi_snapshot.sh           → saves to ~/pi_snapshot/
#   ./pi_snapshot.sh /tmp/foo  → saves to /tmp/foo/
#
# Designed to be re-run periodically. Each run overwrites
# the previous snapshot in the same folder.
# 
# To extend (e.g., when Zigbee2MQTT is added), add new sections
# below following the same pattern.
# ============================================================

OUTDIR="${1:-$HOME/pi_snapshot}"
mkdir -p "$OUTDIR"

# ============================================================
# Helper functions
# ============================================================
# Capture a directory listing (ls -la) into the corresponding
# snapshot subfolder. Useful as a "what files were present at
# snapshot time" diagnostic.
# Usage: snapshot_listing <source-dir> <snapshot-subfolder>
snapshot_listing() {
    local src="$1"
    local dest="$2"
    if [ -d "$src" ]; then
        ls -la "$src" > "$OUTDIR/$dest/_directory_listing.txt" 2>&1
    else
        echo "Directory does not exist: $src" > "$OUTDIR/$dest/_directory_listing.txt"
    fi
}

# ============================================================
# Snapshot metadata
# ============================================================
echo "Snapshot created: $(date)" > "$OUTDIR/_snapshot_info.txt"
echo "Hostname: $(hostname)" >> "$OUTDIR/_snapshot_info.txt"

echo ""
echo "Capturing system info..."

# ============================================================
# System info
# ============================================================
{
    echo "=== /etc/os-release ==="
    cat /etc/os-release
    echo ""
    echo "=== uname -a ==="
    uname -a
    echo ""
    echo "=== hostnamectl ==="
    hostnamectl
    echo ""
    echo "=== Memory (free -h) ==="
    free -h
    echo ""
    echo "=== Disk (df -h) ==="
    df -h
    echo ""
    echo "=== Uptime ==="
    uptime
} > "$OUTDIR/system_info.txt"

# ============================================================
# Network config (no passwords)
# ============================================================
echo "Capturing network config..."
{
    echo "=== Known connections (nmcli) ==="
    nmcli connection show
    echo ""
    echo "=== Active connections ==="
    nmcli connection show --active
    echo ""
    echo "=== IP addresses ==="
    ip addr show
    echo ""
    echo "=== Routes ==="
    ip route show
    echo ""
    echo "=== /etc/hosts ==="
    cat /etc/hosts
} > "$OUTDIR/network_config.txt"

# ============================================================
# Tailscale
# ============================================================
echo "Capturing Tailscale status..."
{
    echo "=== tailscale status ==="
    tailscale status 2>&1
    echo ""
    echo "=== tailscale ip -4 ==="
    tailscale ip -4 2>&1
} > "$OUTDIR/tailscale_status.txt"

# ============================================================
# systemd services we care about
# ============================================================
echo "Capturing service status..."
SERVICES="tailscaled mosquitto nodered"
{
    for svc in $SERVICES; do
        echo "=== $svc ==="
        echo "is-enabled: $(systemctl is-enabled $svc 2>&1)"
        echo "is-active:  $(systemctl is-active $svc 2>&1)"
        echo ""
    done
} > "$OUTDIR/services_status.txt"

# ============================================================
# Mosquitto config
# ============================================================
echo "Capturing Mosquitto config..."
mkdir -p "$OUTDIR/mosquitto"
snapshot_listing "/etc/mosquitto" "mosquitto"
sudo cat /etc/mosquitto/mosquitto.conf > "$OUTDIR/mosquitto/mosquitto.conf" 2>/dev/null
sudo cp /etc/mosquitto/conf.d/*.conf "$OUTDIR/mosquitto/" 2>/dev/null

# ============================================================
# Node-RED config
# ============================================================
echo "Capturing Node-RED config..."
mkdir -p "$OUTDIR/nodered"
snapshot_listing "$HOME/.node-red" "nodered"

cp "$HOME/.node-red/Spinning globe flows.json" "$OUTDIR/nodered/" 2>/dev/null
cp "$HOME/.node-red/package.json" "$OUTDIR/nodered/" 2>/dev/null
cp "$HOME/.node-red/package-lock.json" "$OUTDIR/nodered/" 2>/dev/null

# Selected non-secret excerpts from settings.js (don't commit the whole file)
{
    echo "=== Relevant non-secret settings.js excerpts ==="
    echo ""
    grep -E "^\s*(uiHost|contextStorage|credentialSecret|adminAuth)" ~/.node-red/settings.js 2>/dev/null \
        | sed 's/credentialSecret:.*/credentialSecret: <REDACTED>/' \
        | sed 's/password:.*/password: <REDACTED>/'
    echo ""
    echo "(Note: full settings.js is NOT exported because it contains secrets.)"
} > "$OUTDIR/nodered/settings_excerpts.txt"

# ============================================================
# Installed packages
# ============================================================
echo "Capturing installed packages list..."
apt list --installed 2>/dev/null > "$OUTDIR/apt_packages.txt"

# ============================================================
# Listening ports (useful for "what's actually running")
# ============================================================
echo "Capturing listening ports..."
sudo ss -tlnp > "$OUTDIR/listening_ports.txt" 2>&1

# ============================================================
# Done
# ============================================================
echo ""
echo "Snapshot saved to: $OUTDIR"
echo ""
ls -la "$OUTDIR"