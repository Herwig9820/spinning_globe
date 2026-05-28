#!/bin/bash
# ============================================================
# nr_snapshot.sh — capture Node-RED application state
# ============================================================
# PURPOSE: Back up all Node-RED application-level config:
#   - All flow files (*.json) — the actual logic/wiring
#   - Persistent context storage (context/) — runtime variables
#     that survive restarts (requires contextStorage in settings)
#   - package.json + package-lock.json — installed extra nodes
#   - Non-secret excerpts from settings.js
#
# SCOPE: Application config only — NOT the OS, NOT the nodered
#   systemd service itself (that's in pi_snapshot.sh).
#
# SENSITIVITY: Flow files may contain MQTT credentials embedded
#   in node configs. Treat the snapshot folder as sensitive.
#   Full settings.js is NOT exported (contains secrets).
#
# Usage:
#   ./scripts/nr_snapshot.sh           → saves to ~/nr_snapshot/
#   ./scripts/nr_snapshot.sh /tmp/foo  → saves to /tmp/foo/
#
# Designed to be re-run before any flow change. Each run
# overwrites the previous snapshot in the same folder.
# ============================================================

NR_DIR="$HOME/.node-red"
OUTDIR="${1:-$HOME/nr_snapshot}"

# Defensive: refuse to wipe dangerous paths
if [ -z "$OUTDIR" ] || [ "$OUTDIR" = "/" ] || [ "$OUTDIR" = "$HOME" ]; then
    echo "ERROR: refusing to clear suspicious OUTDIR='$OUTDIR'"
    exit 1
fi

# Verify Node-RED directory exists
if [ ! -d "$NR_DIR" ]; then
    echo "ERROR: Node-RED directory not found at $NR_DIR"
    exit 1
fi

# Clear any previous snapshot to ensure no stale files
if [ -d "$OUTDIR" ]; then
    rm -rf "$OUTDIR"
fi
mkdir -p "$OUTDIR"

# ============================================================
# Snapshot metadata
# ============================================================
{
    echo "Snapshot created: $(date)"
    echo "Hostname: $(hostname)"
    echo "Node-RED source: $NR_DIR"
    echo "Node-RED version: $(node-red --version 2>/dev/null || echo 'unknown')"
    echo "Service status: $(systemctl is-active nodered 2>/dev/null)"
} > "$OUTDIR/_snapshot_info.txt"

# ============================================================
# Flow files — the actual Node-RED logic
# ============================================================
# All *.json files in the Node-RED directory are flow files.
# This includes the main flows, credential store, and any
# sub-flow or project files.
echo "Capturing flow files..."
mkdir -p "$OUTDIR/flows"

for f in "$NR_DIR"/*.json; do
    [ -f "$f" ] || continue
    fname=$(basename "$f")
    # Redact credential files (contain encrypted secrets)
    if echo "$fname" | grep -qiE "cred"; then
        echo "  skipped (credentials): $fname"
    else
        cp "$f" "$OUTDIR/flows/"
        echo "  captured: $fname"
    fi
done

# ============================================================
# Persistent context storage
# ============================================================
# Only exists if contextStorage is enabled in settings.js.
# Contains runtime variables that survive Node-RED restarts.
echo "Capturing persistent context storage..."
if [ -d "$NR_DIR/context" ]; then
    mkdir -p "$OUTDIR/context"
    cp -r "$NR_DIR/context/." "$OUTDIR/context/"
    file_count=$(find "$OUTDIR/context" -type f | wc -l)
    echo "  captured $file_count context files"
else
    echo "  no context/ directory found (contextStorage may not be active)"
fi

# ============================================================
# Installed extra nodes
# ============================================================
# package.json lists all installed community nodes.
# Required to restore the exact same node set after a rebuild.
echo "Capturing installed nodes list..."
if [ -f "$NR_DIR/package.json" ]; then
    cp "$NR_DIR/package.json" "$OUTDIR/"
    echo "  captured: package.json"
fi
if [ -f "$NR_DIR/package-lock.json" ]; then
    cp "$NR_DIR/package-lock.json" "$OUTDIR/"
    echo "  captured: package-lock.json"
fi

# Human-readable list of installed nodes
if [ -f "$NR_DIR/package.json" ]; then
    {
        echo "=== Installed Node-RED nodes ==="
        grep -A 999 '"dependencies"' "$NR_DIR/package.json" 2>/dev/null
    } > "$OUTDIR/installed_nodes.txt"
fi

# ============================================================
# settings.js — non-secret excerpts only
# ============================================================
# Full settings.js is NOT copied (contains credentialSecret
# and adminAuth password hash). Only safe keys are extracted.
echo "Capturing settings.js excerpts..."
{
    echo "=== Relevant non-secret settings.js excerpts ==="
    echo "(Full settings.js NOT exported — contains secrets)"
    echo ""
    grep -E "^\s*(uiHost|uiPort|contextStorage|adminAuth|credentialSecret|flowFile)" \
        "$NR_DIR/settings.js" 2>/dev/null \
        | sed 's/credentialSecret:.*/credentialSecret: <REDACTED>/' \
        | sed 's/password:.*/password: <REDACTED>/'
} > "$OUTDIR/settings_excerpts.txt"

# ============================================================
# Directory listing of Node-RED dir (for reference)
# ============================================================
ls -la "$NR_DIR" > "$OUTDIR/_nr_dir_listing.txt" 2>&1

# ============================================================
# Done
# ============================================================
echo ""
echo "Node-RED snapshot saved to: $OUTDIR"
echo ""
ls -la "$OUTDIR"
