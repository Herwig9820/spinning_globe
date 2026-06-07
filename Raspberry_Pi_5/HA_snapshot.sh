#!/bin/bash
# ============================================================
# ha_snapshot.sh — capture Home Assistant configuration state
# ============================================================
# PURPOSE: Back up all Home Assistant application-level config:
#   - Automations, scripts, scenes, dashboards
#   - Device & entity registry (.storage/)
#   - MQTT integration config
#   - Home zone / location settings
#   - Main configuration.yaml
#   - Custom components (HACS integrations, e.g. ha-composite-tracker)
#   - Blueprints
#
# SCOPE: Application config only — NOT the OS, NOT Docker itself,
#   NOT Mosquitto or Zigbee2MQTT (those are in pi_snapshot.sh).
#
# SENSITIVITY: .storage/ contains auth tokens and integration
#   credentials. Treat the snapshot folder as sensitive.
#   Secrets are redacted where possible but .storage/ is copied
#   as-is — do not commit to a public git repo.
#
# Usage:
#   ./scripts/ha_snapshot.sh           → saves to ~/ha_snapshot/
#   ./scripts/ha_snapshot.sh /tmp/foo  → saves to /tmp/foo/
#
# Designed to be re-run before any HA change. Each run overwrites
# the previous snapshot in the same folder.
# ============================================================

HA_CONFIG="/opt/homeassistant/config"
OUTDIR="${1:-$HOME/ha_snapshot}"

# Defensive: refuse to wipe dangerous paths
if [ -z "$OUTDIR" ] || [ "$OUTDIR" = "/" ] || [ "$OUTDIR" = "$HOME" ]; then
    echo "ERROR: refusing to clear suspicious OUTDIR='$OUTDIR'"
    exit 1
fi

# Verify HA config directory exists
if [ ! -d "$HA_CONFIG" ]; then
    echo "ERROR: HA config directory not found at $HA_CONFIG"
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
    echo "HA config source: $HA_CONFIG"
    echo "HA container status: $(docker inspect --format '{{.State.Status}}' homeassistant 2>/dev/null || echo 'unknown')"
    echo "HA image: $(docker inspect --format '{{.Config.Image}}' homeassistant 2>/dev/null || echo 'unknown')"
} > "$OUTDIR/_snapshot_info.txt"

# ============================================================
# Top-level YAML config files
# ============================================================
echo "Capturing HA YAML config files..."
mkdir -p "$OUTDIR/yaml"

for f in configuration.yaml automations.yaml scripts.yaml scenes.yaml groups.yaml \
          customize.yaml input_boolean.yaml input_select.yaml input_number.yaml; do
    if [ -f "$HA_CONFIG/$f" ]; then
        # Redact any api_key, token, password lines
        sed -E 's/(api_key|token|password|secret):.*/\1: <REDACTED>/gi' \
            "$HA_CONFIG/$f" > "$OUTDIR/yaml/$f"
        echo "  captured: $f"
    fi
done

# ============================================================
# .storage/ — device registry, entity registry, dashboards,
#             MQTT config, user data, zone/location settings
# ============================================================
# This is the most critical folder — losing it means
# reconfiguring all integrations, devices and dashboards.
# Copied as-is; may contain tokens (treat as sensitive).
echo "Capturing .storage/ (device registry, integrations, dashboards)..."
if [ -d "$HA_CONFIG/.storage" ]; then
    mkdir -p "$OUTDIR/storage"
    # Copy all storage files
    sudo cp -r "$HA_CONFIG/.storage/." "$OUTDIR/storage/"
    file_count=$(find "$OUTDIR/storage" -type f | wc -l)
    echo "  captured $file_count storage files"
else
    echo "  WARNING: .storage/ not found — HA may not have completed first boot"
fi

# ============================================================
# Custom dashboards (Lovelace UI)
# ============================================================
echo "Capturing dashboard config..."
mkdir -p "$OUTDIR/dashboards"

# Lovelace config stored in .storage is already captured above.
# Also copy any manually managed dashboard YAML files.
for f in ui-lovelace.yaml lovelace.yaml; do
    if [ -f "$HA_CONFIG/$f" ]; then
        cp "$HA_CONFIG/$f" "$OUTDIR/dashboards/"
        echo "  captured: $f"
    fi
done

if [ -d "$HA_CONFIG/lovelace" ]; then
    cp -r "$HA_CONFIG/lovelace" "$OUTDIR/dashboards/"
    echo "  captured: lovelace/ directory"
fi

# ============================================================
# www/ — custom frontend files (if any)
# ============================================================
if [ -d "$HA_CONFIG/www" ]; then
    echo "Capturing www/ (custom frontend files)..."
    cp -r "$HA_CONFIG/www" "$OUTDIR/"
fi

# ============================================================
# custom_components/ — HACS and manually installed integrations
#   (e.g. ha-composite-tracker). Losing this means reinstalling
#   HACS and re-adding custom repositories.
# ============================================================
if [ -d "$HA_CONFIG/custom_components" ]; then
    echo "Capturing custom_components/..."
    cp -r "$HA_CONFIG/custom_components" "$OUTDIR/"
    component_count=$(find "$OUTDIR/custom_components" -mindepth 1 -maxdepth 1 -type d | wc -l)
    echo "  captured $component_count custom component(s)"
else
    echo "  INFO: no custom_components/ found — skipping"
fi

# ============================================================
# blueprints/ — automation and script blueprints
# ============================================================
if [ -d "$HA_CONFIG/blueprints" ]; then
    echo "Capturing blueprints/..."
    cp -r "$HA_CONFIG/blueprints" "$OUTDIR/"
    blueprint_count=$(find "$OUTDIR/blueprints" -name "*.yaml" | wc -l)
    echo "  captured $blueprint_count blueprint(s)"
else
    echo "  INFO: no blueprints/ found — skipping"
fi

# ============================================================
# Directory listing of full HA config (for reference)
# ============================================================
ls -la "$HA_CONFIG" > "$OUTDIR/_ha_config_listing.txt" 2>&1
ls -la "$HA_CONFIG/.storage" >> "$OUTDIR/_ha_config_listing.txt" 2>&1

# ============================================================
# Done
# ============================================================
echo ""
echo "HA snapshot saved to: $OUTDIR"
echo ""
ls -la "$OUTDIR"
