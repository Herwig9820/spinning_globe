#!/bin/bash
# pi_healthcheck.sh — verify the home automation stack is healthy.
# Pure read-only diagnostics. Safe to run any time, including from cron.
#
# Optional: create ~/.pi_healthcheck.env with one MQTT credential so the
# script can run authenticated tests (broker auth + Z2M bridge state).
#   chmod 600 ~/.pi_healthcheck.env
#   contents:
#     MQTT_TEST_USER="herwig"
#     MQTT_TEST_PASS="<your-herwig-mqtt-password>"
# Without this file, auth-requiring checks are skipped (not failed).
#
# Exit code: 0 if all checks PASS or WARN/SKIP, non-zero == number of FAILs.

set -uo pipefail

# ============================================================
# Colors and counters
# ============================================================
if [ -t 1 ]; then
    RED=$(tput setaf 1)
    GREEN=$(tput setaf 2)
    YELLOW=$(tput setaf 3)
    GRAY=$(tput setaf 8 2>/dev/null || tput setaf 7)
    BOLD=$(tput bold)
    RESET=$(tput sgr0)
else
    RED= GREEN= YELLOW= GRAY= BOLD= RESET=
fi

PASS_COUNT=0
FAIL_COUNT=0
WARN_COUNT=0
SKIP_COUNT=0

pass() { echo "  ${GREEN}[ PASS ]${RESET} $1"; PASS_COUNT=$((PASS_COUNT+1)); }
fail() {
    echo "  ${RED}[ FAIL ]${RESET} $1"
    [ -n "${2:-}" ] && echo "           ${GRAY}$2${RESET}"
    FAIL_COUNT=$((FAIL_COUNT+1))
}
warn() {
    echo "  ${YELLOW}[ WARN ]${RESET} $1"
    [ -n "${2:-}" ] && echo "           ${GRAY}$2${RESET}"
    WARN_COUNT=$((WARN_COUNT+1))
}
skip() {
    echo "  ${GRAY}[ SKIP ]${RESET} $1"
    [ -n "${2:-}" ] && echo "           ${GRAY}$2${RESET}"
    SKIP_COUNT=$((SKIP_COUNT+1))
}
section() { echo ""; echo "${BOLD}── $1 ──${RESET}"; }

# Helper: check a systemd unit is active AND enabled
check_service() {
    local unit="$1"
    local active enabled
    active=$(systemctl is-active "$unit" 2>&1)
    enabled=$(systemctl is-enabled "$unit" 2>&1)

    if [ "$active" = "active" ]; then
        pass "$unit is running"
    else
        fail "$unit is NOT running" "is-active = $active"
        return 1
    fi

    if [ "$enabled" = "enabled" ]; then
        pass "$unit is enabled (auto-start on boot)"
    else
        warn "$unit is NOT enabled" "is-enabled = $enabled  →  won't auto-start after reboot"
    fi
    return 0
}

# Helper: check something is listening on a TCP port
check_port() {
    local port="$1"
    local label="$2"
    if ss -tlnH 2>/dev/null | awk '{print $4}' | grep -qE ":${port}\$"; then
        pass "$label is listening on port $port"
    else
        fail "$label is NOT listening on port $port"
    fi
}

# ============================================================
# Header + optional credentials
# ============================================================
echo ""
echo "${BOLD}Pi-HTA health check — $(date)${RESET}"
echo "Host: $(hostname)   User: $(whoami)"

CREDS_FILE="$HOME/.pi_healthcheck.env"
MQTT_TEST_USER=""
MQTT_TEST_PASS=""
if [ -f "$CREDS_FILE" ]; then
    # shellcheck disable=SC1090
    source "$CREDS_FILE"
fi
HAVE_CREDS=0
if [ -n "$MQTT_TEST_USER" ] && [ -n "$MQTT_TEST_PASS" ]; then
    HAVE_CREDS=1
fi

# ============================================================
# 1. System health
# ============================================================
section "1. System health"

# Throttling history (Pi-specific). 0x0 = no PSU/thermal events ever.
if command -v vcgencmd >/dev/null 2>&1; then
    throttled=$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2)
    if [ "$throttled" = "0x0" ]; then
        pass "No throttling events since boot (get_throttled = 0x0)"
    else
        warn "Throttling history present" "get_throttled = $throttled  →  PSU under-voltage or thermal event has occurred"
    fi

    temp=$(vcgencmd measure_temp 2>/dev/null | sed "s/temp=//;s/'C//")
    temp_int=${temp%.*}
    if [ "${temp_int:-0}" -lt 70 ]; then
        pass "CPU temperature OK (${temp}°C)"
    elif [ "${temp_int:-0}" -lt 80 ]; then
        warn "CPU temperature elevated" "${temp}°C  →  check fan / airflow"
    else
        fail "CPU temperature too high" "${temp}°C  →  thermal throttling imminent"
    fi
else
    skip "vcgencmd not available (not a Pi?)"
fi

# Disk free on root
disk_avail_kb=$(df -k / | awk 'NR==2 {print $4}')
disk_avail_gb=$((disk_avail_kb / 1024 / 1024))
if [ "$disk_avail_gb" -ge 5 ]; then
    pass "Root filesystem has ${disk_avail_gb} GB free"
elif [ "$disk_avail_gb" -ge 1 ]; then
    warn "Root filesystem low on space" "${disk_avail_gb} GB free"
else
    fail "Root filesystem critically full" "${disk_avail_gb} GB free"
fi

# Critical kernel errors since boot
err_count=$(journalctl -p err -b --no-pager 2>/dev/null | wc -l)
# journalctl prints a header line even on empty output; treat <=2 lines as clean
if [ "$err_count" -le 2 ]; then
    pass "No critical errors in journal since boot"
else
    warn "Journal contains $err_count error-level lines since boot" "review with: journalctl -p err -b --no-pager"
fi

# ============================================================
# 2. Tailscale
# ============================================================
section "2. Tailscale"

check_service tailscaled

if command -v tailscale >/dev/null 2>&1; then
    if tailscale status >/dev/null 2>&1; then
        ts_ip=$(tailscale ip -4 2>/dev/null | head -1)
        if [ -n "$ts_ip" ]; then
            pass "Tailscale logged in, tailnet IP = $ts_ip"
        else
            warn "Tailscale running but no IPv4 address"
        fi
    else
        fail "Tailscale not logged in" "run: sudo tailscale up"
    fi
else
    fail "tailscale CLI not found"
fi

# ============================================================
# 3. Mosquitto
# ============================================================
section "3. Mosquitto"

check_service mosquitto
check_port 1883 "Mosquitto"

# Three expected MQTT users
if [ -r /etc/mosquitto/passwd ] || sudo -n true 2>/dev/null; then
    if users=$(sudo -n cat /etc/mosquitto/passwd 2>/dev/null | cut -d: -f1 | sort | tr '\n' ' '); then
        for expected in herwig spinning_globe zigbee2mqtt homeassistant; do
            if echo " $users " | grep -q " $expected "; then
                pass "MQTT user '$expected' exists"
            else
                fail "MQTT user '$expected' missing from /etc/mosquitto/passwd"
            fi
        done
    else
        skip "Cannot read /etc/mosquitto/passwd (sudo needs password)" \
             "run interactively or grant NOPASSWD for this script to enable"
    fi
else
    skip "Cannot read /etc/mosquitto/passwd (need sudo)"
fi

# Live auth test
if [ "$HAVE_CREDS" = "1" ]; then
    if out=$(timeout 3 mosquitto_sub -h localhost -t '$SYS/broker/version' \
                -u "$MQTT_TEST_USER" -P "$MQTT_TEST_PASS" -C 1 2>&1); then
        if echo "$out" | grep -qi mosquitto; then
            pass "MQTT auth works ($MQTT_TEST_USER) — broker reports: $out"
        else
            warn "MQTT auth returned unexpected output" "$out"
        fi
    else
        fail "MQTT auth failed for user '$MQTT_TEST_USER'" "$out"
    fi
else
    skip "MQTT auth test (no credentials in $CREDS_FILE)"
fi

# ============================================================
# 4. Node-RED
# ============================================================
section "4. Node-RED"

check_service nodered
check_port 1880 "Node-RED"

SETTINGS="$HOME/.node-red/settings.js"
if [ -r "$SETTINGS" ]; then
    # A directive is "set" if the line is uncommented and contains a value
    is_set() {
        # match a line starting (after optional whitespace) with the key — not //
        grep -E "^[[:space:]]*$1[[:space:]]*:" "$SETTINGS" | grep -vqE "^[[:space:]]*//"
    }
    if is_set credentialSecret; then
        pass "Node-RED credentialSecret is set"
    else
        fail "Node-RED credentialSecret is NOT set" "flow credentials are unencrypted"
    fi
    if is_set adminAuth; then
        pass "Node-RED adminAuth is set (login required)"
    else
        fail "Node-RED adminAuth is NOT set" "editor is open to the network"
    fi
    if is_set contextStorage; then
        pass "Node-RED contextStorage is set (persistence enabled)"
    else
        warn "Node-RED contextStorage is NOT set" "context vars lost on restart"
    fi
else
    fail "Cannot read $SETTINGS"
fi

# ============================================================
# 5. Zigbee2MQTT (and Sonoff dongle)
# ============================================================
section "5. Zigbee2MQTT"

# Dongle present on USB (Sonoff ZBDongle-E uses Silicon Labs CP2102N, 10c4:ea60)
if dongle_line=$(lsusb -d 10c4:ea60 2>/dev/null) && [ -n "$dongle_line" ]; then
    pass "Sonoff dongle detected on USB ($dongle_line)"
else
    fail "Sonoff dongle NOT detected on USB" "no device with USB ID 10c4:ea60"
fi

# Stable serial path
sonoff_link=$(ls /dev/serial/by-id/ 2>/dev/null | grep -i sonoff | head -1)
if [ -n "$sonoff_link" ]; then
    pass "Stable serial path exists: /dev/serial/by-id/$sonoff_link"
else
    fail "No stable serial path for the dongle" "expected: /dev/serial/by-id/usb-Itead_Sonoff_*"
fi

check_service zigbee2mqtt
check_port 8080 "Z2M frontend"

# Watchdog (Type=notify) — check via systemd properties, not status text
notify_state=$(systemctl show -p NotifyAccess --value zigbee2mqtt 2>/dev/null)
if [ "$notify_state" = "main" ] || [ "$notify_state" = "all" ]; then
    pass "Z2M is using systemd notify protocol (watchdog active)"
else
    skip "Z2M not configured for systemd notify (NotifyAccess=$notify_state)"
fi

# End-to-end pipeline test (two stages)
if [ "$HAVE_CREDS" = "1" ]; then
    # Stage A — retained bridge/state should be "online"
    bridge_state=$(timeout 3 mosquitto_sub -h localhost -t 'zigbee2mqtt/bridge/state' \
                       -u "$MQTT_TEST_USER" -P "$MQTT_TEST_PASS" -C 1 2>/dev/null)
    if echo "$bridge_state" | grep -q '"online"'; then
        pass "Z2M retained bridge/state = online"
    else
        warn "Z2M retained bridge/state is not 'online'" "received: ${bridge_state:-<empty>}"
    fi

    # Stage B — request a live health_check, prove Z2M is actively responding NOW
    # Subscribe in background, then publish the request, then wait for response
    response_file=$(mktemp)
    timeout 5 mosquitto_sub -h localhost -t 'zigbee2mqtt/bridge/response/health_check' \
        -u "$MQTT_TEST_USER" -P "$MQTT_TEST_PASS" -C 1 > "$response_file" 2>/dev/null &
    sub_pid=$!
    sleep 0.5  # give the subscriber time to connect before we publish
    mosquitto_pub -h localhost -t 'zigbee2mqtt/bridge/request/health_check' -m '' \
        -u "$MQTT_TEST_USER" -P "$MQTT_TEST_PASS" 2>/dev/null
    wait "$sub_pid" 2>/dev/null
    response=$(cat "$response_file")
    rm -f "$response_file"

    if echo "$response" | grep -q '"healthy":true'; then
        pass "Z2M responded to live health_check (end-to-end pipeline OK)"
    elif [ -n "$response" ]; then
        warn "Z2M responded but not healthy" "$response"
    else
        fail "Z2M did not respond to health_check within 5s" "process may be hung or not connected to broker"
    fi
else
    skip "Z2M end-to-end pipeline test (no credentials in $CREDS_FILE)"
fi

# ============================================================
# 6. Docker + Home Assistant
# ============================================================
section "6. Docker + Home Assistant"

# Docker daemon running
if command -v docker >/dev/null 2>&1; then
    if docker info >/dev/null 2>&1; then
        pass "Docker daemon is running"
        docker_ver=$(docker version --format '{{.Server.Version}}' 2>/dev/null)
        pass "Docker version: $docker_ver"
    else
        fail "Docker daemon is not responding" "try: sudo systemctl start docker"
    fi
else
    fail "docker CLI not found"
fi

# Docker service enabled at boot
check_service docker

# HA container running
if command -v docker >/dev/null 2>&1; then
    ha_status=$(docker inspect --format '{{.State.Status}}' homeassistant 2>/dev/null)
    ha_restart=$(docker inspect --format '{{.RestartCount}}' homeassistant 2>/dev/null)
    if [ "$ha_status" = "running" ]; then
        pass "Home Assistant container is running"
        if [ "${ha_restart:-0}" -gt 5 ]; then
            warn "HA container has restarted ${ha_restart} times" "check logs: docker logs homeassistant --tail 50"
        elif [ "${ha_restart:-0}" -gt 0 ]; then
            pass "HA container restart count: ${ha_restart} (acceptable)"
        else
            pass "HA container restart count: 0"
        fi
    elif [ -z "$ha_status" ]; then
        fail "Home Assistant container not found" "run: cd /opt/homeassistant && docker compose up -d"
    else
        fail "Home Assistant container status: $ha_status" "run: docker logs homeassistant --tail 50"
    fi
fi

# HA HTTP port 8123 responding
if curl -sf --max-time 5 http://localhost:8123 >/dev/null 2>&1; then
    pass "Home Assistant HTTP responding on port 8123"
else
    warn "Home Assistant HTTP not responding on port 8123" "may still be starting up — retry in 60s"
fi

# docker-compose.yml present
if [ -f /opt/homeassistant/docker-compose.yml ]; then
    pass "docker-compose.yml found at /opt/homeassistant/docker-compose.yml"
else
    fail "docker-compose.yml missing" "expected: /opt/homeassistant/docker-compose.yml"
fi

# ============================================================
# Summary
# ============================================================
echo ""
echo "${BOLD}── Summary ──${RESET}"
echo "  ${GREEN}PASS${RESET}: $PASS_COUNT"
echo "  ${RED}FAIL${RESET}: $FAIL_COUNT"
echo "  ${YELLOW}WARN${RESET}: $WARN_COUNT"
echo "  ${GRAY}SKIP${RESET}: $SKIP_COUNT"
echo ""

if [ "$FAIL_COUNT" -eq 0 ]; then
    echo "${GREEN}${BOLD}Stack is healthy.${RESET}"
    exit 0
else
    echo "${RED}${BOLD}$FAIL_COUNT check(s) FAILED — see details above.${RESET}"
    # Cap exit code at 255 (POSIX limit)
    [ "$FAIL_COUNT" -gt 255 ] && FAIL_COUNT=255
    exit "$FAIL_COUNT"
fi
