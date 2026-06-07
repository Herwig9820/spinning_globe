# PI-HTA — Home Assistant Basic Setup Cheatsheet

Reference for the foundational HA configuration: server-side setup (devices, automations,
areas/zones, people, developer tools) and the Android A36 Companion app settings.

Sister cheatsheets:
- `HA_presence_detection_cheatsheet.md` — composite device tracker + person.herwig
- `HA_location_monitoring_automations.md` — stale-reporting detection

---

## PART A — HOME ASSISTANT SERVER-SIDE (on the Pi)

Accessed via the HA web UI at `http://192.168.0.120:8123` (LAN, home 1) or
`http://100.66.121.78:8123` (Tailscale).

### A1. Device setup — Niko power outlet

The single Zigbee device currently paired.

| Property | Value |
|---|---|
| Device | Niko 170-33505 power outlet |
| Path into HA | Zigbee2MQTT → Mosquitto (MQTT) → HA MQTT integration |
| Pairing route | Joined via Zigbee2MQTT frontend (port 8080), then auto-discovered by HA |
| HA discovery | Enabled via Zigbee2MQTT `homeassistant: enabled: true` |
| Entity ID | `switch.niko_power_outlet_1` — check Developer Tools → States, filter on 'switch' |
| Control surface | One custom dashboard with a single Button card |

**Where to find it in HA:** Settings → Devices & Services → MQTT → look for the
Niko device, or Settings → Devices & Services → Devices and filter by name.

**Key principle:** no Niko hub needed. Only the connected switch module + the
Sonoff Zigbee dongle are required. The 552-72xxx connected switch modules (to be
installed later) are mains-powered Zigbee Router devices that fit standard
flush-mounting boxes.

### A2. Automations

Webhook + presence automations currently defined.

**Webhook automations (for Tasker / external triggers):**

| Name | Trigger | Action |
|---|---|---|
| Niko On | Webhook, ID `niko_on`, methods POST + PUT | Turn on Niko outlet |
| Niko Off | Webhook, ID `niko_off`, methods POST + PUT | Turn off Niko outlet |

Critical webhook setting (this caused a long debug session):
- **"Only accessible from the local network"** must be **UNCHECKED** to allow
  Tailscale (remote) requests.
- Even when it appeared unchecked, HA silently blocked remote requests until the
  checkbox was toggled on→save→off→save to force it to re-apply. If remote
  webhooks ever fail with HA logging *"Received remote request for local webhook"*,
  re-toggle this setting.

**Presence automations (geofencing):**

| Name | Trigger type | Entity | Zone | Event |
|---|---|---|---|---|
| coming home | Zone (under "Time and location") | `person.herwig` | Home | Enter |
| leaving home | Zone (under "Time and location") | `person.herwig` | Home | Leave |

Both trigger on `person.herwig` (not the raw device tracker). See
`HA_presence_detection_cheatsheet.md` for the architecture underneath
(GPS + WiFi → composite → person).

Critical: the trigger type MUST be **Zone**, NOT **Geolocation**.
- **Zone** trigger = fires when a person/device tracker enters or leaves a zone
  (what you want).
- **Geolocation** trigger = fires when entities from a geolocation *integration*
  (earthquakes, weather alerts) appear in a zone — wrong tool, never fires for
  your phone. Was the original bug.

### A3. Areas, labels and zones

**Zones** (Settings → Areas & Zones → Zones):

| Zone | Purpose | Notes |
|---|---|---|
| Home | Presence detection for geofencing | Radius 100–200 m; pin must be on the actual house |

**Areas** (Settings → Areas & Zones → Areas):
- Areas are optional organisational groupings of devices (e.g. "Living room", "Kitchen").
  Worth setting up as more Niko switches are added.

**Labels:**
- Not yet configured. Labels are a cross-cutting tagging system (e.g. tag entities
  "lighting" across areas). Optional, useful later for bulk actions.

### A4. People

`person.herwig` is the primary presence entity. Linked to a single composite
device tracker `device_tracker.a36_combined` (which itself merges GPS + WiFi
signals — see `HA_presence_detection_cheatsheet.md`).

**Best practice:** trigger all presence automations on `person.herwig`, not on
raw device trackers. The composite/person abstraction is what should be visible
to automation logic; the underlying GPS / WiFi mechanics stay encapsulated.

### A5. Home information

- Settings → System → Home Information holds the home location (lat/long),
  unit system, time zone, currency, country. The home location set here should
  match the Home zone. Currently set to the Belgium first home (Merelbeke).

### A6. Developer Tools

Settings → Developer Tools

Tabs across the top: **YAML, States, Actions, Template, Events, Statistics, Assist**
(exact labels can vary by version).

Most useful tab — **States:**
- Filter box to search entities (e.g. type `device_tracker`).
- Shows each entity's current state and attributes.
- Key entities to watch for geofencing:
  - `person.herwig` — top-level, what automations should trigger on
  - `device_tracker.a36_combined` — composite of GPS + WiFi
  - `device_tracker.local_a36_herwig` — raw Companion app GPS, feeds composite
  - `binary_sensor.a36_on_home_wifi` — template, true when on homeNet1
  - `sensor.a36_herwig_wi_fi_connection` — current SSID as string
- States seen on trackers: `home`, `not_home`, `unknown`, `unavailable`.
- `unknown` / `unavailable` = phone hasn't reported a location recently; zone
  triggers cannot fire from these states.

### A7. Switching the Niko outlet from a bash script

There are two ways to do this. The distinction is **which protocol the caller can
speak**, not where it physically is:
- If the caller can reach the **Mosquitto broker** (port 1883) — whether via
  localhost, LAN, or Tailscale — use **MQTT publish**. This is the canonical,
  cleanest method.
- If the caller is limited to **HTTP** and can't reach the broker (e.g. Tasker on
  the phone), use **webhook + curl**.

#### A7.1 — MQTT publish (canonical) — TESTED, WORKING

Talks **directly to Mosquitto**, bypassing HA entirely.

```bash
#!/bin/bash
source ~/scripts/lamp-common.sh
require_online
mosquitto_pub "${MOSQ_ARGS[@]}" -t "${TOPIC_BASE}/set" -m '{"state":"ON"}'
```

(The "off" variant publishes `'{"state":"OFF"}'`.) Common settings — broker host,
credentials, `TOPIC_BASE`, the `require_online` helper — live in
`~/scripts/lamp-common.sh`.

Why this is the cleanest method:
- **Fewest hops:** script → Mosquitto → Zigbee2MQTT → device.
- **No dependency on HA or any automation.** Doesn't break if the `niko_on`
  automation is edited/disabled or if the webhook "local network only" checkbox
  flips.
- **Honest failure:** if `mosquitto_pub` fails, it's a real failure (unlike the
  webhook).
- **Correct architectural layer:** controlling a Zigbee device is exactly what
  the MQTT layer is for.

#### A7.2 — Webhook + curl (HTTP-only alternative) — NOT YET TESTED

Use only when the caller cannot reach the Mosquitto broker and is limited to HTTP
(e.g. Tasker on the phone). Goes **through HA**: curl → HA webhook → `niko_on`
automation → HA's MQTT integration → Mosquitto → device.

```bash
#!/usr/bin/env bash
# niko_webhook.sh — switch the Niko outlet via HA webhook (HTTP-only fallback)
# Usage: ./niko_webhook.sh on | off

set -euo pipefail

# Choose one:
#   HA_HOST="192.168.0.120:8123"     # LAN (home 1)
#   HA_HOST="100.66.121.78:8123"     # Tailscale (remote)
HA_HOST="100.66.121.78:8123"

case "${1:-}" in
  on)  WEBHOOK="niko_on"  ;;
  off) WEBHOOK="niko_off" ;;
  *)   echo "Usage: $0 on|off" >&2; exit 1 ;;
esac

curl -fsS -X POST "http://${HA_HOST}/api/webhook/${WEBHOOK}" \
  && echo "Sent: ${WEBHOOK}" \
  || { echo "Failed to reach HA at ${HA_HOST}" >&2; exit 1; }
```

Caveats specific to the webhook method:
- `curl -X POST` is the correct method; a plain GET (or a browser) returns
  **HTTP 405 Method Not Allowed** — expected, not a setup error.
- HA returns **HTTP 200 even for a non-existent webhook ID** (deliberate, to stop
  attackers probing valid IDs). So a 200 / curl success does NOT prove the
  automation fired — confirm via the outlet state or the automation trace. This
  is a key reason the MQTT method is preferred where possible.

**Rule of thumb:** if the caller can reach the Mosquitto broker (port 1883) —
localhost, LAN, or Tailscale → use A7.1 (MQTT). If the caller is HTTP-only and
can't reach the broker (the phone via Tasker) → use the webhook (A7.2).

---

## PART B — ANDROID A36 COMPANION APP

All paths below are inside the Home Assistant Android app
(HA's own term for it is the "Companion App").
Menu route abbreviated as: **☰ → Companion App → ...**

### B1. Server (single-server architecture)

One server configured (☰ → Companion App → Servers). The internal/external URL
mechanism handles home vs away automatically based on the SSID list.

| Setting | Value |
|---|---|
| Name | your choice (e.g. "Home") |
| Home Assistant URL | `http://100.66.121.78:8123` (Tailscale — external, used when not on a listed SSID) |
| Internal Connection URL | `http://192.168.0.120:8123` (LAN, wlan0 — used on listed SSIDs) |
| Home network SSIDs | `homeNet1` |
| VPN connection | OFF (Tailscale URL used directly) |
| Ethernet connected | OFF |
| Remote connection security level | less secure |
| Remotely control app & device | ON |

**Why the WiFi IP (`...0.120`) for Internal URL, not the Ethernet IP (`...0.119`):**
the WiFi IP continues to work if the Pi is ever moved to a room without Ethernet.
Speed difference is negligible for the app's small JSON payloads. The Ethernet IP
is reserved for large `scp` transfers (see SSH cheatsheet, `pi-eth` alias).

**Why ONE server, not two:** an older configuration used two servers (one LAN +
one Tailscale). Consolidated to one because the internal/external URL pattern is
simpler and the auth state of two parallel servers tended to drift, causing
"tracking went stale" issues.

Key practical notes:
- After any server URL change, **force-stop and reopen the app** so it picks the
  right URL based on the current SSID.
- Security level "less secure" is required because the URLs are plain HTTP, not
  HTTPS. Fine in practice — Tailscale encrypts the remote transport, and LAN runs
  inside the home network.

Symptom-to-cause reference:
- *"failed to send", on cellular/away from home* → check Tailscale is up on the
  phone; verify Home Assistant URL is the Tailscale IP.
- *"failed to send", on home WiFi* → check the SSID is in the Home Network list;
  verify Internal Connection URL is correct.
- *Tracking repeatedly goes stale* → delete and re-add the server (auth state can
  get stuck — see `HA_location_monitoring_automations.md` for recovery).

### B2. Sensors — the location sensor groups

☰ → Companion App → Sensors.

**(1) Geolocation Sensors**

| Setting | Value |
|---|---|
| Enable sensor | ON |
| Update interval | 1 minute |
| Minimum accuracy | 200 m |

Reports a geocoded address and periodic position. NOT the one that drives zone
enter/leave on its own.

**(2) Location Sensors** — *the "holy grail" group*

| Setting | Value | Purpose |
|---|---|---|
| Background location | enabled | Lets the app report position when not in foreground — essential |
| High accuracy mode | false (OFF) | Continuous GPS; big battery drain — keep OFF |
| High accuracy update interval | 5 s (inactive while high-accuracy OFF) | Only relevant if high accuracy is on |
| Location zone | enabled | **Drives the enter/leave zone trigger** via Android geofencing API |
| Single accurate location | enabled | One-shot precise fix on demand |

Critical distinction:
- **Periodic background reporting** (~every 15 min) keeps `device_tracker` fresh.
- **Location zone** uses Android's geofencing API and fires the moment you cross
  the zone boundary — it does NOT wait for the 15-min cycle. This is why zone
  entry/exit should fire promptly even between periodic updates.

**(3) WiFi Connection sensor** — required for the presence architecture

| Setting | Value |
|---|---|
| WiFi Connection | enabled |
| Resulting HA entity | `sensor.a36_herwig_wi_fi_connection` (note: `wi_fi` with underscore, not `wifi`) |
| State | The currently-connected SSID, e.g. `homeNet1` |

Feeds the WiFi template binary sensor in HA — see presence detection cheatsheet.

### B3. Update frequency behaviour (what the logs mean)

Watched via ☰ → Companion App → Troubleshooting → Location tracking.

- Every ~3 minutes: a check runs. If position hasn't changed → logged as
  **"duplicate"** and not sent. Normal and harmless.
- Every ~15 minutes: a full update is **"sent"** regardless of duplication.
- **"failed to send"** is misleadingly named — often just means "not sent because
  duplicate" OR an attempt to an unreachable URL. Does not necessarily indicate
  a real failure.

### B4. Other relevant phone-side settings

These are **not in HA** and **not in any Pi snapshot** — they live in Android /
the app and must be reconfigured manually on a new phone or app reinstall.

| Setting | Location on phone | Required value | Why |
|---|---|---|---|
| Battery optimization | Settings → Apps → Home Assistant → Battery | **Unrestricted** | Android kills background location otherwise |
| Location permission | Settings → Apps → Home Assistant → Permissions → Location | **Allow all the time** + precise | "Only while using" breaks background reporting AND prevents WiFi SSID reading |
| Background data | Settings → Apps → Home Assistant → Mobile data | Allow background data | Needed for reporting off-WiFi |
| Sleeping apps | Settings → Battery → Background usage limits | Tailscale removed from "Sleeping apps" (HA app couldn't be added on this device — Samsung quirk, appears harmless) | Sleeping apps lose background connectivity |
| Persistent connection | ☰ → Companion App → Servers → (server) | 'ALWAYS ON' | Keeps a live connection for notifications/background |
| Default digital assistant | Settings → Apps → Default apps → Digital assistant app | Tasker (for voice control via Tasker actions) | — |

---

## QUICK REGRESSION CHECKLIST

After any phone or HA change, check:

1. Developer Tools → States → `person.herwig` shows `home` when home (not `unknown`).
2. Developer Tools → States → `binary_sensor.a36_on_home_wifi` is `on` when on homeNet1.
3. Location tracking screen shows successful "sent" on WiFi and off-WiFi (Tailscale).
4. Tasker widgets switch the outlet on/off — on WiFi AND with WiFi off (Tailscale).
5. The two presence automations still use a **Zone** trigger on `person.herwig` (not Geolocation, not raw device tracker).
6. The two webhook automations still have "Only accessible from local network" **unchecked**.
   (Where to find it: open the automation → click the webhook trigger → click the
   gear/settings icon on the trigger card. That opens the popup with
   GET/HEAD/POST/PUT checkboxes and, at the bottom, "Only accessible from the
   local network".)
