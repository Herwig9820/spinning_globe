# PI-HTA — Home Assistant Basic Setup Cheatsheet

Reference for the foundational HA configuration that wasn't previously documented:
server-side setup (devices, automations, areas/zones, people, developer tools) and
the Android A36 companion app settings.

---

## PART A — HOME ASSISTANT SERVER-SIDE (on the Pi)

Accessed via the HA web UI at `http://192.168.1.120:8123` (LAN) or
`http://100.66.121.78:8123` (Tailscale).

### A1. Device setup — Niko power outlet

The single Zigbee device currently paired.

| Property | Value |
|---|---|
| Device | Niko 170-33505 power outlet |
| Path into HA | Zigbee2MQTT → Mosquitto (MQTT) → HA MQTT integration |
| Pairing route | Joined via Zigbee2MQTT frontend (port 8080), then auto-discovered by HA |
| HA discovery | Enabled via Zigbee2MQTT `homeassistant: enabled: true` |
| Entity ID | switch.niko_power_outlet_1 — check Developer Tools → States, filter on 'switch' |
| Control surface | One custom dashboard with a single Button card |

**Where to find it in HA:** Settings → Devices & Services → MQTT → look for the
Niko device, or Settings → Devices & Services → Devices and filter by name.

**Key principle:** no Niko hub needed. Only the connected switch module + the
Sonoff Zigbee dongle are required. The 552-72xxx connected switch modules (to be
installed later) are mains-powered Zigbee Router devices that fit standard
flush-mounting boxes.

### A2. Automations

Four automations currently defined.

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

**Zone automations (geofencing / presence):**

| Name | Trigger type | Entity | Zone | Event |
|---|---|---|---|---|
| coming home | Zone (under "Time and location") | `device_tracker` for A36 Herwig | Home | Enter |
| leaving home | Zone (under "Time and location") | `device_tracker` for A36 Herwig | Home | Leave |

Critical: the trigger type MUST be **Zone**, NOT **Geolocation**.
- **Zone** trigger = fires when a person/device tracker enters or leaves a zone
  (what you want).
- **Geolocation** trigger = fires when entities from a geolocation *integration*
  (earthquakes, weather alerts) appear in a zone — wrong tool, never fires for
  your phone. This was the original bug.

### A3. Areas, labels and zones

**Zones** (Settings → Areas & Zones → Zones):

| Zone | Purpose | Notes |
|---|---|---| 
| Home | Presence detection for geofencing | Radius set at 200 m (typical range 100–200 m); pin must be on the actual house |

**Areas** (Settings → Areas & Zones → Areas):
- Areas are optional organisational groupings of devices (e.g. "Living room", "Kitchen"). 
  Worth setting up as more Niko switches are added.

**Labels:**
- not yet configured. Labels are a cross-cutting tagging system (e.g. tag entities "lighting" across areas).
  Optional, useful later for bulk actions.

### A4. People

- we referred to the device tracker `device_tracker` for "A36 Herwig"
  directly in the zone automations, rather than a Person entity.
- A **Person** in HA (Settings → People) groups one or more device trackers under
  a named individual. Best practice is to create a Person and attach the phone's
  device tracker, then trigger automations on the Person rather than the raw
  device tracker. We have NOT done this yet — currently triggering on the device
  tracker directly, which works but is less robust.

### A5. Home information

- Settings → System → Home Information holds the home location (lat/long),
  unit system, time zone, currency, country. The home location set here should
  match the Home zone. During prototyping in the second home (Vaison-la-Romaine, France)
  it can be set to that location; will need updating to the Belgium first home
  on the permanent move.

### A6. Developer Tools

Settings -> Developer Tools

Tabs across the top: **YAML, States, Actions, Template, Events, Statistics, Assist**
(exact tab labels can vary by version).

Most useful tab — **States:**
- Filter box to search entities (e.g. type `device_tracker`).
- Shows each entity's current state and attributes.
- Key entity to watch for geofencing: `device_tracker.local_a36_herwig`
  States seen: `home`, `not_home`, `unknown`, `unavailable`.
- `unknown` / `unavailable` = the phone hasn't reported a location recently;
  zone triggers cannot fire from these states.

### A7. Switching the Niko outlet from a bash script

There are two ways to do this. The distinction is **which protocol the caller can
speak**, not where it physically is:
- If the caller can reach the **Mosquitto broker** (port 1883) — whether via
  localhost, LAN, or Tailscale — use **MQTT publish**. This is the canonical,
  cleanest method.
- If the caller is limited to **HTTP** and can't reach the broker (e.g. Tasker on
  the phone), use **webhook + curl**.

#### A7.1 — MQTT publish (canonical) — TESTED, WORKING

This talks **directly to Mosquitto**, the same path Zigbee2MQTT uses, bypassing
HA entirely. This is the existing working script.

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
  webhook, see below).
- **Correct architectural layer:** controlling a Zigbee device is exactly what the
  MQTT layer is for.

#### A7.2 — Webhook + curl (HTTP-only alternative) — NOT YET TESTED

Use only when the caller cannot reach the Mosquitto broker and is limited to HTTP
(e.g. Tasker on the phone). This goes **through HA**: curl → HA webhook →
`niko_on` automation → HA's MQTT integration → Mosquitto → device. More hops, more
fragility; documented here for completeness and because it mirrors what Tasker
does on the phone.

```bash
#!/usr/bin/env bash
# niko_webhook.sh — switch the Niko outlet via HA webhook (remote-only fallback)
# Usage: ./niko_webhook.sh on | off

set -euo pipefail

# Choose one:
#   HA_HOST="192.168.1.120:8123"     # LAN
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
  automation fired — confirm via the outlet state or the automation trace. This is
  a key reason the MQTT method is preferred where possible.

**Rule of thumb:** if the caller can reach the Mosquitto broker (port 1883) —
localhost, LAN, or Tailscale → use A7.1 (MQTT). If the caller is HTTP-only and
can't reach the broker (the phone via Tasker) → use the webhook (A7.2).

---

## PART B — ANDROID A36 COMPANION APP

All paths below are inside the Home Assistant Android app
(HA's own term for it is the "Companion App").
Menu route abbreviated as: **☰ → Companion App → ...**

### B1. Servers

Two servers configured (☰ → Companion App → Servers). The app pushes location to
ALL configured servers; each has its own connection settings.

| Setting | home Vaison – LAN | home Vaison – Tailscale |
|---|---|---|
| URL | `http://192.168.1.120:8123` | `http://100.66.121.78:8123` |
| Home network SSID | `homeNet2` | empty (leave blank) |
| VPN connection | OFF | ON |
| Ethernet connected | OFF | OFF |
| Remote connection security level | less secure | less secure |
| Remotely control app & device | ON |  |
| Internal connection URL | not set | not set |

Key learnings:
- The 'Tailscale' server (IP 100.66.121.78:8123) had to be **deleted and re-added while WiFi was OFF** so it
  authenticated freshly over the Tailscale path. Before that, background location silently failed to that server.
  Later, same happened with the LAN server as well.
- Security level must be "less secure" on both servers. "Most secure" requires
  an encrypted HTTPS connection; your servers use plain HTTP, so HA rejects it.
  This is fine — it has no effect on geofencing (which works regardless), and
  Tailscale is encrypted at the transport layer anyway. The setting only governs
  whether home-network/connection state is shared, and only over an encrypted link.


Symptom-to-cause reference:
- *"failed to send", server = LAN, while off WiFi* → expected, harmless noise
  (LAN unreachable off-WiFi).
- *"failed to send", server = Tailscale* → real problem; was the unauthenticated
  Tailscale server, fixed by delete + re-add.

### B2. Sensors — the two confusingly-named location sensors

☰ → Companion App → Sensors. There are **TWO separate sensor groups** for
location, with genuinely confusing naming — this cost a lot of debugging time.

**(1) Geolocation Sensors**

| Setting | Value |
|---|---|
| Enable sensor | ON |
| Update interval |  1 minute |
| Minimum accuracy | 200 m |

This reports a geocoded address and periodic position. It is NOT the one that
drives zone enter/leave on its own.

**(2) Location Sensors** — *this was the "holy grail" group*

| Setting | Value | Purpose |
|---|---|---|
| Background location | enabled | Lets the app report position when not in foreground — essential |
| High accuracy mode | false (OFF) | Continuous GPS; big battery drain — keep OFF |
| High accuracy update interval | 5 s (inactive while high-accuracy OFF) | Only relevant if high accuracy is on |
| Location zone | enabled | **Drives the enter/leave zone trigger** via Android geofencing API — separate from the 15-min periodic push |
| Single accurate location | enabled | One-shot precise fix on demand |

Critical distinction:
- **Periodic background reporting** (~every 15 min) keeps `device_tracker` fresh.
- **Location zone** uses Android's geofencing API and fires the moment you cross
  the zone boundary — it does NOT wait for the 15-min cycle. This is why zone
  entry/exit should fire promptly even between periodic updates.

### B3. Update frequency behaviour (what the logs mean)

Watched via ☰ → Companion App → Troubleshooting → Location tracking.

- Every ~3 minutes: a check runs. If position hasn't changed → logged as
  **"duplicate"** and not sent. This is normal and harmless.
- Every ~15 minutes: a full update is **"sent"** regardless of duplication.
- **"failed to send"** is misleadingly named — it often just means "not sent
  because duplicate" OR an attempt to an unreachable server (e.g. LAN off-WiFi).
  It does not necessarily indicate a real failure.

### B4. Other relevant phone-side settings

These are **not in HA** and **not in any Pi snapshot** — they live in Android /
the app and must be reconfigured manually on a new phone or app reinstall.

| Setting | Location on phone | Required value | Why |
|---|---|---|---|
| Battery optimization | Settings → Apps → Home Assistant → Battery | **Unrestricted** | Android kills background location otherwise |
| Location permission | Settings → Apps → Home Assistant → Permissions → Location | **Allow all the time** + precise | "Only while using" breaks background reporting |
| Background data | Settings → Apps → Home Assistant → Mobile data | Allow background data | Needed for reporting off-WiFi |
| Sleeping apps | Settings → Battery → Background usage limits | Tailscale removed from "Sleeping apps" (HA app couldn't be added on this device — Samsung quirk, appears harmless) | Sleeping apps lose background connectivity |
| Persistent connection | ☰ → Companion App → Servers → (server) | 'ALWAYS ON' for Tailscale | Keeps a live connection for notifications/background |
| Default digital assistant | Settings → Apps → Default apps → Digital assistant app | (relevant to voice control — see note) | — |

Note on the default assistant: we suspected changing it might break geofencing,
but later realised the geofencing failure at that time was actually the wrong
trigger type + unauthenticated Tailscale server. **There is no proven link**
between the default assistant setting and geofencing. Still worth testing one
change at a time when configuring voice control.

---

## QUICK REGRESSION CHECKLIST

After any phone or HA change, check:

1. Developer Tools → States → `device_tracker` shows `home` when home (not `unknown`).
2. Location tracking screen shows successful "sent" to BOTH servers on WiFi.
3. Tasker widgets switch the outlet on/off — on WiFi AND with WiFi off (Tailscale).
4. The two zone automations still use a **Zone** trigger (not Geolocation).
5. The two webhook automations still have "Only accessible from local network" **unchecked**
    (where to find it: In the automation's Webhook trigger block: open the 
    automation → click the webhook trigger → click the gear / settings icon on the trigger card. 
    That opens the popup you screenshotted earlier (the one with GET/HEAD/POST/PUT checkboxes and, 
    at the bottom, "Only accessible from the local network").)
  

---

