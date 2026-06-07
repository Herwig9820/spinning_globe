# PI-HTA — Presence Detection Cheatsheet

Architecture and configuration for combined GPS + WiFi presence detection of the
A36 phone, with `person.herwig` as the top-level entity.

The HACS installation procedure is documented separately and not repeated here.

---

## Architecture overview

```
            ┌────────────────────────────────────────────┐
            │       HA Companion App on A36 phone        │
            └────────────────────┬───────────────────────┘
                                 │
              ┌──────────────────┴───────────────────┐
              ▼                                      ▼
  device_tracker.local_a36_herwig       sensor.a36_herwig_wi_fi_connection
  (Companion app, GPS-based)            (Companion app, current SSID as string)
              │                                      │
              │                                      ▼
              │                        binary_sensor.a36_on_home_wifi
              │                        (HA template: SSID == 'homeNet1')
              │                                      │
              └──────────────┬───────────────────────┘
                             ▼
              ┌────────────────────────────────┐
              │ device_tracker.a36_combined    │
              │ (Composite tracker, OR-logic)  │
              └────────────────┬───────────────┘
                               ▼
                       ┌───────────────┐
                       │ person.herwig │
                       └───────┬───────┘
                               ▼
                  ← all presence automations
```

---

## Why combined GPS + WiFi (the OR-logic rationale)

Each source alone has a weakness; OR-combined, they complement:

- **GPS alone:** accurate but slow. Android typically updates location every
  1–3 minutes for zone entry/exit. Also vulnerable to Android battery
  optimisation killing the Companion app.
- **WiFi alone:** very fast (sub-second on arrival) but has no concept of "where" —
  only "connected to known SSID = home". Can't detect away beyond "not on home
  WiFi".
- **Combined (OR):** WiFi gives fast detection on arrival; GPS confirms
  departure (waits for actual zone exit, so a brief WiFi drop doesn't false-flag
  as "leaving home").

| Scenario | GPS | WiFi | Composite result | Why |
|---|---|---|---|---|
| Arriving home | slow (1–3 min) | instant (associates with AP) | `home` immediately | WiFi is the fast trigger |
| Leaving home | exits zone after 1–3 min | drops AP immediately | `not_home` only when GPS exits zone | WiFi-off is deliberately *ignored* |
| WiFi off while at home | still home | off | `home` | GPS keeps composite anchored |
| Phone reboots / app killed | possibly stale | stale | last known state held | Watchdog (see `HA_location_monitoring_automations.md`) catches prolonged staleness |

---

## Components

### 1. WiFi connection sensor (Companion app)

Required on the phone:
- ☰ → Companion App → Manage sensors → **WiFi Connection** → enabled
- Android Settings → Apps → Home Assistant → Permissions → Location →
  **Allow all the time** + precise (Android requires location permission to read
  the SSID)

Result: `sensor.a36_herwig_wi_fi_connection` in HA, with state = current SSID name
(e.g. `homeNet1`).

Note the entity naming: `wi_fi` (with underscore), not `wifi`. Filtering on
"wifi" in Developer Tools → States won't find it. Use `a36` or `wi_fi`.

### 2. WiFi binary sensor (HA template)

Defined in `/opt/homeassistant/config/configuration.yaml`:

```yaml
template:
  - binary_sensor:
      - name: "A36 on home WiFi"
        state: >
          {{ states('sensor.a36_herwig_wi_fi_connection') == 'homeNet1' }}
```

Result: `binary_sensor.a36_on_home_wifi`, `on` when phone is on homeNet1, `off`
otherwise.

Restart HA after editing the YAML, then verify in Developer Tools → States.

### 3. Composite Device Tracker (HACS integration, pnbruckner)

Assumes HACS and the integration are already installed (separate procedure —
this cheatsheet does not cover HACS installation).

**Add the configured instance:**

Settings → Devices & Services → + Add Integration → search "Composite" → select it.

| Setup field | Value |
|---|---|
| Name | `A36 combined` |
| Entities to track | `device_tracker.local_a36_herwig` AND `binary_sensor.a36_on_home_wifi` |
| Composite entity picture | none |
| Entities for which "all states should be used" | **none** — leave empty |
| Require movement | OFF |
| Show unknown speed as zero | OFF |

**Critical: the "all states used" option must NOT include the WiFi binary sensor.**

Why this matters — the option controls how the composite reacts to a non-GPS
source going `off`:

| Option for the WiFi sensor | WiFi `on` | WiFi `off` |
|---|---|---|
| **Not selected (default — correct)** | Drives composite to `home` immediately | Ignored — GPS keeps deciding |
| Selected | Drives composite to `home` immediately | Drives composite to `not_home` immediately (brittle) |

Selecting it would mean a momentary WiFi drop while sitting at home (router
reboot, phone sleeping) would falsely flip the composite to `not_home` and fire
a "leaving home" notification. The default (unselected) preserves the
asymmetric OR-logic that's the whole point of combining sources.

Result: `device_tracker.a36_combined` — single entity merging both sources.

### 4. Person entity

Settings → People → Herwig → "Select the devices that belong to this person":

- **Add:** `device_tracker.a36_combined`
- **Remove:** `device_tracker.local_a36_herwig` (the raw GPS tracker)

The raw GPS tracker still exists and feeds the composite; it's just disconnected
from the person entity to prevent double-counting (the composite already
incorporates GPS).

Result: `person.herwig` reflects the merged composite signal. Use this entity
for all presence automations.

---

## Behaviour summary

| Situation | `binary_sensor.a36_on_home_wifi` | `device_tracker.a36_combined` | `person.herwig` |
|---|---|---|---|
| At home, WiFi on | `on` | `home` (fast) | `home` |
| At home, WiFi off (e.g. airplane mode in garden) | `off` | `home` (GPS holds it) | `home` |
| Leaving home, on cellular | `off` | `home` → `not_home` when GPS exits zone | `home` → `not_home` |
| Returning home, WiFi connects | `off` → `on` | `not_home` → `home` (immediate) | `not_home` → `home` |
| Phone off / app killed | last value frozen | last value frozen | last value frozen |

---

## Tests

| Test | Description | Status |
|---|---|---|
| 1 | In-house: WiFi off while at home → composite stays `home`, no false "leaving home" notification | PASSED |
| 2 | Real-world: leaving home → composite goes `not_home` within 1–3 min after exiting zone | TBC |
| 3 | Real-world: returning home → composite goes `home` within seconds of WiFi reconnecting | TBC |

---

## Multi-home consideration (deferred)

Current setup is home-1 only (homeNet1, Belgium). If/when home-2 presence is
needed (Vaison, France):

- Add a second zone in HA at the home-2 coordinates (Settings → Areas & Zones →
  Zones → Add Zone). The composite tracker's state automatically becomes the
  zone name (e.g. `Vaison`) when GPS is inside it.
- Per-zone automations: trigger on `person.herwig` state changing to `home`
  (Belgium), to `Vaison` (France), to `not_home` (traveling), etc.
- Optionally a second template binary sensor for `homeNet2`. Important: do NOT
  add it to the composite tracker — the composite interprets *any* binary sensor
  `on` as "home zone", which would falsely tag home-2 presence as home-1. Use
  the home-2 binary sensor *directly* in home-2 automations instead, bypassing
  the composite/person abstraction for that home.

---

## Key gotchas (lessons from setup)

- **Geolocation vs Zone trigger:** HA's "Geolocation" trigger fires only on
  earthquake/weather feed entities. Presence automations need a **Zone** trigger.
- **Sensor name:** `sensor.a36_herwig_wi_fi_connection` — `wi_fi` with underscore.
- **Composite tracker custom repo:** not in the default HACS registry; added as
  custom repository (`https://github.com/pnbruckner/ha-composite-tracker`,
  type: Integration).
- **"All states used" trap:** explained above. The default (empty) is the
  correct choice; the dialog text is somewhat opaque.
