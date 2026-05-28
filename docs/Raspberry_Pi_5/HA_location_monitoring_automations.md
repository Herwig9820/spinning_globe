# Phone location reporting — monitoring

Two HA automations + one helper that detect when the phone stops reporting its location to HA and notify on transition. Single notification per transition (no flooding).

## Purpose

Catches the "catastrophic" class of location-reporting failures:
- Phone off, app crashed, both servers unreachable
- Location permission revoked
- Device tracker entity goes silent for any reason

Does **not** catch single-path failures (e.g. Tailscale-only break while LAN keeps the entity fresh). For that, a separate Tailscale-side watchdog would be needed.

## Components

| Component | Entity ID | Purpose |
|---|---|---|
| Helper (input_boolean) | `input_boolean.location_reporting_stale` | State machine: `off` = healthy, `on` = stale |
| Automation 1 | `Location reporting going stale` | Fires once when phone hasn't reported for 60+ min |
| Automation 2 | `Location reporting recovered` | Fires once when phone resumes reporting |
| Notify channel | `notify.mobile_app_a36_herwig` | LAN-server Android push |

## Watched entity

`device_tracker.local_a36_herwig` — the single device tracker entity created by HA Companion. Both LAN and Tailscale servers update this same entity (the Companion App creates one entity per phone, not per server, despite multi-server config).

The trigger watches the entity's `last_reported` metadata attribute, which ticks on every update regardless of state change.

## Automation 1 — Going stale

- **Trigger:** Template
  ```jinja
  {{ (now() - states.device_tracker.local_a36_herwig.last_reported) > timedelta(minutes=60) }}
  ```
- **Condition:** `input_boolean.location_reporting_stale` is `off` (guarantees single notification)
- **Actions, in order:**
  1. `input_boolean.turn_on` on `input_boolean.location_reporting_stale`
  2. `notify.mobile_app_a36_herwig`
     - Title: `HA: location reporting stale`
     - Message: `⚠️ Phone location reporting has gone stale (no update in 60+ min). Likely needs Tailscale server re-add in HA Companion.`

## Automation 2 — Recovered

- **Trigger:** Template
  ```jinja
  {{ (now() - states.device_tracker.local_a36_herwig.last_reported) < timedelta(minutes=15) }}
  ```
- **Condition:** `input_boolean.location_reporting_stale` is `on`
- **Actions, in order:**
  1. `input_boolean.turn_off` on `input_boolean.location_reporting_stale`
  2. `notify.mobile_app_a36_herwig`
     - Title: `HA: location reporting recovered`
     - Message: `✅ Phone location reporting has resumed.`

## Recovery procedure when alert fires

When the "going stale" notification arrives:

1. Open HA Companion App on the phone
2. Settings → Companion app → **Servers**
3. Tap the **Tailscale server** entry (`home Vaison - Tailscale`)
4. **Delete** the server
5. **Add server** → URL `http://100.66.121.78:8123` → name `home Vaison - Tailscale` → log in
6. Accept "less secure / allow cleartext" prompt (cleartext OK; Tailscale encrypts the transport)
7. Verify location reporting resumes (Settings → Companion app → Troubleshooting → Location tracking)
8. The "recovered" notification should arrive within 15 min

⚠ Note: native HA action widgets bound to the Tailscale server may need rebinding after the re-add (server gets a new internal ID).

## Testing

Real-world test performed and passed:
1. Disabled HA Companion → Manage sensors → Background location
2. Waited ~60 min → "going stale" notification arrived, helper flipped to `on`
3. Re-enabled Background location
4. Within ~15 min → "recovered" notification arrived, helper flipped to `off`

Both transitions clean, single notification each, no flooding.

## Future improvements (deferred)

- **Layer 2 — Tailscale-specific watchdog:** Node-RED ping to phone's Tailscale IP every 5 min, with same state-machine pattern. Would catch single-path Tailscale breaks that Layer 1 misses. Build if a third incident occurs.
- **Email fallback (option b from original plan):** add email notification for cases where push delivery itself fails. Requires SMTP integration.
- **Persistent HA banner (option c):** add `notify.persistent_notification` action so a yellow banner appears inside HA. Useful for at-desk visibility.
