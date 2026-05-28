# Tasker — HA control via webhooks

Tasker is used for **remote control** of HA (when away from home) via webhook calls over Tailscale. Native HA action widgets handle the local case; Tasker handles what they currently can't.

## Architecture

```
Phone (Tasker widget tap)
  → Tasker task (HTTP Post action)
  → HA webhook endpoint over Tailscale or LAN
  → HA automation triggered
  → Zigbee2MQTT / device action
```

## Tasker concepts

| Concept | Meaning |
|---|---|
| **Task** | A sequence of actions (the thing Tasker *does*) |
| **Profile** | A trigger (when X, run task Y) |
| **Action** | A single step inside a task (e.g. HTTP Post) |
| **Widget** | Homescreen icon bound to a task — tap to run |

For our use case (HA control), **tasks + widgets** are what matter. Profiles are not needed for tap-driven control.

## Creating a Tasker → HA webhook task

1. Tasker → Tasks tab → **+** → name (e.g. `Niko outlet 1 On`)
2. Add action: **Net → HTTP Post**
   *(important: HTTP Post works; HTTP Request does not in our setup)*
3. Configure:
   - **Server:Port** — `http://192.168.1.120:8123` (LAN) or `http://100.66.121.78:8123` (Tailscale)
   - **Path** — `/api/webhook/niko_on` (or whatever the HA webhook ID is)
   - **Body** — empty
   - **Headers** — empty (HA webhooks don't require auth)
4. Save the task

## HA side — the matching webhook automation

In HA (Settings → Automations & scenes → create new):
- Trigger: **Webhook**
- Webhook ID: `niko_on` (matches the Tasker URL path)
- Action: whatever you want fired (e.g. `switch.turn_on` on the outlet)

## Creating a Tasker widget on the homescreen

1. Long-press empty homescreen → Widgets → **Tasker** → drag **Task** widget out
2. Pick the task to bind
3. Set label and icon
4. Tap to test

## LAN vs Tailscale widgets

Tasker widgets work on **either** LAN or Tailscale, depending on the URL configured in the task. Two strategies:

- **One widget per server** (e.g. `Niko On (LAN)` and `Niko On (Tail)`) — explicit, no auto-detection logic
- **One widget with conditional logic** — task uses an `If WIFI SSID = home` action to pick the URL. More complex; useful if homescreen real estate is tight.

For most cases, two widgets is simpler and more robust.

## Why Tasker over native HA widgets for remote use

- HA Companion native action widgets currently fail on the Tailscale server (known issue)
- Tasker over Tailscale works reliably
- Decision: native widgets for LAN, Tasker for Tailscale, both coexist

## Permissions & Samsung-specific gotchas

Samsung kills background apps aggressively. For Tasker to stay reliable:
- Settings → Apps → Tasker → Battery → **Unrestricted**
- Settings → Apps → Tasker → permissions: **Display over other apps**, **Notifications** enabled
- Do not enable Data Saver on the device, or whitelist Tasker and HA Companion

## Debugging

Tasker → three-dot menu → **More → Run Log** — chronological list of every task execution and result. The first place to look when a widget tap doesn't produce the expected result.

If Tasker fires correctly but HA doesn't act:
- Test the webhook URL directly in a phone browser (or `curl` from a laptop) — `POST` to `http://192.168.1.120:8123/api/webhook/niko_on`
- Check HA's Logbook (Sidebar → Logbook) for the automation firing or not
