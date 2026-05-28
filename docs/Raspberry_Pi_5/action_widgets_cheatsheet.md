# HA Companion action widgets

Native HA Companion widgets that call an HA action directly when tapped. Shorter chain than Tasker → webhook → HA. Built-in green/red success indicator.

## When to use which widget type

| Use case | Tool |
|---|---|
| Local quick-action (at home, on LAN) | **HA Action widget** |
| Remote quick-action (away from home) | **Tasker widget** (webhook over Tailscale) |
| Both | one of each |

Native action widgets currently fail with "Unable to send action" when bound to the Tailscale server, despite Tailscale being fully functional. Cause unknown. Use LAN for native widgets; use Tasker for the remote case.

## Creating an action widget

1. HA Companion → Settings → Manage widgets → Add widget → **Action button**
   *(or long-press empty homescreen → Widgets → Home Assistant → Action button)*

2. Configure:
   - **Server**: `home Vaison - LAN` (Tailscale won't work — known issue)
   - **Action**: e.g. `switch.turn_on`, `switch.turn_off`, `switch.toggle`, `scene.turn_on`, `script.<name>`, `light.turn_on`, `cover.open_cover`
   - **Entity / Target**: use the **entity ID** (e.g. `switch.stopcontact_hobby_kamer`), **not** the friendly name
   - **Label**: what shows on the widget
   - **Icon**: MDI icon, e.g. `mdi:power-socket-eu`, `mdi:lightbulb-on`
   - **Require confirmation**: off for trivial actions, on for high-impact ones (e.g. alarm disarm)

3. Save → place on homescreen → tap to test (green checkmark = success).

## Finding the entity ID

Settings → Devices & services → device → entity → settings icon. Entity ID is the `domain.something_something` form (lowercase, underscores). **Not** auto-updated when friendly name changes — always verify.

## Design patterns

- **Toggle vs separate on/off** — use `switch.toggle` for things you flip often (one widget instead of two). Use separate widgets when state visibility matters.
- **Scenes for grouped actions** — define `scene.evening_mode` in HA (multiple devices), point one widget at it. Keep complexity in HA, simplicity in widgets.
- **Folders per room** — Samsung folders keep dozens of widgets organised.
- **Color-code by category** — lights one color, outlets another, scenes a third.

## Troubleshooting "Unable to send action"

1. Wrong entity ID (used friendly name instead) — fix to entity ID
2. Server unreachable — check active server in Companion settings; LAN works, Tailscale currently doesn't
3. Test the action server-side: Developer tools → Actions tab → call the same action manually. If that works, the widget config is the issue.

## Caveats

- Widget needs **background data** enabled and **Data Saver off** for the HA app
- Widgets do **not** auto-update when entity IDs change — recreate if the underlying entity is replaced
