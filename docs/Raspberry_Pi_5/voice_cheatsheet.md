# Voice control — HA Assist (Dutch)

> **Status (May 2026):** the HA Companion App on Android is **not yet mature enough for reliable one-step voice recognition** on the Samsung A36. Wake-word detection ("Hey Mycroft" / "Okay Nabu") triggers unreliably and frequently fails with "Unable to receive audio." Native homescreen widget/shortcut requires 2–3 taps despite the "Start listening" toggle.
>
> Setup below is fully wired and ready. Voice path itself is parked until either the Android wake-word implementation matures or an ESP32 voice satellite is added at the Belgium install.

## What's set up and working

- HA Assist language: **Nederlands**
- Outlet entity exposed to Assist
- Friendly name: `stopcontact hobbykamer` (compound noun, one word — matches how Google STT transcribes Dutch)
- Alias: `lamp hobbykamer`
- Typed commands in Assist chat work fully (e.g. `zet stopcontact hobbykamer aan`)
- Voice via mic button in Assist works when manually triggered

## Naming conventions for future entities

1. Match how Google STT writes Dutch — compound nouns one word (`hobbykamer`, `slaapkamer`, `woonkamer`, `keuken`).
2. **Test names by speaking** them in Assist before committing. Whatever the STT echoes back is the canonical form.
3. Add aliases generously for natural variation (e.g. `stopcontact hobbykamer`, `lamp hobbykamer`, `hobbykamerlamp`).

## How to set up a new entity for voice

1. Settings → Voice assistants → exposed entities → add the entity
2. Settings → Devices & services → entity → gear icon → set friendly name and aliases
3. Test by typing the command in Assist chat first
4. Add more aliases if speech recognition mishears or natural phrasing differs

## Finding what commands exist (deterministic, no trial-and-error)

**Bookmarks — Dutch intent templates and docs:**

- Dutch sentence templates per domain:
  `https://github.com/home-assistant/intents/tree/main/sentences/nl`
- HA Conversation integration docs (custom sentences, YAML extension):
  `https://www.home-assistant.io/integrations/conversation/`
- All supported intents reference:
  `https://github.com/home-assistant/intents/tree/main/intents`

**How to use the Dutch templates:**

- File naming: `<domain>_<intent>.yaml`
  e.g. `light_HassTurnOn.yaml`, `switch_HassTurnOff.yaml`, `cover_HassSetPosition.yaml`
- Inside each file, `sentences:` lists the phrase patterns
- Grammar: `[optional]`, `{entity_name_slot}`, `(alternative1|alternative2)`

## Future re-evaluation triggers

- HA Companion App wake-word reliability improves (track GitHub `home-assistant/android` issues)
- ESP32 voice satellite (e.g. M5Stack Atom Echo, ESP32-S3-Box) installed at Belgium home → uses HA Wyoming protocol, works reliably today
