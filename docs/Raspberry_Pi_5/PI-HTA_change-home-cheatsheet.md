# PI-HTA — "Change Home" Cheatsheet

## The good news: no system stack setup changes on the Pi

The stack is designed with `localhost` throughout — no hardcoded LAN IPs in
Mosquitto, Zigbee2MQTT, or Home Assistant. Both home networks are pre-configured
on the Pi (static IPs, WiFi credentials). Moving the Pi to a different network
requires only **changing one functional HA server setting** and **two taps in the HA Android app**.

---

## On the Pi — no system stack changes

| Component | Config | Action needed |
|-----------|--------|---------------|
| Mosquitto | Binds to all interfaces, no IP hardcoded | ✅ Nothing |
| Zigbee2MQTT | Uses `mqtt://localhost` | ✅ Nothing |
| Home Assistant | Uses `localhost` for MQTT (network_mode: host) | ✅ Nothing |
| Node-RED | ESP32 via mDNS, MQTT via localhost | ✅ Nothing |
| Tailscale | Network-independent by design | ✅ Nothing |
| Network / IP | Static IPs pre-configured via nmcli for both homes | ✅ Nothing |
| WiFi credentials | Both `homeNet1` and `homeNet2` pre-configured | ✅ Nothing |

Just plug the Pi into the new network (Ethernet at home 1, or power on for WiFi)
and wait ~60s for full boot. Everything comes up on its own.


## Update the Home Assistant Home Location (HA server functional setting)

Settings → System → Home Information holds the home location (lat/long),
unit system, time zone, currency, country. The home location set here should
match the Home zone. During prototyping in the second home (Vaison-la-Romaine, France)
it can be set to that location; will need updating to the Belgium first home
on the permanent move.

---

## On the Android app — 2 changes

Open **Settings → Companion app → (local server entry)**

### 1. Update the LAN URL
Update the internal/local server URL to the Pi's IP on the new network:

| Home | URL |
|------|-----|
| Home 1 | `http://192.168.0.120:8123` |
| Home 2 | `http://192.168.1.120:8123` |

> Tailscale always works regardless of which home you're in (`http://100.66.121.78:8123`).

### 2. Update the Home Network SSID
In the same server entry, update the **Home network** SSID to match the new home.

The app will then automatically use:
- LAN URL when connected to the home WiFi network
- Tailscale URL (`http://100.66.121.78:8123`) everywhere else
---

## Checklist — moving to home 1

- [ ] Pi plugged into home 1 network (Ethernet preferred, or WiFi via `homeNet1`)
- [ ] Pi powered on — wait ~60s for full boot
- [ ] Verify via Tailscale SSH: `ssh pi` then `pi_healthcheck.sh`
- [ ] HA server: update Home Location
- [ ] Android app: update LAN URL to `http://192.168.0.120:8123`
- [ ] Android app: update Home network SSID to `homeNet1`
- [ ] Test app on WiFi → should reach Pi via LAN
- [ ] Test app on 4G (WiFi off) → should reach Pi via Tailscale

## Checklist — moving back to home 2

- [ ] Pi powered on (connects to `homeNet2` automatically)
- [ ] Verify via Tailscale SSH: `ssh pi` then `pi_healthcheck.sh`
- [ ] HA server: update Home Location
- [ ] Android app: update LAN URL to `http://192.168.1.120:8123`
- [ ] Android app: update Home network SSID to `homeNet2`
- [ ] Test app on WiFi → should reach Pi via LAN
- [ ] Test app on 4G (WiFi off) → should reach Pi via Tailscale
