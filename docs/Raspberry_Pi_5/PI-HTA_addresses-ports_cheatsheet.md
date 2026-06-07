# PI-HTA — Addresses & Ports Reference

---

## Pi IP addresses

| Network | Interface | Address | How set | Notes |
|---------|-----------|---------|---------|-------|
| Tailscale (always works) | `tailscale0` | `100.66.121.78` | Tailscale | Works from anywhere |
| Home 1 LAN | `eth0` (Ethernet) | `192.168.0.119` | Static IP via nmcli (`home1-eth` profile) | Home 1 only — primary |
| Home 1 LAN | `wlan0` (WiFi) | `192.168.0.120` | Static IP via nmcli (`homeNet1` profile) | Home 1 only — fallback |
| Home 2 LAN | `wlan0` (WiFi) | `192.168.1.120` | Static DHCP lease (MAC `88:a2:9e:d6:a1:f6`) | Home 2 only |

### Gateways
| Home | Gateway |
|------|---------|
| Home 1 | `192.168.0.1` |
| Home 2 | `192.168.1.254` |

---

## Services on the Pi

| Service | Port | Local URL | Tailscale URL | Auth |
|---------|------|-----------|---------------|------|
| Home Assistant | 8123 | `http://192.168.x.120:8123` | `http://100.66.121.78:8123` | HA account (herwig) |
| Zigbee2MQTT frontend | 8080 | `http://192.168.x.120:8080` | `http://100.66.121.78:8080` | auth_token |
| Node-RED editor | 1880 | `http://192.168.x.120:1880` | `http://100.66.121.78:1880` | adminAuth |
| Node-RED Dashboard 2 | 1880 | `http://192.168.x.120:1880/dashboard` | `http://100.66.121.78:1880/dashboard` | — |
| Mosquitto MQTT broker | 1883 | `mqtt://localhost:1883` | — | username + password |

> Replace `192.168.x.120` with `192.168.0.120` (home 1) or `192.168.1.120` (home 2)

---

## MQTT users (Mosquitto)

| Username | Used by |
|----------|---------|
| `herwig` | Manual testing, Node-RED |
| `spinning_globe` | Globe project flows |
| `zigbee2mqtt` | Zigbee2MQTT broker connection |
| `homeassistant` | Home Assistant MQTT integration |

---

## SSH access

| Alias | Address | Use |
|-------|---------|-----|
| `ssh pi` | `100.66.121.78` | Via Tailscale — works anywhere |
| `ssh pi-eth`  | `192.168.0.119` | LAN only-ethernet (home 1) |
| `ssh pi-lan1` | `192.168.0.120` | LAN only-WiFi     (home 1) |
| `ssh pi-lan2` | `192.168.1.120` | LAN only-WiFi     (home 2) |

- Key-based auth only — password login disabled
- `scp` uses same aliases: `scp file.txt pi:~/destination/`

---

## NetworkManager WiFi profiles on the Pi

| Profile name | SSID | Home | IP |
|-------------|------|------|----|
| `home1-eth` | — (Ethernet) | Home 1 | `192.168.0.119` (static) |
| `homeNet1`  | `homeNet1`   | Home 1 | `192.168.0.120` (static) |
| `homeNet2`  | `homeNet2`   | Home 2 | `192.168.1.120` (static DHCP lease) |

> Both WiFi networks share the same password by design.

---

## ESP32 (spinning globe)

| Item | Value | Notes |
|------|-------|-------|
| IP | DHCP (dynamic) | Don't rely on it |
| Hostname (mDNS) | `spinning_globe_mqtt_bridge.local` | Use this in Node-RED ping |
| MQTT broker it connects to | `192.168.x.120:1883` | Must match current home LAN |

> ✅ Broker IP is selected automatically by the ESP32 based on which WiFi network
> it connects to — no recompiling needed when moving between homes.
> Defined in firmware as `knownLocations[]`: each entry pairs a WiFi SSID with
> its corresponding MQTT broker IP and port.

---

## Docker

| Item | Value |
|------|-------|
| HA compose file | `/opt/homeassistant/docker-compose.yml` |
| HA config directory | `/opt/homeassistant/config` |
| HA container name | `homeassistant` |
| HA image | `ghcr.io/home-assistant/home-assistant:stable` |
