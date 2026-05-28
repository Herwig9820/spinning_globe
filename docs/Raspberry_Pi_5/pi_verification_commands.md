# Pi-HTA — Verification Cheat Sheet

Commands to confirm the home automation stack is healthy. Run from any SSH session as user `herwig`.

Stack components verified below:

1. OS + system health
2. Tailscale
3. Mosquitto
4. Node-RED
5. Zigbee2MQTT (and the Sonoff dongle)
6. End-to-end pipeline test
7. Reboot resilience test

---

## 0. The 30-second one-shot health check

If you only run one thing, run this:

```bash
systemctl is-active mosquitto nodered zigbee2mqtt tailscaled
```

Expected: four lines, each saying `active`. Anything else → drill down into the relevant section below.

For a slightly fuller picture:

```bash
systemctl status mosquitto nodered zigbee2mqtt tailscaled --no-pager | grep -E "●|Active:"
```

Each service should show `Active: active (running)` in green.

---

## 1. OS + system health

```bash
# Hostname / OS / kernel
hostnamectl

# Uptime + load
uptime

# Memory and disk free
free -h
df -h /

# Recent kernel errors (last boot only). Should be empty or only benign warnings.
journalctl -p err -b --no-pager | tail -30

# CPU temperature (Pi 5 should sit well under 80 °C with the fan)
vcgencmd measure_temp

# Throttling history (should print throttled=0x0)
vcgencmd get_throttled
```

`get_throttled = 0x0` means no under-voltage or thermal throttling has happened since boot. Anything else suggests PSU or cooling problems.

---

## 2. Tailscale

```bash
# Daemon running?
systemctl is-active tailscaled

# Connected, IP allocated, peers visible?
tailscale status

# Just your tailnet IP
tailscale ip -4

# Connectivity to coordination server (should print "Online")
tailscale netcheck 2>&1 | head -20
```

In `tailscale status` you should see your Pi's tailnet name on the first line, then a list of peer machines (your PC, phone, etc.). Each line ends in either `idle`, `active`, or `-` (never connected this session) — all are normal.

---

## 3. Mosquitto

### Service running

```bash
systemctl is-active mosquitto
systemctl is-enabled mosquitto    # should say 'enabled' (auto-start on boot)
```

### Listening on port 1883

```bash
ss -tlnp 2>/dev/null | grep 1883
```

You should see one or two lines with `LISTEN` and `:1883`. If empty → broker is not actually serving.

### Recent log (no errors)

```bash
journalctl -u mosquitto -n 30 --no-pager
```

Look for `mosquitto version X.Y.Z starting` and `Opening ipv4 listen socket on port 1883`. No red `error` lines.

### The three MQTT users exist

```bash
sudo cat /etc/mosquitto/passwd | cut -d: -f1
```

Expected output, one per line:

```
herwig
spinning_globe
zigbee2mqtt
```

### Auth actually works (live login test)

This subscribes for 2 seconds then exits — proves the broker accepts the credential and the topic ACL is open:

```bash
timeout 2 mosquitto_sub -h localhost -t '$SYS/broker/version' -u herwig -P 'YOUR-HERWIG-PW' -v
```

Expected: a single line like `$SYS/broker/version mosquitto version 2.0.x`. If you get `Connection Refused: not authorised` → wrong password. If `Connection refused` → broker not running.

(Replace `herwig` with `spinning_globe` or `zigbee2mqtt` and the matching password to test the other accounts.)

---

## 4. Node-RED

### Service running

```bash
systemctl is-active nodered
systemctl is-enabled nodered
```

### Listening on port 1880

```bash
ss -tlnp 2>/dev/null | grep 1880
```

Should show `0.0.0.0:1880` (bound on all interfaces — required so PC and phone can reach it).

### Recent log (no errors)

```bash
journalctl -u nodered -n 40 --no-pager
```

Look for `Started flows` near the end. Red lines about missing nodes are worth investigating; warnings about deprecated APIs are usually harmless.

### Hardening present (admin auth + credential secret)

```bash
grep -E "^\s*(uiHost|contextStorage|credentialSecret|adminAuth)" ~/.node-red/settings.js
```

Expected (values redacted in real output):

- `credentialSecret: "..."` — uncommented, non-empty
- `adminAuth: { ... }` — uncommented block
- `contextStorage: { default: { module: 'localfilesystem' } }` — uncommented

If any of these is commented out (line starts with `//`), the corresponding security/persistence feature is OFF.

### Editor reachable

From your PC browser: `http://pi-hta:1880` should show the login page (because `adminAuth` is set). Logging in as `herwig` lands on the flow editor.

---

## 5. Zigbee2MQTT (and the Sonoff dongle)

### Dongle physically present

```bash
lsusb | grep -i itead
```

Expected: one line containing `ITEAD` (or `Sonoff`). Empty → dongle not detected; check the USB cable / port.

### Stable serial path (the one in `configuration.yaml`)

```bash
ls -l /dev/serial/by-id/ | grep -i sonoff
```

Should show a symlink whose name starts with `usb-Itead_Sonoff_Zigbee_3.0_USB_Dongle_Plus_V2_...-if00-port0` pointing to `/dev/ttyUSB0` (or `ttyACM0`). If the file is missing, the dongle is not enumerated.

### Service running

```bash
systemctl is-active zigbee2mqtt
systemctl is-enabled zigbee2mqtt
```

### Status with watchdog detail

```bash
systemctl status zigbee2mqtt --no-pager
```

Look for:

- `Active: active (running)` (green)
- `Status: "Healthy"` — the string published via `Type=notify`. Confirms the watchdog handshake is alive.
- No red error lines.

### Recent log

```bash
journalctl -u zigbee2mqtt -n 40 --no-pager
```

Healthy markers near the bottom:

- `zigbee-herdsman started (resumed)` — picked up the existing network. (`(reset)` would indicate it formed a new one — not what you want on a normally-running system.)
- `Connected to MQTT server`
- `Zigbee2MQTT started!`

### Frontend reachable

From your PC: `http://pi-hta:8080` → Z2M dashboard, devices list, network map.

### Z2M is publishing to MQTT (subscribe to its bridge state)

```bash
timeout 5 mosquitto_sub -h localhost -t 'zigbee2mqtt/bridge/state' -u herwig -P 'YOUR-HERWIG-PW' -v
```

Expected: a line like `zigbee2mqtt/bridge/state {"state":"online"}` (Z2M publishes this as a retained message, so it shows up immediately). No output → Z2M isn't connected to the broker.

---

## 6. End-to-end pipeline test

This is the real "everything works together" check. It proves: dongle → Z2M → Mosquitto → MQTT consumer.

In one SSH window, subscribe to the entire Z2M topic tree:

```bash
mosquitto_sub -h localhost -t 'zigbee2mqtt/#' -v -u herwig -P 'YOUR-HERWIG-PW'
```

You should see, immediately:

- `zigbee2mqtt/bridge/state {"state":"online"}`
- `zigbee2mqtt/bridge/info { ... long JSON with version, coordinator, network ... }`
- `zigbee2mqtt/bridge/devices [ ... ]` — currently `[]` since nothing is paired yet
- `zigbee2mqtt/bridge/groups [ ... ]`

These four retained messages confirm the full chain. Once you pair the Niko outlet, you'll start seeing `zigbee2mqtt/<friendly_name>` messages in this same window. Ctrl+C to exit.

---

## 7. Reboot resilience (run occasionally, not daily)

The whole point of `systemctl enable` and `Restart=always`. Confirms the stack is genuinely self-healing:

```bash
sudo reboot
```

Wait ~60–90 seconds, SSH back in, then:

```bash
systemctl is-active mosquitto nodered zigbee2mqtt tailscaled
```

All four `active` → industry-grade box. Anything else → that service's `enable` or `Restart=` is misconfigured.

---

## Where things live (reference)

| Component       | Service name   | Port        | Config / key files                                                  |
|-----------------|----------------|-------------|---------------------------------------------------------------------|
| Tailscale       | `tailscaled`   | —           | `/var/lib/tailscale/`                                               |
| Mosquitto       | `mosquitto`    | 1883        | `/etc/mosquitto/mosquitto.conf`, `/etc/mosquitto/conf.d/*.conf`, `/etc/mosquitto/passwd` |
| Node-RED        | `nodered`      | 1880        | `~/.node-red/settings.js`, `~/.node-red/flows.json`, `~/.node-red/flows_cred.json` |
| Zigbee2MQTT     | `zigbee2mqtt`  | 8080 (UI)   | `/opt/zigbee2mqtt/data/configuration.yaml`, `/opt/zigbee2mqtt/data/coordinator_backup.json` |

| Log tail (live)            | Command                                  |
|----------------------------|------------------------------------------|
| Mosquitto                  | `journalctl -u mosquitto -f`             |
| Node-RED                   | `journalctl -u nodered -f`               |
| Zigbee2MQTT                | `journalctl -u zigbee2mqtt -f`           |
| Tailscale                  | `journalctl -u tailscaled -f`            |
| Everything (kernel + all)  | `journalctl -f`                          |

Ctrl+C exits any `-f` (follow) view.
