# PI-HTA — Backup & Restore Cheatsheet

---

## Backup strategy overview

| Type | Tool | What it captures | Size | How often |
|------|------|-----------------|------|-----------|
| Config snapshot | `pi_snapshot.sh` | Config files, service state, device DB | ~150 KB | Before any change |
| Full SSD image | `dd` over SSH | Everything — OS, data, Docker, all configs | ~5–6 GB compressed | Before major changes |

Both are **complementary** — the snapshot is fast and readable; the image is the ultimate safety net.

---

## Taking a backup

### A. Config snapshot (fast, ~10 seconds)

Run on the Pi:
```bash
sudo ~/scripts/pi_snapshot.sh
```
Output lands in `~/pi_snapshot/`. To archive it to OneDrive, tar it up from Windows PowerShell:
```powershell
ssh pi "tar czf ~/pi_snapshot_$(date +%Y%m%d).tar.gz ~/pi_snapshot"
```
Then copy to OneDrive:
```powershell
scp pi:~/pi_snapshot_*.tar.gz "D:\herwig\cloud\OneDrive\Onedrive Documents\myLab\Arduino\spinning_globe\raspberry_Pi_5\"
```
Clean up on Pi afterward:
```bash
rm ~/pi_snapshot_*.tar.gz
```

### B. Full SSD image (slow, 60 min)

**Step 1 — Zero-fill free space first** (makes image compress well):
```bash
dd if=/dev/zero of=/home/herwig/zero.tmp bs=4M status=progress; rm /home/herwig/zero.tmp
```
Expected output: fills ~200+ GB, ends with "No space left on device" — normal.

**Step 2 — Stream image to Windows** (from PowerShell):
```powershell
mkdir D:\herwig\pi-backup-YYYYMMDD
ssh pi "sudo dd if=/dev/sda bs=4M status=none | gzip" > "D:\herwig\pi-backup-YYYYMMDD\pi-hta-LABEL.img.gz"
```
Replace `YYYYMMDD` and `LABEL` (e.g. `pre-docker`, `post-docker`).
Expected output file size: ~5–6 GB.

**Verify size when done:**
```powershell
(Get-Item "D:\herwig\pi-backup-YYYYMMDD\pi-hta-LABEL.img.gz").length / 1GB
```

---

## Restoring from backup

### Scenario 1 — Single config file gone wrong

Restore from snapshot tar. Example: restore Mosquitto config:
```bash
# Extract snapshot on Pi
tar xzf ~/pi_snapshot_YYYYMMDD.tar.gz -C /tmp
# Copy the file you need back into place
sudo cp /tmp/pi_snapshot/mosquitto/local.conf /etc/mosquitto/conf.d/
sudo systemctl restart mosquitto
```
Same pattern for any other service config.

### Scenario 2 — Service broken, OS intact

1. Run healthcheck to identify what's broken: `pi_healthcheck.sh`
2. Restore relevant config files from snapshot (see Scenario 1)
3. If Docker/HA broken: `cd /opt/homeassistant && docker compose down && docker compose up -d`
4. If Z2M broken: restore `configuration.yaml` and `coordinator_backup.json` from snapshot, then `sudo systemctl restart zigbee2mqtt`

> ⚠️ The `coordinator_backup.json` contains the Zigbee network key.
> Without it, all paired devices must be re-paired from scratch.
> Always keep a recent snapshot.

### Scenario 3 — Full disaster (dead SSD or total corruption)

**What you need:**
- A new SSD (same or larger — Philips 250GB USB-C portable SSD)
- The `.img.gz` file from Windows
- A Windows PC with WSL or Win32DiskImager

**Option A — Write image with `dd` in WSL (recommended):**
```bash
# In WSL on Windows — find the new SSD device
lsblk
# Write image (replace /dev/sdX with correct device — TRIPLE CHECK this)
gunzip -c /mnt/d/herwig/pi-backup-YYYYMMDD/pi-hta-LABEL.img.gz | sudo dd of=/dev/sdX bs=4M status=progress
sync
```
> ⚠️ `dd` will overwrite whatever disk you point it at with no confirmation.
> Double-check the device letter before running.

**Option B — Win32DiskImager (GUI, safer for beginners):**
1. Decompress the `.img.gz` to `.img` first (use 7-Zip)
2. Open Win32DiskImager
3. Select the `.img` file and the correct drive letter
4. Click Write

**After writing:**
1. Insert SSD into Pi, power on
2. SSH in via Tailscale: `ssh pi`
3. Run healthcheck: `pi_healthcheck.sh`
4. Verify all services: `sudo systemctl is-active mosquitto zigbee2mqtt nodered && docker ps`

---

## Backup file inventory

| File/Location | Contents | Sensitivity |
|---------------|----------|-------------|
| `~/pi_snapshot/` | Config files, service state | Medium — contains redacted configs |
| `~/pi_snapshot/zigbee2mqtt/coordinator_backup.json` | Zigbee network key | 🔴 High — keep secure |
| `D:\herwig\pi-backup-YYYYMMDD\*.img.gz` | Full SSD image | 🔴 High — contains all secrets |
| OneDrive snapshot tar | Config snapshot archive | Medium |

---

## SSH access reminder

| Alias | When to use | Address |
|-------|-------------|---------|
| `ssh pi` | Always works (via Tailscale) | 100.66.121.78 |
| `ssh pi-lan` | When on same LAN as Pi | 192.168.1.120 |

- Key-based auth only — **password login is disabled**
- Private key lives on Windows at default SSH key location
- `scp` uses same aliases: `scp myfile.txt pi:~/destination/`

---

## Quick reference — useful commands

```bash
# Health check
pi_healthcheck.sh

# Config snapshot
sudo pi_snapshot.sh

# Check all services at a glance
sudo systemctl is-active mosquitto zigbee2mqtt nodered && docker ps

# HA container logs
docker logs homeassistant --tail 50

# Restart HA
cd /opt/homeassistant && docker compose restart

# Restart Z2M
sudo systemctl restart zigbee2mqtt

# Restart Mosquitto
sudo systemctl restart mosquitto

# Restart Node-RED
sudo systemctl restart nodered
```
