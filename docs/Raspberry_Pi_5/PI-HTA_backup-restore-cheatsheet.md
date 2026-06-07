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

**Step 1 – run the snapshot on the Pi**
```powershell
ssh pi "~/scripts/pi_snapshot.sh"
```

**Step 2 – copy to a temp folder, then move to OneDrive**

Windows `scp.exe` cannot handle spaces in the destination path, so we stage via a temp folder:
```powershell
Remove-Item -Recurse -Force D:\pi_snapshot_tmp -ErrorAction SilentlyContinue
scp -r pi:~/pi_snapshot D:\pi_snapshot_tmp
robocopy D:\pi_snapshot_tmp "D:\herwig\cloud\OneDrive\Onedrive Documents\myLab\Arduino\spinning_globe\Raspberry_Pi_5\pi_snapshot" /E /PURGE
Remove-Item -Recurse -Force D:\pi_snapshot_tmp
```
`/E` copies all subfolders; `/PURGE` removes stale files at the destination — no manual delete needed.

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

**Step 1 — Copy the snapshot back from OneDrive to the Pi** (from PowerShell):
```powershell
scp -r "D:\herwig\cloud\OneDrive\Onedrive Documents\myLab\Arduino\spinning_globe\Raspberry_Pi_5\pi_snapshot" pi:~/pi_snapshot_restore
```
> Spaces in the *source* path are fine — the scp mkdir issue only affects the *destination*. Here the destination is the Pi, which has no spaces.

**Step 2 — Copy the file you need back into place** (on the Pi):
```bash
# Example: restore Mosquitto config
sudo cp ~/pi_snapshot_restore/mosquitto/local.conf /etc/mosquitto/conf.d/
sudo systemctl restart mosquitto
```
Same pattern for any other service config — just adjust the source path and destination.

### Scenario 2 — Service broken, OS intact

1. Run healthcheck to identify what's broken: `pi_healthcheck.sh`
2. Restore the snapshot from OneDrive to the Pi (see Scenario 1, Step 1)
3. Restore relevant config files from `~/pi_snapshot_restore/` (see Scenario 1, Step 2)
4. If Docker/HA broken: `cd /opt/homeassistant && docker compose down && docker compose up -d`
5. If Z2M broken: restore `configuration.yaml` and `coordinator_backup.json` from the snapshot, then `sudo systemctl restart zigbee2mqtt`

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
| `~/pi_snapshot/` (on Pi) | Config files, service state | Medium — contains configs |
| `~/pi_snapshot/zigbee2mqtt/coordinator_backup.json` | Zigbee network key | 🔴 High — keep secure |
| `...\raspberry_Pi_5\pi_snapshot\` (OneDrive) | Config snapshot folder | Medium |
| `D:\herwig\pi-backup-YYYYMMDD\*.img.gz` | Full SSD image | 🔴 High — contains all secrets |

---

## SSH access reminder

| Alias | When to use | Address |
|-------|-------------|---------|
| `ssh pi` | Always works (via Tailscale) | 100.66.121.78 |
| `ssh pi-eth`  | When on same LAN as Pi (ethernet) | 192.168.0.119 |
| `ssh pi-lan1` | When on same LAN as Pi (homeNet1) | 192.168.0.120 |
| `ssh pi-lan2` | When on same LAN as Pi (homeNet2) | 192.168.1.120 |

- Key-based auth only — **password login is disabled**
- Private key lives on Windows at default SSH key location
- `scp` uses same aliases: `scp myfile.txt pi:~/destination/`

---

## Quick reference — useful commands

```bash
# Health check
pi_healthcheck.sh

# Config snapshot
sudo ~/scripts/pi_snapshot.sh

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
