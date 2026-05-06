# Pi configuration snapshot

Snapshot of the Raspberry Pi 5 configuration.

This is documentation, not deployment. Files here are static records:
copies of config files, command output, and setup notes. They are not 
synced from the live Pi automatically.

To restore on a new Pi:
1. Install Pi OS (full image) on USB SSD
2. Apply settings from system_info.txt
3. Install services per installed_packages.txt
4. Apply Mosquitto config from mosquitto/local.conf
5. Restore Node-RED flows from nodered/flows_pi.json
6. (Optional) Re-run all the gotchas to remind yourself which broke last time