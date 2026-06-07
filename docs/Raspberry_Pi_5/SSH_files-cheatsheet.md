# SSH Files Cheatsheet

## Folder: `C:\Users\herwi\.ssh\`

| File | Contains | Used by | Notes |
|------|----------|---------|-------|
| `config` | SSH aliases (Host, HostName, User, IdentityFile) | SSH & SCP client on Windows | Defines `pi`, `pi-eth`, `pi-lan1`, `pi-lan2` shortcuts |
| `id_ed25519` | **Your private key** | SSH & SCP client on Windows | 🔴 Never share, never copy to cloud. If lost, generate a new keypair and re-register on the Pi |
| `id_ed25519.pub` | **Your public key** | Copied to the Pi during setup | Safe to share — this is what gets placed in `authorized_keys` on the server |
| `authorized_keys` | Public keys of clients allowed to connect **to this Windows machine** | Windows OpenSSH server (if enabled) | Normally empty/unused if you don't SSH *into* Windows |
| `known_hosts` | Fingerprints of servers you've connected to before | SSH client on Windows | Protects against man-in-the-middle attacks — if a server's fingerprint changes unexpectedly, SSH warns you |

---

## On the Pi: `/home/herwig/.ssh/`

| File | Contains | Notes |
|------|----------|-------|
| `authorized_keys` | Your public key (`id_ed25519.pub`) | This is what was copied to the Pi during setup — it's what allows your Windows machine to connect without a password |

---

## How they work together

```
Windows                              Pi
──────────────────────────────────────────────────────
id_ed25519 (private key)    ←→    authorized_keys (your public key)
config (aliases)             →    connects to HostName with User
known_hosts                  ←    stores Pi's server fingerprint
```

1. You run `ssh pi`
2. Windows looks up `pi` in `config` → finds IP, username, key
3. Pi checks if your public key is in `authorized_keys` → yes → access granted
4. Windows stores Pi's fingerprint in `known_hosts` on first connect

---

## Key rule
> 🔴 `id_ed25519` (private key) never leaves `C:\Users\herwi\.ssh\`
> 🟢 `id_ed25519.pub` (public key) can be placed on any server you want to access
