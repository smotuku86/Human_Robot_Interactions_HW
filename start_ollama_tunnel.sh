#!/bin/bash
# Description:
#   Expose Ollama running on Windows to WSL applications via 127.0.0.1:11434
#   Dynamically discovers the Windows host IP and creates a socat tunnel.

set -e

# --- 1. Detect Windows Host IP ---
WIN_HOST_IP=$(ip route show default | awk '/default/ {print $3; exit}')
if [[ -z "$WIN_HOST_IP" ]]; then
    echo "[ERROR] Could not determine Windows host IP." >&2
    exit 1
fi

# --- 2. Verify dependencies ---
if ! command -v socat >/dev/null 2>&1; then
    echo "[ERROR] socat not found. Install it with: sudo apt install socat -y" >&2
    exit 1
fi

# --- 3. Kill any previous socat tunnels (quietly) ---
pkill -f "socat TCP-LISTEN:11434" 2>/dev/null || true

# --- 4. Start tunnel ---
echo "[INFO] Starting socat tunnel: 127.0.0.1:11434 -> ${WIN_HOST_IP}:11434"
nohup socat TCP-LISTEN:11434,reuseaddr,fork TCP:${WIN_HOST_IP}:11434 >/dev/null 2>&1 &
TUNNEL_PID=$!

# --- 5. Give socat time to initialize ---
sleep 1

# --- 6. Verify connectivity ---
if curl -s -o /dev/null --fail http://127.0.0.1:11434/api/tags; then
    echo "[SUCCESS] Ollama reachable at http://127.0.0.1:11434 (PID: ${TUNNEL_PID})"
else
    echo "[ERROR] Tunnel failed. Verify Ollama is running and Firewall allows connections." >&2
    kill "${TUNNEL_PID}" 2>/dev/null || true
    exit 1
fi