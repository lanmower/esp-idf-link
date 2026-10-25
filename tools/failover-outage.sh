#!/bin/sh
# Host-AP outage for the failover test. RUN ON THE PI, as root, by the mc-420 agent.
#   failover-outage.sh [hold_seconds]     default 100
# Sequence (as agreed with mc-420): autoap stop -> kill hostapd -> hold -> autoap start.
# Prints one timestamped line per sample. The ESP's behaviour is judged from its own serial
# log (STA down n/30, re-hosting, tie-break, Link peer back), not from this script.
# Busybox-safe: no pgrep -c, no pkill.
HOLD=${1:-100}

snap() {
  n=$(pgrep hostapd 2>/dev/null | wc -l)
  info=$(iw dev wlan0 info 2>/dev/null)
  ssid=$(echo "$info" | sed -n 's/^[[:space:]]*ssid //p')
  type=$(echo "$info" | sed -n 's/^[[:space:]]*type //p')
  sta=$(iw dev wlan0 station dump 2>/dev/null | grep -c '^Station')
  echo "$(date -u +%FT%TZ) $1 hostapd=$n ssid=${ssid:-none} type=${type:-none} stations=$sta"
}

snap BEFORE
rc-service autoap stop
for p in $(pgrep hostapd 2>/dev/null); do kill "$p"; done
echo "$(date -u +%FT%TZ) STOP"

elapsed=0
while [ "$elapsed" -lt "$HOLD" ]; do
  sleep 10
  elapsed=$((elapsed + 10))
  snap "DURING+${elapsed}"
done

rc-service autoap start
echo "$(date -u +%FT%TZ) START"
sleep 20; snap AFTER+20
sleep 40; snap AFTER+60
