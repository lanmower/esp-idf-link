#!/bin/sh
# Host-AP outage for the failover test. RUN ON THE PI, as root, by the mc-420 agent.
#   failover-outage.sh [hold_seconds]     default 100
# Sequence (as agreed with mc-420): autoap stop -> kill hostapd -> hold -> autoap start.
# Prints one timestamped line per sample. The ESP's behaviour is judged from its own serial
# log (STA down n/30, re-hosting, tie-break, Link peer back), not from this script.
# Busybox-safe: no pgrep -c, no pkill.
HOLD=${1:-100}

# The AP must come back however this run ends: normal completion, a failed command, or a signal.
# started=0 until the explicit restore, so the EXIT trap restores exactly once.
started=0
scanpid=""
restore() {
  [ "$started" = 1 ] && return
  started=1
  [ -n "$scanpid" ] && kill "$scanpid" 2>/dev/null
  rc-service autoap start >/dev/null 2>&1
  echo "$(date -u +%FT%TZ) START"
}
trap restore EXIT
trap 'exit 1' HUP INT TERM

# Outage-only scan (H2, P7): while the AP is down the Pi's radio is free, so it can see whether the
# ESP is beaconing ticker and at which BSSID. The loop exists only between STOP and START; restore()
# kills it on every exit path. Never runs during P0 or while the Pi is hosting.
scan_loop() {
  while :; do
    iw dev wlan0 scan ap-force 2>/dev/null | awk '
      /^BSS /  { bss = $2; sub(/\(.*/, "", bss) }
      /SSID: / { if ($2 == "ticker") print "BSS " bss " ticker" }' |
      while read -r line; do echo "$(date -u +%FT%TZ) SCAN $line"; done
    sleep 10
  done
}

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
scan_loop &
scanpid=$!

elapsed=0
while [ "$elapsed" -lt "$HOLD" ]; do
  sleep 10
  elapsed=$((elapsed + 10))
  snap "DURING+${elapsed}"
done

restore
sleep 20; snap AFTER+20
sleep 40; snap AFTER+60
