#!/usr/bin/env bash
# Recover the Dabai DC1 colour sensor after the Orbbec SDK wedges it.
set -uo pipefail
VID=2bc5; PID=0557

find_dev() {
  for d in /sys/bus/usb/devices/*; do
    [ -f "$d/idVendor" ] || continue
    [ "$(cat $d/idVendor)" = "$VID" ] && [ "$(cat $d/idProduct)" = "$PID" ] && { echo "$d"; return; }
  done
}
D=$(find_dev); [ -z "$D" ] && { echo "colour sensor not on the bus"; exit 1; }
BUS=$(cat "$D/busnum"); DEV=$(cat "$D/devnum")
NODE=$(printf "/dev/bus/usb/%03d/%03d" "$BUS" "$DEV")
IFACE=$(basename "$D"):1.0

echo "$IFACE" | sudo tee /sys/bus/usb/drivers/uvcvideo/unbind >/dev/null 2>&1
sudo python3 -c "
import fcntl,sys
fcntl.ioctl(open(sys.argv[1],'wb'), (ord('U')<<8)|20, 0)" "$NODE" 2>/dev/null
sleep 1
echo "$IFACE" | sudo tee /sys/bus/usb/drivers/uvcvideo/bind >/dev/null 2>&1
sudo udevadm trigger --subsystem-match=video4linux --action=add
sleep 1
LINK=$(ls /dev/v4l/by-id/*Dabai*index0 2>/dev/null | head -1)

if [ -z "$LINK" ]; then
  echo 0 | sudo tee "$D/authorized" >/dev/null 2>&1
  sleep 1
  echo 1 | sudo tee "$D/authorized" >/dev/null 2>&1
  sleep 2
  sudo udevadm trigger --subsystem-match=video4linux --action=add
  sleep 1
  LINK=$(ls /dev/v4l/by-id/*Dabai*index0 2>/dev/null | head -1)
fi

if [ -z "$LINK" ] && command -v uhubctl >/dev/null 2>&1; then
  PARENT=$(basename "$D")
  HUBLOC="${PARENT%.*}"
  PORT="${PARENT##*.}"
  sudo uhubctl -l "$HUBLOC" -p "$PORT" -a cycle >/dev/null 2>&1
  sleep 2
  sudo udevadm trigger --subsystem-match=video4linux --action=add
  sleep 1
  LINK=$(ls /dev/v4l/by-id/*Dabai*index0 2>/dev/null | head -1)
fi

[ -z "$LINK" ] && { echo "FAILED - no capture node"; exit 1; }
python3 -c "
import cv2,sys
c=cv2.VideoCapture(sys.argv[1])
ok=all(c.read()[0] for _ in range(5)); c.release()
print('colour camera OK' if ok else 'FAILED - opens but cannot read'); sys.exit(0 if ok else 1)" "$LINK"
