#!/usr/bin/env bash
set -euo pipefail

reload_driver=0

usage() {
    cat <<'EOF'
Usage: setup_orin_kcan_names.sh [--reload]

Configures KH/PEAK USB-CAN naming on Jetson/Orin:
  onboard CAN stays can0/can1
  KH USB-CAN channels become kcan1..kcan6

Run after installing KH-UCANFD_Linux_SDK. Use --reload only when it is safe to
temporarily unload the kcan driver; otherwise unplug/replug the USB-CAN board or reboot.
EOF
}

for arg in "$@"; do
    case "${arg}" in
        --reload)
            reload_driver=1
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            usage >&2
            exit 2
            ;;
    esac
done

if [[ -e /etc/udev/rules.d/100-kcan.rules && ! -e /etc/udev/rules.d/100-kcan.rules.disabled ]]; then
    sudo mv /etc/udev/rules.d/100-kcan.rules /etc/udev/rules.d/100-kcan.rules.disabled
fi

echo 'options kcan assign=kunhong' | sudo tee /etc/modprobe.d/99-kcan-netnames.conf >/dev/null

tmp_rules="$(mktemp)"
for i in 1 2 3 4 5 6; do
    n=$((31 + i))
    echo "SUBSYSTEM==\"net\", ACTION==\"add\", DRIVERS==\"kcan\", KERNEL==\"can${n}\", NAME=\"kcan${i}\""
done > "${tmp_rules}"
sudo install -m 0644 "${tmp_rules}" /etc/udev/rules.d/99-kcan-netnames.rules
rm -f "${tmp_rules}"

sudo udevadm control --reload-rules

if (( reload_driver )); then
    for c in $(ip -br link | awk '/^(kcan[0-9]+|can[2-9][0-9]*)/ {print $1}'); do
        sudo ip link set "${c}" down 2>/dev/null || true
    done
    sudo modprobe -r kcan
    sudo modprobe kcan
fi

echo "kcan naming config installed."
if [[ -r /sys/module/kcan/parameters/assign ]]; then
    echo -n "kcan assign parameter: "
    cat /sys/module/kcan/parameters/assign
fi

echo "Current CAN links:"
ip -br link | grep -E 'can|kcan' || true

if command -v lskcan >/dev/null 2>&1; then
    echo "Current KH devices:"
    lskcan || true
fi

if (( ! reload_driver )); then
    echo "If names have not changed yet, unplug/replug the USB-CAN board or run this script with --reload."
fi
