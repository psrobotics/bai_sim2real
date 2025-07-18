#!/bin/bash

# ==============================================================================
#  Persistent CAN Interface Naming Script for canable Devices
# ==============================================================================
#
# This script creates a udev rule to ensure your canable USB-to-CAN adapters
# are always assigned the same interface name (can0, can1), even if they are
# plugged into different USB ports or in a different order.
#
# It identifies the devices by their unique USB serial numbers.
#
# USAGE:
#   1. Save this file as 'setup_can_rules.sh'
#   2. Make it executable: chmod +x setup_can_rules.sh
#   3. Run with sudo:     sudo ./setup_can_rules.sh
#
# ==============================================================================

# --- Check for root privileges ---
if [ "$EUID" -ne 0 ]; then
  echo "Please run this script with sudo or as root."
  exit 1
fi

echo "--- CANable Persistent Naming Setup ---"
echo "This script will help you create a udev rule to lock your two"
echo "canable devices to the names 'can0' and 'can1'."
echo

# --- Function to find all canable serial numbers ---
find_all_canable_serials() {
    # For each network device that could be a CAN interface...
    for iface in /sys/class/net/can*; do
        # Check if the interface exists and is a gs_usb device
        if [[ -e "$iface/device/driver" && $(basename $(readlink -f "$iface/device/driver")) == "gs_usb" ]]; then
            # The parent of the 'device' symlink is the USB device itself.
            # Read its 'serial' file to get the unique hardware serial number.
            local usb_device_path
            usb_device_path=$(dirname "$(readlink -f "$iface/device")")
            if [ -f "$usb_device_path/serial" ]; then
                cat "$usb_device_path/serial"
            fi
        fi
    done | sort | uniq # Sort and get unique serials
}


# --- Step 1: Find the serial numbers ---

echo "--> Unplug both canable devices from your computer."
echo "--> Then, plug in ONLY the device you want to be 'can0' and press Enter."
read -p ""

echo "Waiting 3 seconds for the device to initialize..."
sleep 3

SERIAL_CAN0=$(find_all_canable_serials | head -n 1)

if [ -z "$SERIAL_CAN0" ]; then
    echo
    echo "Error: Could not find a canable device."
    echo "Troubleshooting steps:"
    echo "  1. Make sure the device is securely plugged in."
    echo "  2. Verify the gs_usb driver is loaded with 'lsmod | grep gs_usb'."
    echo "  3. Try a different USB port."
    exit 1
fi

echo "Found device for can0. Serial Number: $SERIAL_CAN0"
echo

echo "--> Now, please plug in the SECOND canable device (the one for 'can1') and press Enter."
read -p ""

echo "Waiting 3 seconds for the second device to initialize..."
sleep 3

# Find all serials and filter out the one we already found
SERIAL_CAN1=$(find_all_canable_serials | grep -v "^$SERIAL_CAN0$" | head -n 1)

if [ -z "$SERIAL_CAN1" ]; then
    echo
    echo "Error: Could not find a second, different canable device."
    echo "Please ensure both devices are plugged in and are distinct."
    exit 1
fi

echo "Found device for can1. Serial Number: $SERIAL_CAN1"
echo

# --- Step 2: Create the udev rule file ---

UDEV_RULE_FILE="/etc/udev/rules.d/99-persistent-can.rules"

echo "Creating udev rule file at $UDEV_RULE_FILE..."

# Write the rules to the file. This will overwrite any existing file.
cat > "$UDEV_RULE_FILE" << EOL
# ===========================================================
#  Persistent rules for canable USB-to-CAN adapters
#  Generated by setup_can_rules.sh
# ===========================================================

# Rule for can0 - Serial: $SERIAL_CAN0
ACTION=="add", SUBSYSTEM=="net", DRIVERS=="gs_usb", ATTRS{serial}=="$SERIAL_CAN0", NAME="can0"

# Rule for can1 - Serial: $SERIAL_CAN1
ACTION=="add", SUBSYSTEM=="net", DRIVERS=="gs_usb", ATTRS{serial}=="$SERIAL_CAN1", NAME="can1"
EOL

echo "Successfully wrote the following rules:"
echo "-------------------------------------------"
cat "$UDEV_RULE_FILE"
echo "-------------------------------------------"
echo

# --- Step 3: Reload udev rules ---

echo "Reloading udev rules to apply changes..."
udevadm control --reload-rules
udevadm trigger

echo
echo "--- Setup Complete! ---"
echo "The udev rules have been created and loaded."
echo "To test, unplug and replug both of your canable devices."
echo "Then, run 'ip link show' to verify that 'can0' and 'can1' have been created correctly."
echo "You may need to reboot for the changes to take full effect in all cases."

exit 0
