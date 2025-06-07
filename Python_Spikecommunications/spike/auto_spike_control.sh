#!/bin/bash

SESSION_NAME="spike_interactive_session" # Usado por rpi_spike_controller.py
SERIAL_PORT="/dev/ttyACM0"
BAUD_RATE="115200"
LOG_FILE="/home/maker/spike/spike_interactive.log"

# Determine the target user for screen commands
if [ -n "$SUDO_USER" ]; then
    TARGET_USER="$SUDO_USER"
elif [ -f "/home/maker/.bashrc" ]; then # Heuristic for 'maker' user
    TARGET_USER="maker"
else
    TARGET_USER=$(logname 2>/dev/null || whoami)
fi
echo "Screen session will be run as user: $TARGET_USER"
TARGET_UID=$(id -u "$TARGET_USER")

cleanup_executed=0

cleanup() {
    if [ "$cleanup_executed" -eq 1 ]; then
        return
    fi
    cleanup_executed=1

    echo -e "\nCleaning up screen session '$SESSION_NAME' (as user $TARGET_USER)..."
    if sudo -u "$TARGET_USER" screen -list | grep -q "$SESSION_NAME"; then
        sudo -u "$TARGET_USER" screen -S "$SESSION_NAME" -X stuff "\003"
        sleep 0.2
        echo "Closing screen session '$SESSION_NAME'..."
        sudo -u "$TARGET_USER" screen -S "$SESSION_NAME" -X quit
        sleep 0.5
    fi
    sudo -u "$TARGET_USER" screen -wipe "$SESSION_NAME" &>/dev/null 
    pkill -f "SCREEN.*$SESSION_NAME.*$TARGET_USER" &>/dev/null

    # Re-enable ModemManager if it was stopped by this script
    if [ -f /tmp/modemmanager_was_stopped_by_spike_script ]; then
        echo "Re-enabling ModemManager service..."
        sudo systemctl start ModemManager.service
        sudo systemctl enable ModemManager.service # Or use 'unmask' if it was masked
        rm -f /tmp/modemmanager_was_stopped_by_spike_script
    fi

    stty sane
    echo "Cleanup complete. Log file: $LOG_FILE (should be owned by $TARGET_USER)"
}

trap 'cleanup; exit 0' EXIT
trap 'echo -e "\nCtrl+C detected by shell script. Cleaning up..."; cleanup; exit 1' SIGINT
trap 'cleanup; exit 1' SIGTERM

# Attempt to stop ModemManager if it's running
if systemctl is-active --quiet ModemManager.service; then
    echo "ModemManager is active. Attempting to stop it temporarily..."
    sudo systemctl stop ModemManager.service
    # sudo systemctl disable ModemManager.service # Use with caution, this persists across reboots
    # Instead of disable, we'll just stop it and re-enable on cleanup
    touch /tmp/modemmanager_was_stopped_by_spike_script
    sleep 1 # Give it a moment to release resources
fi

echo "Stopping any existing '$SESSION_NAME' screen session (as user $TARGET_USER)..."
sudo -u "$TARGET_USER" screen -S "$SESSION_NAME" -X quit >/dev/null 2>&1
sleep 0.5
sudo -u "$TARGET_USER" screen -wipe "$SESSION_NAME" &>/dev/null

if [ ! -e "$SERIAL_PORT" ]; then
    echo "Error: Serial port $SERIAL_PORT not found. Is Spike Prime connected?"
    exit 1
fi

echo "Setting permissions for $SERIAL_PORT..."
sudo chmod a+rw "$SERIAL_PORT"

if ! command -v screen &> /dev/null; then
    echo "Screen not found. Installing screen..."
    sudo apt-get update && sudo apt-get install -y screen
    if ! command -v screen &> /dev/null; then
        echo "Failed to install screen. Please install it manually."
        exit 1
    fi
fi

# Ensure screen socket directory exists and has correct permissions for TARGET_USER
SCREEN_SOCKET_DIR="/run/screen/S-$TARGET_USER"
if [ ! -d "$SCREEN_SOCKET_DIR" ]; then
    echo "Creating screen socket directory: $SCREEN_SOCKET_DIR"
    sudo mkdir -p "$SCREEN_SOCKET_DIR"
    sudo chown "$TARGET_USER:$TARGET_USER" "$SCREEN_SOCKET_DIR"
    sudo chmod 700 "$SCREEN_SOCKET_DIR"
else
    # Ensure permissions are correct even if it exists
    sudo chown "$TARGET_USER:$TARGET_USER" "$SCREEN_SOCKET_DIR"
    sudo chmod 700 "$SCREEN_SOCKET_DIR"
fi

rm -f "$LOG_FILE"
mkdir -p "$(dirname "$LOG_FILE")"
# Log file will be created by screen -L as TARGET_USER

echo "Starting new screen session '$SESSION_NAME' as user '$TARGET_USER' in detached mode."
echo "Logging to '$LOG_FILE'..."
sudo -u "$TARGET_USER" screen -L -Logfile "$LOG_FILE" -dmS "$SESSION_NAME" "$SERIAL_PORT" "$BAUD_RATE"
sleep 3 # Slightly more wait

echo "Verifying screen session '$SESSION_NAME' for user '$TARGET_USER'..."
if sudo -u "$TARGET_USER" screen -list | grep -q "\.$SESSION_NAME\s"; then # More precise grep
    echo "Screen session '$SESSION_NAME' confirmed active for user '$TARGET_USER'."
    sudo -u "$TARGET_USER" screen -list # Display all sessions for the user for verbosity
else
    echo "Error: Screen session '$SESSION_NAME' NOT FOUND for user '$TARGET_USER' after attempt to start."
    echo "Output of 'sudo -u $TARGET_USER screen -list':"
    sudo -u "$TARGET_USER" screen -list
    echo "- Verify $SERIAL_PORT is not held by another process (e.g., ModemManager if stop failed)."
    echo "- Check dmesg for serial port errors."
    exit 1
fi

echo "Ensuring Spike Prime is in REPL mode (Ctrl+C, then Ctrl+D to session as $TARGET_USER)..."
sudo -u "$TARGET_USER" screen -S "$SESSION_NAME" -X stuff "\003"
sleep 1.5 # Increased sleep
sudo -u "$TARGET_USER" screen -S "$SESSION_NAME" -X stuff "\004"
sleep 3   # Increased sleep for reboot to complete

echo ""
echo "======================================================================"
echo "Screen session '$SESSION_NAME' (as user $TARGET_USER) is running in detached mode."
echo "Spike Prime should be in MicroPython REPL (>>>)."
echo "Log file: $LOG_FILE (owned by $TARGET_USER)"
echo ""
echo "You can now run '/home/maker/spike/rpi_spike_controller.py' (as user $TARGET_USER) in another terminal."
echo "Or, to attach manually for debugging: sudo -u $TARGET_USER screen -r $SESSION_NAME"
echo ""
echo "This script (auto_spike_control.sh) will keep running."
echo "Press Ctrl+C in THIS terminal to stop and cleanup."
echo "======================================================================"
echo ""

while true; do
    sleep 60
    if ! sudo -u "$TARGET_USER" screen -list | grep -q "$SESSION_NAME"; then
        echo "Screen session '$SESSION_NAME' (as user $TARGET_USER) appears to have died. Exiting..."
        break
    fi
done

