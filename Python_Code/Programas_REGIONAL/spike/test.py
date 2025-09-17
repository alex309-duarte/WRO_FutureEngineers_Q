import os
import time
import subprocess

SCREEN_SESSION_NAME = "spike_interactive_session"
# Ensure this log path matches the one in your auto_spike_control.sh or screen startup command
SCREEN_LOG_FILE = "/home/maker/spike/spike_interactive.log" 
COMMAND_DELAY = 0.2  # Delay after sending a command via screen
LINE_SEND_DELAY = 0.1 # Delay between sending lines of a multi-line block

def check_screen_session_exists(session_name):
    """Checks if the specified screen session exists."""
    try:
        # List screen sessions and check if our session_name is in the output
        result = subprocess.run(['screen', '-ls'], capture_output=True, text=True, check=True)
        return f"\t{session_name}\t" in result.stdout
    except (subprocess.CalledProcessError, FileNotFoundError):
        # CalledProcessError if screen -ls returns non-zero (e.g., no sessions)
        # FileNotFoundError if screen is not installed
        return False

def send_command_to_spike_via_screen(command, session_name=SCREEN_SESSION_NAME):
    """Sends a command string to the Spike REPL via a screen session."""
    if not check_screen_session_exists(session_name):
        print(f"Error: Screen session '{session_name}' not found. Cannot send command: {command}")
        return False
    
    # Escape special characters for shell and screen's 'stuff'
    # Basic escaping: \, ", '
    # For 'stuff', a literal newline is often not needed if command ends with \r
    # but complex commands might need more careful handling.
    escaped_command = command.replace('\\', '\\\\').replace('"', '\\"').replace("'", "'\\''")
    
    # Ensure command ends with a carriage return for REPL execution
    if not escaped_command.endswith('\\r'):
        screen_cmd_to_send = f'{escaped_command}\\r'
    else:
        screen_cmd_to_send = escaped_command

    # Construct the screen command
    # Using -X stuff injects the string into the screen session
    full_screen_command = f'screen -S {session_name} -X stuff "{screen_cmd_to_send}"'
    
    print(f"Sending to screen '{session_name}': {command.strip()}") # Print the original command for clarity
    exit_code = os.system(full_screen_command)
    
    if exit_code != 0:
        print(f"Error executing screen command: '{full_screen_command}'. Exit code: {exit_code}")
        return False
    time.sleep(COMMAND_DELAY) # Give Spike/screen a moment to process
    return True

def send_multiline_micropython_to_screen(code_block, session_name=SCREEN_SESSION_NAME):
    """Sends a multiline block of MicroPython code to the Spike via screen, line by line."""
    print(f"Sending MicroPython block to screen session '{session_name}':")
    lines = code_block.strip().split('\n')
    for line in lines:
        print(f"  Line: {line.strip()}")
        if not send_command_to_spike_via_screen(line, session_name):
            print(f"Failed to send line: {line.strip()}. Aborting block send.")
            return False
        time.sleep(LINE_SEND_DELAY) # Small delay between lines
    # Send an extra carriage return to ensure the last line/block is processed if it's a definition
    send_command_to_spike_via_screen("", session_name) 
    print("MicroPython block sent.")
    return True

def initialize_spike_motors_via_screen():
    """Initializes motor objects on the Spike by sending MicroPython code via screen.
       Traction=D, Steering=B.
       User must check SCREEN_LOG_FILE for 'MOTOR_INIT_VIA_SCREEN_SUCCESS'.
    """
    print(f"\n--- Initializing Motors on Spike via Screen (Traction=D, Steering=B) ---")
    print(f"IMPORTANT: Check log file '{SCREEN_LOG_FILE}' for 'MOTOR_INIT_VIA_SCREEN_SUCCESS' message.")

    # Send Ctrl+C and Ctrl+D first to clear REPL state
    print("Attempting to send Ctrl+C (interrupt) to Spike REPL via screen...")
    send_command_to_spike_via_screen('\x03', SCREEN_SESSION_NAME) # Ctrl+C
    time.sleep(0.5)
    print("Attempting to send Ctrl+D (soft reboot) to Spike REPL via screen...")
    send_command_to_spike_via_screen('\x04', SCREEN_SESSION_NAME) # Ctrl+D
    time.sleep(2) # Wait for soft reboot to potentially occur
    # Send a couple of empty CRs to ensure prompt is clear if reboot worked
    send_command_to_spike_via_screen("", SCREEN_SESSION_NAME)
    send_command_to_spike_via_screen("", SCREEN_SESSION_NAME)
    print("Soft reboot commands sent. Continuing with motor initialization code.")

    micropython_motor_init_code = """
from hub import port
import utime
motor_traction = None
motor_steering = None
print('Attempting motor init via screen...')
try:
    motor_traction = hub.port.D.motor
    motor_steering = hub.port.B.motor
    print('MOTOR_INIT_VIA_SCREEN_SUCCESS: Traction=D, Steering=B')
except Exception as e:
    print(f'MOTOR_INIT_VIA_SCREEN_FAIL: {{e}}')
"""
    if not send_multiline_micropython_to_screen(micropython_motor_init_code):
        print("Failed to send motor initialization block to Spike.")
        return False
    
    print("Motor initialization block sent. Please check screen log for confirmation.")
    return True

# --- Motor Control Functions (using Spike-side variables via screen) ---
def traction_forward(speed=50):
    send_command_to_spike_via_screen(f"motor_traction.run_at_speed({speed})", SCREEN_SESSION_NAME)

def traction_backward(speed=50):
    send_command_to_spike_via_screen(f"motor_traction.run_at_speed(-{speed})", SCREEN_SESSION_NAME)

def traction_stop():
    send_command_to_spike_via_screen("motor_traction.brake()", SCREEN_SESSION_NAME)

def steer_left(angle=30, speed_pct=30):
    send_command_to_spike_via_screen(f"motor_steering.run_for_degrees({angle}, {speed_pct})", SCREEN_SESSION_NAME)

def steer_right(angle=30, speed_pct=30):
    send_command_to_spike_via_screen(f"motor_steering.run_for_degrees(-{angle}, {speed_pct})", SCREEN_SESSION_NAME)

def steer_center(speed_pct=30):
    send_command_to_spike_via_screen(f"motor_steering.run_to_position(0, speed={speed_pct})", SCREEN_SESSION_NAME)

def brake_all_motors():
    send_command_to_spike_via_screen("if 'motor_traction' in globals() and motor_traction: motor_traction.brake()", SCREEN_SESSION_NAME)
    send_command_to_spike_via_screen("if 'motor_steering' in globals() and motor_steering: motor_steering.brake()", SCREEN_SESSION_NAME)

if __name__ == "__main__":
    print(f"Starting Spike Prime control via test.py (using screen session: {SCREEN_SESSION_NAME})...")
    print(f"Motor Mapping: Traction=Port D, Steering=Port B")
    print(f"Ensure screen session '{SCREEN_SESSION_NAME}' is running and connected to Spike.")
    print(f"Log file for Spike output: {SCREEN_LOG_FILE}")

    if not check_screen_session_exists(SCREEN_SESSION_NAME):
        print(f"Critical: Screen session '{SCREEN_SESSION_NAME}' not found. Please start it first.")
        print("You might need to run: ./auto_spike_control.sh or similar.")
        exit(1)

    print("Screen session found.")
    input(f"Press Enter to initialize motors (via screen) and run test sequence...\nCheck {SCREEN_LOG_FILE} for 'MOTOR_INIT_VIA_SCREEN_SUCCESS'.")

    if not initialize_spike_motors_via_screen():
        print("Motor initialization attempt finished (may have failed). Check logs.")
        # Not exiting here, as we can't be sure of failure without log parsing
    else:
        print("Motor initialization attempt finished. Check logs for success.")

    print("\n--- Running Test Movement Sequence (via screen) ---")
    print(f"Watch your robot and check {SCREEN_LOG_FILE} for command echoes and errors.")
    
    steer_center(speed_pct=40)
    time.sleep(1)

    traction_forward(60)
    time.sleep(1.5)
    traction_stop()
    time.sleep(0.5)

    steer_left(angle=45, speed_pct=50)
    time.sleep(1.5)
    steer_right(angle=45, speed_pct=50)
    time.sleep(1.5)
    steer_center(speed_pct=40)
    time.sleep(1)

    traction_backward(50)
    time.sleep(1.5)
    brake_all_motors()
    time.sleep(0.5)
    print("--- Test Movement Sequence Complete (commands sent via screen) ---")
    print(f"Please verify execution by checking the robot and the log file: {SCREEN_LOG_FILE}")
    print("Script finished.")
