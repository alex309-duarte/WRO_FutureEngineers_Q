import os
import time
import readchar

SESSION_NAME = "spike_interactive_session"
SPIKE_INIT_DELAY = 1.0 # Seconds to wait after sending a line of MicroPython code

def send_command_to_spike(command):
    """Envía un comando MicroPython individual a la sesión screen del Spike Prime."""
    if not check_screen_session_exists(SESSION_NAME):
        print(f"Error en send_command_to_spike: Sesión '{SESSION_NAME}' no encontrada ANTES de enviar '{command}'.")
        return False

    if not command.endswith('\r'):
        command += '\r'
    
    escaped_command = command.replace('\\', '\\\\').replace('"', '\\"').replace("'", "'\\''")
    screen_cmd_to_execute = f'screen -S {SESSION_NAME} -X stuff "{escaped_command}"'
    
    exit_code = os.system(screen_cmd_to_execute)
    
    if exit_code != 0:
        print(f"Error al ejecutar screen command: '{screen_cmd_to_execute}'. Código de salida: {exit_code}")
        if not check_screen_session_exists(SESSION_NAME):
            print(f"Error en send_command_to_spike: Sesión '{SESSION_NAME}' desapareció DESPUÉS del intento fallido de enviar '{command}'.")
        return False
    
    time.sleep(0.1) # Pausa breve después de cada comando individual
    return True

def send_micropython_block(code_block_string):
    """Envía un bloque de código MicroPython multilínea al Spike."""
    print(f"Sending MicroPython block to Spike ({len(code_block_string.splitlines())} lines)...")
    lines = code_block_string.strip().split('\n')
    for i, line in enumerate(lines):
        line = line.rstrip() # Eliminar espacios en blanco finales, pero no iniciales (para indentación)
        if not line: # Omitir líneas vacías que podrían salir del modo de indentación del REPL
            continue
        print(f"  Sending line {i+1}/{len(lines)}: {line}")
        if not send_command_to_spike(line):
            print(f"  Failed to send line: {line}. Aborting block send.")
            return False
        time.sleep(SPIKE_INIT_DELAY) # Esperar después de cada línea del bloque
    print("MicroPython block sent.")
    return True

# Código MicroPython para definir en el Spike
spike_code_definitions_str = r"""
import hub
import utime

print('[Spike] Hub and utime imported.')

motor_b = None
motor_d = None

try:
    motor_b = hub.port.B.motor # Traction: Port B
    motor_d = hub.port.D.motor # Steering: Port D
    print('[Spike] Motors B (Traction) and D (Steering) assigned.')
except Exception as e:
    print(f'[Spike] Error assigning motors: {e}')

def sp_forward(speed=50):
    if motor_b:
        motor_b.run_at_speed(speed)
    # else:
        # print('[Spike] Traction motor_b not available.')

def sp_backward(speed=50):
    if motor_b:
        motor_b.run_at_speed(-speed)
    # else:
        # print('[Spike] Traction motor_b not available.')

def sp_stop_traction():
    if motor_b:
        motor_b.brake()
    # else:
        # print('[Spike] Traction motor_b not available.')

def sp_steer_left(angle_deg=30, speed_percent=30):
    if motor_d:
        motor_d.run_for_degrees(angle_deg, speed_percent)
    # else:
        # print('[Spike] Steering motor_d not available.')

def sp_steer_right(angle_deg=30, speed_percent=30):
    if motor_d:
        motor_d.run_for_degrees(-angle_deg, speed_percent)
    # else:
        # print('[Spike] Steering motor_d not available.')

def sp_center_steering(speed_percent=30):
    if motor_d:
        motor_d.run_to_position(0, speed=speed_percent)
    # else:
        # print('[Spike] Steering motor_d not available.')

def sp_brake_all():
    if motor_b: motor_b.brake()
    if motor_d: motor_d.brake()
    # print('[Spike] All motors braked.')

print('[Spike] Functions defined.')
print('[Spike] Initializing: Centering steering and braking traction.')
if motor_d: sp_center_steering()
if motor_b: sp_stop_traction()
print('[Spike] Initialization complete. Ready for commands.')
"""

def initialize_spike_environment():
    """Define el entorno y las funciones en el Spike Prime."""
    print("Initializing Spike Prime environment by defining functions...")
    if not send_micropython_block(spike_code_definitions_str):
        print("Critical error: Failed to send MicroPython definitions to Spike. Aborting.")
        return False
    print("Spike Prime environment initialization attempt complete.")
    return True

def main_control_loop():
    """Bucle principal para leer teclas y enviar comandos a funciones del Spike."""
    print("\nAckermann Spike Controller (Screen+Functions)")
    print("---------------------------------------------")
    print("w: Forward | s: Backward | x: Stop Traction")
    print("a: Left    | d: Right    | c: Center Steer")
    print("b: Brake All | q: Quit")
    print("---------------------------------------------")
    print("Press keys to control Spike Prime...")

    speed_traction = 50
    steer_angle = 25
    steer_speed = 30

    while True:
        try:
            key = readchar.readkey()

            if key == 'w':
                print(f"Forward (speed {speed_traction}%)")
                send_command_to_spike(f"sp_forward({speed_traction})")
            elif key == 's':
                print(f"Backward (speed {speed_traction}%)")
                send_command_to_spike(f"sp_backward({speed_traction})")
            elif key == 'x':
                print("Stop Traction")
                send_command_to_spike("sp_stop_traction()")
            elif key == 'a':
                print(f"Steer Left (angle {steer_angle} deg, speed {steer_speed}%)")
                send_command_to_spike(f"sp_steer_left({steer_angle}, {steer_speed})")
            elif key == 'd':
                print(f"Steer Right (angle {steer_angle} deg, speed {steer_speed}%)")
                send_command_to_spike(f"sp_steer_right({steer_angle}, {steer_speed})")
            elif key == 'c':
                print(f"Center Steering (speed {steer_speed}%)")
                send_command_to_spike(f"sp_center_steering({steer_speed})")
            elif key == 'b':
                print("Brake All Motors")
                send_command_to_spike("sp_brake_all()")
            elif key == 'q':
                print("Quitting...")
                send_command_to_spike("sp_brake_all()")
                send_command_to_spike("print('[RPi] Exiting control loop, all motors braked.')")
                break
            # else:
                # print(f"Unassigned key: {key}")
        except KeyboardInterrupt:
            print("\nKeyboardInterrupt caught. Quitting...")
            send_command_to_spike("sp_brake_all()")
            send_command_to_spike("print('[RPi] KeyboardInterrupt, all motors braked.')")
            break

def check_screen_session_exists(session_name):
    """Verifica si la sesión de screen especificada existe para el usuario actual."""
    try:
        result = os.popen(f'screen -list').read()
        if f".{session_name}" in result and "(Detached)" in result:
            return True
        elif f".{session_name}" in result and "(Attached)" in result:
            print(f"Warning: Screen session '{session_name}' found but is attached.")
            return True 
        else:
            # print(f"Screen session '{session_name}' not found or not in expected state.")
            # print(f"Full output of 'screen -list':\n{result}") # Can be verbose
            return False
    except Exception as e:
        print(f"Error checking screen session: {e}")
        return False

if __name__ == "__main__":
    print(f"Attempting to use screen session: {SESSION_NAME}")
    # input(f"Ensure 'auto_spike_control.sh' has started screen session '{SESSION_NAME}' and Spike is in REPL. Press Enter to continue...")
    print(f"Ensure 'auto_spike_control.sh' has started screen session '{SESSION_NAME}' and Spike is in REPL.")
    print("Will proceed after a short delay...")
    time.sleep(3) # Give user time to read / switch if auto_spike_control.sh was just started

    if not check_screen_session_exists(SESSION_NAME):
        print(f"Exiting: Screen session '{SESSION_NAME}' not found by rpi_spike_controller.py.")
        print("Please ensure 'auto_spike_control.sh' is running and has successfully started the screen session.")
        exit(1)
    
    print("Screen session confirmed. Proceeding with Spike environment initialization...")
    if initialize_spike_environment():
        print("Spike environment setup complete. Starting main control loop.")
        main_control_loop()
    else:
        print("Failed to initialize Spike environment. Exiting.")
        send_command_to_spike("print('[RPi] Failed to initialize Spike environment from RPi.')")

    print("Exited Ackermann Spike Controller.")
