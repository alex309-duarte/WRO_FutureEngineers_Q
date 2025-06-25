"""
This module provides support for serial communication in Python.
"""
import serial
SERIAL_TIMEOUT = 1 # seconds
spike = serial.Serial('/dev/ttyACM0', 115200, timeout=SERIAL_TIMEOUT)

def _consume_spike_prompt(operation_name="unknown_operation"):
    """Reads from Spike until '>>>' is found or a few timeouts occur."""
    prompt_data = b''
    # print(f"DEBUG (_consume_spike_prompt for {operation_name}): Attempting to read prompt...")
    for i in range(4): # Try up to 4 reads (increased from 3)
        try:
            chunk = spike.readline()
            if not chunk: # readline timed out and returned empty
                # print(f"DEBUG (_consume_spike_prompt for {operation_name}): Read attempt {i+1} timed out.")
                if b'>>>' in prompt_data: # If we already got it, timeout is fine
                    break
                if i > 0 and prompt_data: # If we have some data but not prompt, and it's not the first try
                    pass # Allow loop to continue if we have partial data
                elif i > 1: # If multiple timeouts with no data, break
                    break
                continue 
            
            # print(f"DEBUG (_consume_spike_prompt for {operation_name}): Read chunk: {repr(chunk)}")
            prompt_data += chunk
            if b'>>>' in prompt_data:
                # print(f"DEBUG (_consume_spike_prompt for {operation_name}): Found '>>>' in accumulated data.")
                break
        except Exception as e:
            print(f"DEBUG (_consume_spike_prompt for {operation_name}): Error during readline: {e}")
            break
    
    # Final check and log
    if b'>>>' not in prompt_data:
        print(f"DEBUG (_consume_spike_prompt for {operation_name}): FAILED to find '>>>'. Accumulated: {repr(prompt_data)}")
    else:
        print(f"DEBUG (_consume_spike_prompt for {operation_name}): Consumed prompt. Data: {repr(prompt_data)}")
    return prompt_data

#variables globales rasp
der = -1
izq = 1

def initialize_Libraries():
    print("DEBUG: >> Entering initialize_Libraries")
    # Helper to send command and log response within initialize_Libraries
    def _send_cmd_init(cmd_str, note=""):
        if not cmd_str.endswith('\r'):
            cmd_str += '\r'
        print(f"DEBUG (initialize_Libraries): Sending: {cmd_str.strip()} {note}")
        spike.write(cmd_str.encode())
        response_bytes = spike.readline()
        print(f"DEBUG (initialize_Libraries): Received: {repr(response_bytes)}")

    _send_cmd_init("import motor") 
    _send_cmd_init("from hub import port")
    _send_cmd_init("from hub import motion_sensor")
    _send_cmd_init("import distance_sensor")
    #declare varianbles globales spike
    _send_cmd_init("der = -1")
    _send_cmd_init("izq = 1")
    _send_cmd_init("error = 0")
    #declare functions for motors
    _send_cmd_init("def fr():", "(Start func def)")
    _send_cmd_init("motor.stop(port.F, stop = motor.HOLD)")
    _send_cmd_init("motor.stop(port.B, stop = motor.HOLD)")
    _send_cmd_init("distancias = [distance_sensor.distance(port.C), distance_sensor.distance(port.A), distance_sensor.distance(port.E)]")
    print("DEBUG (initialize_Libraries): Calling end_Function for fr")
    end_Function()

    _send_cmd_init("def fc():", "(Start func def)")
    _send_cmd_init("motor.stop(port.F, stop = motor.COAST)")
    _send_cmd_init("motor.stop(port.B, stop = motor.COAST)")
    print("DEBUG (initialize_Libraries): Calling end_Function for fc")
    end_Function()
    # centrar el vehiculo 
    _send_cmd_init("def cv():", "(Start func def)")
    _send_cmd_init("motor.run_to_absolute_position(port.F, 0, 550,")
    _send_cmd_init("direction = motor.SHORTEST_PATH, stop = motor.HOLD, acceleration = 1000, deceleration = 1000)")
    _send_cmd_init("return 255")
    print("DEBUG (initialize_Libraries): Calling end_Function for cv")
    end_Function()

    #pd 
    _send_cmd_init("def pd(s1,s2,vel,kp,kd,ea):", "(Start func def)") #ea es error anterior
    _send_cmd_init("error=s1-s2")
    _send_cmd_init("et= (kp*error) + (kd*(error-ea))") #et es error total
    _send_cmd_init("motor.run_to_absolute_position(port.F, int(et), 550, direction = motor.SHORTEST_PATH)")
    _send_cmd_init("motor.set_duty_cycle(port.B, -vel*100)")
    _send_cmd_init("return error")
    print("DEBUG (initialize_Libraries): Calling end_Function for pd")
    end_Function()

    #reset Gyro
    _send_cmd_init("def rg(grados):", "(Start func def)")
    _send_cmd_init("motion_sensor.reset_yaw(grados)")
    print("DEBUG (initialize_Libraries): Calling end_Function for rg")
    end_Function()

    #avanzar derecho
    _send_cmd_init("def ad(vel,distancia,referencia):", "(Start func def)")
    _send_cmd_init("error = 0")
    _send_cmd_init("while 0 > distance_sensor.distance(port.C) or distance_sensor.distance(port.C) > distancia:")
    _send_cmd_init("error = pd(referencia,motion_sensor.tilt_angles()[0],vel,0.3,1,error)")
    _send_cmd_init("return 255")
    print("DEBUG (initialize_Libraries): Calling end_Function for ad")
    end_Function()

    _send_cmd_init("def vuelta(direccion,velocidad,grados):", "(Start func def)")
    _send_cmd_init("motor.run_to_absolute_position(port.F, 49*(direccion), 550, direction = motor.SHORTEST_PATH)")
    _send_cmd_init("while abs(grados) > abs(motion_sensor.tilt_angles()[0]):")
    _send_cmd_init("motor.set_duty_cycle(port.B, -velocidad*100)")
    _send_cmd_init("return 255")
    print("DEBUG (initialize_Libraries): Calling end_Function for vuelta")
    end_Function()

    _send_cmd_init("def da(vel):", "(Start func def)")
    _send_cmd_init("ulz = distance_sensor.distance(port.A)")
    _send_cmd_init("uld = distance_sensor.distance(port.E)")
    _send_cmd_init("error = 0")
    _send_cmd_init("while ulz > 0 and uld > 0:")
    _send_cmd_init("error = pd(0,motion_sensor.tilt_angles()[0],vel,0.3,1,error)")
    _send_cmd_init("ulz = distance_sensor.distance(port.A)")
    _send_cmd_init("uld = distance_sensor.distance(port.E)")
    _send_cmd_init("fc()")
    _send_cmd_init("return 255")
    print("DEBUG (initialize_Libraries): Calling end_Function for da")
    end_Function()

    _send_cmd_init("def va(vel, grados):", "(Start func def)")
    _send_cmd_init("ulz = distance_sensor.distance(port.A)")
    _send_cmd_init("uld = distance_sensor.distance(port.E)")
    _send_cmd_init("der = -1")
    _send_cmd_init("izq = 1")
    _send_cmd_init("if ulz == -1:")
    _send_cmd_init("vuelta(izq,vel,grados)")
    _send_cmd_init("if uld == -1:")
    _send_cmd_init("vuelta(der,vel,grados)")
    _send_cmd_init("fc()")
    _send_cmd_init("return 255")
    print("DEBUG (initialize_Libraries): Calling end_Function for va")
    end_Function()
    print("DEBUG: << Exiting initialize_Libraries")

def center_vehicle():
    cmd = "cv()\r"
    print(f"DEBUG (center_vehicle): Sending: {cmd.strip()}")
    spike.write(cmd.encode())
    echo_response = spike.readline()
    print(f"DEBUG (center_vehicle): Echo response: {repr(echo_response)}")
    _consume_spike_prompt("center_vehicle")

def Free_spikeDirection():
    cmd = "fr()\r"
    print(f"DEBUG (Free_spikeDirection): Sending: {cmd.strip()}")
    spike.write(cmd.encode())
    echo_response = spike.readline()
    print(f"DEBUG (Free_spikeDirection): Echo response: {repr(echo_response)}")
    _consume_spike_prompt("Free_spikeDirection (fr)")

def Coast_motors():
    cmd = "fc()\r" # Assuming fc() coasts all motors
    print(f"DEBUG (Coast_motors): Sending: {cmd.strip()}")
    spike.write(cmd.encode())
    echo_response = spike.readline()
    print(f"DEBUG (Coast_motors): Echo response: {repr(echo_response)}")
    _consume_spike_prompt("Coast_motors (fc)")


def end_Function():
    print("DEBUG (end_Function): Sending 1st CR to finalize function block.")
    spike.write(b"\r") # Attempt to complete the multi-line block
    r1 = spike.readline()
    print(f"DEBUG (end_Function): Response after 1st CR: {repr(r1)}")

    print("DEBUG (end_Function): Sending Ctrl-C to interrupt and get fresh prompt.")
    spike.write(b"\x03") # Send Ctrl-C
    ctrl_c_response = spike.readline()
    print(f"DEBUG (end_Function): Response after Ctrl-C: {repr(ctrl_c_response)}")

    # It's possible Ctrl-C itself prints a new prompt, or we need one more CR.
    # Let's send a CR and read a few lines to be sure we get to a clean prompt.
    print("DEBUG (end_Function): Sending final CR to solicit prompt.")
    spike.write(b"\r")
    for i in range(3):
        final_response = spike.readline()
        print(f"DEBUG (end_Function): Post Ctrl-C read attempt {i+1}: {repr(final_response)}")
        if b">>>" in final_response:
            print("DEBUG (end_Function): Successfully found '>>>' prompt.")
            return
    print("DEBUG (end_Function): Did NOT find '>>>' prompt after Ctrl-C and CR.")
    
def reset_gyro(grados):
    cmd = f"rg({grados})\r"
    print(f"DEBUG (reset_gyro): Sending: {cmd.strip()}")
    spike.write(cmd.encode())
    echo_response = spike.readline()
    print(f"DEBUG (reset_gyro): Echo response: {repr(echo_response)}")
    _consume_spike_prompt(f"reset_gyro({grados})")
    
def avanzar_distancia(vel,distancia,referencia):
    cmd = f"ad({vel},{distancia},{referencia})\r"
    print(f"DEBUG (avanzar_distancia): Sending: {cmd.strip()}")
    spike.write(cmd.encode())
    echo_response = spike.readline()
    print(f"DEBUG (avanzar_distancia): Echo response: {repr(echo_response)}")
    _consume_spike_prompt(f"avanzar_distancia({vel},{distancia},{referencia})")

    return_value_str = "0"
    max_reads = 15 # Increased max_reads
    print(f"DEBUG (avanzar_distancia): Entering loop to get 255 (max {max_reads} reads)")
    for i in range(max_reads):
        print(f"DEBUG (avanzar_distancia): Loop iteration {i+1}/{max_reads}, reading for return value...")
        raw_return = spike.readline()
        return_value_str = raw_return.decode().strip()
        print(f"DEBUG (avanzar_distancia): Raw: {repr(raw_return)}, Decoded: '{return_value_str}'")
        if not return_value_str: # Handle empty strings that might come from timeouts
            print("DEBUG (avanzar_distancia): Empty response, continuing...")
            continue
        if return_value_str.isdigit() and int(return_value_str) == 255:
            print("DEBUG (avanzar_distancia): Successfully received 255!")
            break
        # If it's not 255, it might be a prompt '>>>' or other unexpected output
        print(f"DEBUG (avanzar_distancia): Did not receive 255, got '{return_value_str}'. Continuing loop...")
    else:
        print(f"DEBUG (avanzar_distancia): Loop finished after {max_reads} reads. Last value: '{return_value_str}'")
    
    print("Fin del recorrido")
    Coast_motors()

def vuelta_grados(direccion,velocidad,grados):
    cmd = f"vuelta({direccion},{velocidad},{grados})\r"
    print(f"DEBUG (vuelta_grados): Sending: {cmd.strip()}")
    spike.write(cmd.encode())
    echo_response = spike.readline()
    print(f"DEBUG (vuelta_grados): Echo response: {repr(echo_response)}")
    _consume_spike_prompt(f"vuelta_grados({direccion},{velocidad},{grados})")

    return_value_str = "0"
    max_reads = 15
    print(f"DEBUG (vuelta_grados): Entering loop to get 255 (max {max_reads} reads)")
    for i in range(max_reads):
        print(f"DEBUG (vuelta_grados): Loop iteration {i+1}/{max_reads}, reading for return value...")
        raw_return = spike.readline()
        return_value_str = raw_return.decode().strip()
        print(f"DEBUG (vuelta_grados): Raw: {repr(raw_return)}, Decoded: '{return_value_str}'")
        if not return_value_str:
            print("DEBUG (vuelta_grados): Empty response, continuing...")
            continue
        if return_value_str.isdigit() and int(return_value_str) == 255:
            print("DEBUG (vuelta_grados): Successfully received 255!")
            break
        print(f"DEBUG (vuelta_grados): Did not receive 255, got '{return_value_str}'. Continuing loop...")
    else:
        print(f"DEBUG (vuelta_grados): Loop finished after {max_reads} reads. Last value: '{return_value_str}'")

    print("Fin de la vuelta")
    Coast_motors()

def avanzar_detection(vel):
    cmd = f"da({vel})\r"
    print(f"DEBUG (avanzar_detection): Sending: {cmd.strip()}")
    spike.write(cmd.encode())
    echo_response = spike.readline()
    print(f"DEBUG (avanzar_detection): Echo response: {repr(echo_response)}")
    _consume_spike_prompt(f"avanzar_detection({vel})")

    return_value_str = "0"
    max_reads = 15
    print(f"DEBUG (avanzar_detection): Entering loop to get 255 (max {max_reads} reads)")
    for i in range(max_reads):
        print(f"DEBUG (avanzar_detection): Loop iteration {i+1}/{max_reads}, reading for return value...")
        raw_return = spike.readline()
        return_value_str = raw_return.decode().strip()
        print(f"DEBUG (avanzar_detection): Raw: {repr(raw_return)}, Decoded: '{return_value_str}'")
        if not return_value_str:
            print("DEBUG (avanzar_detection): Empty response, continuing...")
            continue
        if return_value_str.isdigit() and int(return_value_str) == 255:
            print("DEBUG (avanzar_detection): Successfully received 255!")
            break
        print(f"DEBUG (avanzar_detection): Did not receive 255, got '{return_value_str}'. Continuing loop...")
    else:
        print(f"DEBUG (avanzar_detection): Loop finished after {max_reads} reads. Last value: '{return_value_str}'")

    print("Fin de la deteccion")
    Coast_motors()

def vuelta_automatica(vel,grados):
    cmd = f"va({vel},{grados})\r"
    print(f"DEBUG (vuelta_automatica): Sending: {cmd.strip()}")
    spike.write(cmd.encode())
    echo_response = spike.readline()
    print(f"DEBUG (vuelta_automatica): Echo response: {repr(echo_response)}")
    _consume_spike_prompt(f"vuelta_automatica({vel},{grados})")

    return_value_str = "0"
    max_reads = 15
    print(f"DEBUG (vuelta_automatica): Entering loop to get 255 (max {max_reads} reads)")
    for i in range(max_reads):
        print(f"DEBUG (vuelta_automatica): Loop iteration {i+1}/{max_reads}, reading for return value...")
        raw_return = spike.readline()
        return_value_str = raw_return.decode().strip()
        print(f"DEBUG (vuelta_automatica): Raw: {repr(raw_return)}, Decoded: '{return_value_str}'")
        if not return_value_str:
            print("DEBUG (vuelta_automatica): Empty response, continuing...")
            continue
        if return_value_str.isdigit() and int(return_value_str) == 255:
            print("DEBUG (vuelta_automatica): Successfully received 255!")
            break
        print(f"DEBUG (vuelta_automatica): Did not receive 255, got '{return_value_str}'. Continuing loop...")
    else:
        print(f"DEBUG (vuelta_automatica): Loop finished after {max_reads} reads. Last value: '{return_value_str}'")

    print("Fin de la vuelta")
    Coast_motors()

try:# Main Program
    print("DEBUG: Main program started.")
    initialize_Libraries()
    i = 0
    while i < 8:
        i = i + 1
    #    reset_gyro()
    #    avanzar_distancia(90,780)
    #    vuelta_grados(der,60,90)
    #   Free_spikeDirection()
    #   reset_gyro()
    #time.sleep(0.050)
        reset_gyro(0)
        avanzar_detection(70)
        vuelta_automatica(60,91)
        reset_gyro(0)
        avanzar_distancia(70,780,-90)

    #Coast_motors() #brake all motors
    #spike.close() #close serial connection

except KeyboardInterrupt:
    print("\nDEBUG: KeyboardInterrupt caught!")
    print("DEBUG: Sending Ctrl+C to Spike to interrupt any ongoing MicroPython execution.")
    spike.write(chr(3).encode())
    r1 = spike.readline()
    print(f"DEBUG (KeyboardInterrupt): Cleared buffer 1: {repr(r1)}")
    r2 = spike.readline()
    print(f"DEBUG (KeyboardInterrupt): Cleared buffer 2: {repr(r2)}")
    r3 = spike.readline()
    print(f"DEBUG (KeyboardInterrupt): Cleared buffer 3: {repr(r3)}")
    print("DEBUG (KeyboardInterrupt): Calling Coast_motors().")
    Coast_motors()
    print("DEBUG (KeyboardInterrupt): Cleanup finished.")




