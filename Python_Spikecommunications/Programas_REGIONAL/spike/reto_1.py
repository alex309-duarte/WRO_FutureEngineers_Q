"""
This module provides support for serial communication in Python.
"""
import serial
import time
import sys # Ensure sys is imported for sys.exit if needed later


spike = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

#variables globales rasp
der = -1
izq = 1

def initialize_Libraries():
    print("DEBUG: >> Entering initialize_Libraries")
    # Helper to send command and optionally print debug info
    def send_spike_cmd(cmd, debug_msg="Sending"):
        print(f"DEBUG: {debug_msg}: Sending: {cmd.strip()}")
        spike.write(cmd.encode()) # Send the actual command
        # Aggressively try to clear and get a prompt
        # time.sleep(0.05) # Short pause for command to be processed by Spike
        print(f"DEBUG (send_spike_cmd): Sending Ctrl+C for {cmd.strip()}")
        spike.write(b'\x03') # Ctrl+C
        # print(f"DEBUG (send_spike_cmd): Reading after Ctrl+C for {cmd.strip()}...")
        # ctrl_c_resp = spike.read_until(timeout=0.1) # Read anything for a very short time
        # print(f"DEBUG (send_spike_cmd): Ctrl+C response: {repr(ctrl_c_resp)}")
        
        print(f"DEBUG (send_spike_cmd): Sending CR to get prompt after Ctrl+C for {cmd.strip()}")
        spike.write(b'\r') # CR
        print(f"DEBUG (send_spike_cmd): Reading for prompt for {cmd.strip()}...")
        prompt_bytes = spike.readline()
        print(f"DEBUG (send_spike_cmd): Prompt read for {cmd.strip()}: {repr(prompt_bytes)}")

    send_spike_cmd("import motor\r")
    send_spike_cmd("from hub import port\r")
    send_spike_cmd("from hub import motion_sensor\r")
    send_spike_cmd("import distance_sensor\r")
    
    # Declare global variables on Spike
    send_spike_cmd("der = -1\r")
    send_spike_cmd("izq = 1\r")
    send_spike_cmd("error = 0\r")

    # Define function fr()
    send_spike_cmd("def fr():\r", "Defining fr")
    send_spike_cmd("  motor.stop(port.F, stop = motor.HOLD)\r")
    send_spike_cmd("  motor.stop(port.B, stop = motor.HOLD)\r")
    send_spike_cmd("  distancias = [distance_sensor.distance(port.C), distance_sensor.distance(port.A), distance_sensor.distance(port.E)]\r")
    end_Function() # Completes fr definition

    # Define function fc()
    send_spike_cmd("def fc():\r", "Defining fc")
    send_spike_cmd("  motor.stop(port.F, stop = motor.COAST)\r")
    send_spike_cmd("  motor.stop(port.B, stop = motor.COAST)\r")
    end_Function() # Completes fc definition

    # Define function cv() - Center Vehicle
    send_spike_cmd("def cv():\r", "Defining cv")
    send_spike_cmd("  motor.run_to_absolute_position(port.F, 0, 550, direction = motor.SHORTEST_PATH, stop = motor.HOLD, acceleration = 1000, deceleration = 1000)\r")
    send_spike_cmd("  return 255\r")
    end_Function() # Completes cv definition

    # Define function pd() - Proportional-Derivative Controller
    send_spike_cmd("def pd(s1,s2,vel,kp,kd,ea):\r", "Defining pd")
    send_spike_cmd("  error=s1-s2\r")
    send_spike_cmd("  et= (kp*error) + (kd*(error-ea))\r")
    send_spike_cmd("  motor.run_to_absolute_position(port.F, int(et), 550, direction = motor.SHORTEST_PATH)\r")
    send_spike_cmd("  motor.set_duty_cycle(port.B, (-100)*vel)\r")
    send_spike_cmd("  return error\r")
    end_Function() # Completes pd definition

    # Define function rg() - Reset Gyro
    send_spike_cmd("def rg(grados):\r", "Defining rg")
    send_spike_cmd("  motion_sensor.reset_yaw(grados)\r")
    end_Function() # Completes rg definition

    # Define function ad() - Avanzar Derecho (Advance Straight with distance)
    send_spike_cmd("def ad(vel,distancia,referencia):\r", "Defining ad")
    send_spike_cmd("  error = 0\r")
    send_spike_cmd("  while 0 > distance_sensor.distance(port.C) or distance_sensor.distance(port.C) > distancia:\r")
    send_spike_cmd("    error = pd(referencia,motion_sensor.tilt_angles()[0],vel,0.3,1,error)\r")
    send_spike_cmd("  return 255\r")
    end_Function() # Completes ad definition

    # Define function vuelta() - Turn (degrees)
    send_spike_cmd("def vuelta(direccion,velocidad,grados):\r", "Defining vuelta")
    send_spike_cmd("  motor.run_to_absolute_position(port.F, 49*(direccion), 550, direction = motor.SHORTEST_PATH)\r")
    send_spike_cmd("  while abs(grados*10) > abs(motion_sensor.tilt_angles()[0]):\r")
    send_spike_cmd("    motor.set_duty_cycle(port.B, -100*(velocidad))\r")
    send_spike_cmd("  return 255\r")
    end_Function() # Completes vuelta definition

    # Define function da() - Avanzar con Deteccion (Advance with detection)
    send_spike_cmd("def da(vel):\r", "Defining da")
    send_spike_cmd("  ulz = distance_sensor.distance(port.A)\r")
    send_spike_cmd("  uld = distance_sensor.distance(port.E)\r")
    send_spike_cmd("  error = 0\r")
    send_spike_cmd("  while ulz > 0 and uld > 0:\r")
    send_spike_cmd("    error = pd(0,motion_sensor.tilt_angles()[0],vel,0.3,1,error)\r")
    send_spike_cmd("    ulz = distance_sensor.distance(port.A)\r")
    send_spike_cmd("    uld = distance_sensor.distance(port.E)\r")
    send_spike_cmd("  fc()\r")
    send_spike_cmd("  return 255\r")
    end_Function() # Completes da definition

    # Define function va() - Vuelta Automatica (Automatic Turn)
    send_spike_cmd("def va(vel, grados):\r", "Defining va")
    send_spike_cmd("  ulz = distance_sensor.distance(port.A)\r")
    send_spike_cmd("  uld = distance_sensor.distance(port.E)\r")
    send_spike_cmd("  der = -1\r") # Note: These are already global, re-declaring here might shadow them or be redundant
    send_spike_cmd("  izq = 1\r") # Same as above
    send_spike_cmd("  if ulz == -1:\r")
    send_spike_cmd("    vuelta(izq,vel,grados)\r")
    send_spike_cmd("  if uld == -1:\r")
    send_spike_cmd("    vuelta(der,vel,grados)\r")
    send_spike_cmd("  fc()\r")
    send_spike_cmd("  return 255\r")
    end_Function() # Completes va definition
    print("DEBUG: << Exiting initialize_Libraries") #clear buffer
    spike.write("ulz = distance_sensor.distance(port.A)\r".encode())
    spike.readline() #clear buffer
    spike.write("uld = distance_sensor.distance(port.E)\r".encode())
    spike.readline() #clear buffer
    spike.write("der = -1\r".encode())
    spike.readline() #clear buffer
    spike.write("izq = 1\r".encode())
    spike.readline() #clear buffer
    spike.write("if ulz == -1:\r".encode())
    spike.readline() #clear buffer
    spike.write("vuelta(izq,vel,grados)\r".encode())
    spike.readline() #clear buffer
    spike.write("if uld == -1:\r".encode())
    spike.readline() #clear buffer
    spike.write("vuelta(der,vel,grados)\r".encode())
    spike.readline() #clear buffer
    spike.write("fc()\r".encode())
    spike.readline() #clear buffer
    spike.write("return 255\r".encode())
    spike.readline() #clear buffer
    end_Function()
    end_Function()

def center_vehicle():
    spike.write("cv()\r".encode())
    spike.readline() #clear buffer

def Free_spikeDirection():
    spike.write("fr()\r".encode())
    spike.readline() #clear buffer

def Coast_motors():
    spike.write("fc()\r".encode())
    spike.readline() #clear buffer


def end_Function():
    print("DEBUG (end_Function): Sending 1st CR")
    spike.write(b'\r')
    print("DEBUG (end_Function): Attempting 1st readline")
    line1_bytes = spike.readline()
    print(f"DEBUG (end_Function): 1st readline got: {repr(line1_bytes)}")

    print("DEBUG (end_Function): Sending 2nd CR")
    spike.write(b'\r')
    print("DEBUG (end_Function): Attempting 2nd readline")
    line2_bytes = spike.readline()
    print(f"DEBUG (end_Function): 2nd readline got: {repr(line2_bytes)}")

    print("DEBUG (end_Function): Sending 3rd CR")
    spike.write(b'\r')
    print("DEBUG (end_Function): Attempting 3rd readline")
    line3_bytes = spike.readline()
    print(f"DEBUG (end_Function): 3rd readline got: {repr(line3_bytes)}")
    
def reset_gyro(grados):
    cmd = "rg(" + str(grados) + ")\r"
    print(f"Sending to Spike (reset_gyro): {cmd.strip()}")
    spike.write(cmd.encode())
    # For now, assume echo is consumed by subsequent operations or main loop's readline
    # This function will be revisited once initialize_Libraries is stable.
    response_bytes = spike.readline() 
    print(f"DEBUG (reset_gyro): Read after rg(): {repr(response_bytes)}")
    
def avanzar_distancia(vel,distancia,referencia):
    spike.write(("ad("+str(vel)+","+str(distancia)+","+str(referencia)+")\r").encode())
    spike.readline() #clear buffer
    return_value = spike.readline().decode()
    if return_value == "":
        return_value = "0"
    while int(return_value) != 255:
        return_value = spike.readline().decode()
        if return_value == "":
            return_value = "0"
    print("Fin del recorrido")
    Coast_motors()

def vuelta_grados(direccion,velocidad,grados):
    spike.write(("vuelta("+str(direccion)+","+str(velocidad)+","+str(grados)+")\r").encode())
    spike.readline() #clear buffer
    return_value = spike.readline().decode()
    if return_value == "":
        return_value = "0"
    while int(return_value) != 255:
        return_value = spike.readline().decode()
        if return_value == "":
            return_value = "0"
    print("Fin de la vuelta")
    Coast_motors()

def avanzar_detection(vel):
    cmd = "da(" + str(vel) + ")\r"
    print(f"Sending to Spike: {cmd.strip()}")
    spike.write(cmd.encode())
    spike.readline()  # Clear buffer after sending command (e.g., command echo)

    print("--- Start of Spike Response (avanzar_detection) ---")
    received_lines = []
    # Expecting a value, potentially preceded by other output or traceback
    # Read lines until we get '255', a timeout, or too many lines.
    max_lines_to_read = 15  # Safety break for unexpected long output
    success_code_found = False

    for i in range(max_lines_to_read):
        line_bytes = spike.readline()
        if not line_bytes: # Timeout occurred
            print(f"DEBUG: spike.readline() timed out after {i} lines.")
            break
        
        line_decoded = line_bytes.decode().strip() # Strip whitespace and newline
        print(f"DEBUG: Raw line from Spike: {repr(line_bytes)} -> Decoded/Stripped: {repr(line_decoded)}")
        received_lines.append(line_decoded)

        if line_decoded == "255":
            print("DEBUG: Success code '255' received from Spike.")
            success_code_found = True
            break
        elif "Traceback" in line_decoded:
            print("DEBUG: 'Traceback' detected in Spike output.")
            # Continue reading to capture the rest of the traceback
        elif not line_decoded and i > 0: # Empty line after some data might be end of useful output
            print("DEBUG: Received empty stripped line, assuming end of relevant Spike output.")
            # break # Decide if an empty line should terminate reading

    print("--- End of Spike Response (avanzar_detection) ---")

    if success_code_found:
        print("Fin de la deteccion (avanzar_detection successful).")
    else:
        print("Error or unexpected response in avanzar_detection.")
        print("Full output received from Spike during avanzar_detection call:")
        for idx, l in enumerate(received_lines):
            print(f"  Line {idx}: {l}")
        # Potentially re-raise an exception or handle error

    Coast_motors()

def vuelta_automatica(vel,grados):
    spike.write(("va("+str(vel)+","+str(grados)+")\r").encode())
    spike.readline() #clear buffer
    return_value = spike.readline().decode()
    if return_value == "":
        return_value = "0"
    while int(return_value) != 255:
        return_value = spike.readline().decode()
        if return_value == "":
            return_value = "0"
    print("Fin de la vuelta")
    Coast_motors()

try:# Main Program
    print("DEBUG: Starting main program try block...")
    initialize_Libraries()
    print("DEBUG: initialize_Libraries() completed.")
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
        time.sleep(0.1) # Added delay as in base.py
        avanzar_detection(70)
        vuelta_automatica(60,91)
        reset_gyro(0)
        avanzar_distancia(70,780,-90)

    #Coast_motors() #brake all motors
    #spike.close() #close serial connection

except KeyboardInterrupt:
    print("\nProgram interrupted! Cleaning up...")
    spike.write(chr(3).encode())
    spike.readline() #clear buffer
    spike.readline() #clear buffer
    spike.readline() #clear buffer
    Coast_motors()
    
    
    
    
