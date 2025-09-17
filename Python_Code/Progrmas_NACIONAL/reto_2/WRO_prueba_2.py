#!/usr/bin/env python3
#se importan todas las librerias necesarias
import time, queue, serial, subprocess, os, pty, threading, lgpio, board, adafruit_vl53l1x, cv2, hailo
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
from dataclasses import dataclass
from math import floor
from rplidar import RPLidar, RPLidarException
from pathlib import Path
import numpy as np

from hailo_apps.hailo_app_python.core.common.buffer_utils import get_caps_from_pad, get_numpy_from_buffer
from hailo_apps.hailo_app_python.core.gstreamer.gstreamer_app import app_callback_class
from hailo_apps.hailo_app_python.apps.detection.detection_pipeline import GStreamerDetectionApp

#se define la clase para el hilo del lidar
@dataclass
class RPlidar_data:     
    dist_front: int         
    dist_left: int  
    dist_75: int 
    dist_315: int
    dist_right: int 
    area: float  

@dataclass
class Distance_data:
    distance_izq: int
    distance_derecho: int
    distance_trasero: int

#se define la clase para el hilo del sensor tof
@dataclass
class Sensor_tof_data:
    identifier : adafruit_vl53l1x.VL53L1X #adafruit_vl53l0x.VL53L0X
    name : str
    slave_address : int
    pin_gpio_enable :int   

#Clase para la camara
@dataclass
class Camera_data:
    label : str
    punto_medio_x: int
    punto_medio_y: int
    area: int
    time: float


class user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        self.new_variable = 42  # New variable example

    def new_function(self):  # New function example
        return "The meaning of life is: "

#se guardan los sensores en una lista
SENSORS = [
    Sensor_tof_data(None,"izquierda", slave_address=0x26, pin_gpio_enable =16),
    Sensor_tof_data(None,"derecho", slave_address=0x28, pin_gpio_enable =20),
    Sensor_tof_data(None,"trasero", slave_address=0x29, pin_gpio_enable =21)
]


running = True
vision_queue: "queue.Queue[RPlidar_data]" = queue.Queue(maxsize=1) #se define la cola para el hilo del lidar
distance_queue: "queue.Queue[Distance_data]" = queue.Queue(maxsize=1) #se define la cola para el hilo del sensor tof
camera_queue: "queue.Queue[Camera_data]" = queue.Queue(maxsize=1) #se define la cola para el hilo de la camara

#se crea el lidar y se define su puerto
PORT_NAME = "/dev/ttyUSB0"
lidar = RPLidar(PORT_NAME)

#se crea el sensor tof y se define su puerto
i2c = board.I2C() #i2c = busio.I2C(board.SCL, board.SDA)

#se define la funcion que inicia el screen
def screenSimplified():
    if not os.path.exists("/dev/ttyACM0"):
        return

    master_fd, slave_fd = pty.openpty() 
    
    subprocess.Popen( 
        ["screen", "/dev/ttyACM0", "115200"],
        stdin=slave_fd, 
        stdout=slave_fd,
        stderr=slave_fd,
    )

    os.close(slave_fd)
    time.sleep(1.0)
    os.write(master_fd, b'\x03') # Send Ctrl+C to screen
    time.sleep(0.2)
    os.write(master_fd, b'\x01') # Send Ctrl+A to screen
    time.sleep(0.2)
    os.write(master_fd, b'k') # Send 'k' to kill the screen session
    time.sleep(0.2)
    os.write(master_fd, b'y') # Send 'y' to confirm kill
    time.sleep(0.2)
    
    os.close(master_fd)

#se corre el screen para el spike

    
#se define la variable para el spike
spike = None
#variables globales rasp
der = 1
izq = -1

#Todas las funciones que se mandan por serial para el spike
def initialize_Libraries():
    spike.write("import motor\r".encode()) 
    spike.readline()
    spike.write("from hub import port\r".encode())
    spike.readline()#clear buffer
    spike.write("from hub import motion_sensor\r".encode())
    spike.readline()#clear buffer
    spike.write("import distance_sensor, runloop\r".encode())
    spike.readline()#clear buffer
    #declare varianbles globales spike
    spike.write("der = -1\r".encode())
    spike.readline() #clear buffer
    spike.write("izq = 1\r".encode())
    spike.readline() #clear buffer
    spike.write("error = 0\r".encode())
    spike.readline() #clear buffer
    #declare functions for motors
    spike.write("def Hold():\r".encode())
    spike.readline() #clear buffer 
    spike.write("motor.stop(port.F, stop = motor.HOLD)\r".encode())
    spike.readline() #clear buffer 
    spike.write("motor.stop(port.B, stop = motor.HOLD)\r".encode())
    spike.readline() #clear buffer
    end_Function()

    spike.write("def fc():\r".encode())
    spike.readline() #clear buffer 
    spike.write("motor.stop(port.F, stop = motor.COAST)\r".encode())
    spike.readline() #clear buffer 
    spike.write("motor.stop(port.B, stop = motor.COAST)\r".encode())
    spike.readline() #clear buffer
    end_Function()

    # centrar el vehiculo normal
    spike.write("async def cv_especial():\r".encode())
    spike.readline() #clear buffer 
    spike.write("await motor.run_to_relative_position(port.F, 0, 550,\r".encode())
    spike.readline()#clear buffemotor.stop(port.F, stop = motor.COAST)r
    spike.write("stop = motor.HOLD, acceleration = 1000, deceleration = 1000)\r".encode())
    spike.readline() #clear buffer
    end_Function()

    spike.write("def cv():\r".encode())
    spike.readline() #clear buffer 
    spike.write("runloop.run(cv_especial())\r".encode())
    spike.readline()#clear buffemotor.stop(port.F, stop = motor.COAST)r
    spike.write("return 255\r".encode())
    spike.readline() #clear buffer
    end_Function()

    #pd 
    spike.write("def pd(s1,s2,vel,kp,kd,ea):\r".encode()) #ea es error anterior
    spike.readline() #clear buffer 
    spike.write("error=s1-s2\r".encode())
    spike.readline()#clear buffer
    spike.write("et= (kp*error) + (kd*(error-ea))\r".encode()) #et es error total
    spike.readline()#clear buffer
    spike.readline() #clear buffer
    spike.write("motor.run_to_absolute_position(port.F, int(et*6.4), 600, direction = motor.SHORTEST_PATH, stop = motor.HOLD, acceleration = 10000)\r".encode())
    spike.readline() #clear buffer
    spike.write("motor.set_duty_cycle(port.B, (100)*(vel))\r".encode())
    spike.readline() #clear buffer
    spike.write("return error\r".encode())
    spike.readline() #clear buffer
    end_Function()

    #reset Gyro
    spike.write("def rg(grados):\r".encode())
    spike.readline() #clear buffer
    spike.write("motion_sensor.reset_yaw(grados)\r".encode())
    spike.readline() #clear buffer
    end_Function()

    #imprimir giroscopio
    spike.write("def pg():\r".encode())
    spike.readline() #clear buffer
    spike.write("return motion_sensor.tilt_angles()[0]\r".encode())
    spike.readline() #clear buffer
    end_Function()

    #avanzar derecho
    
    spike.write("def ad(vel,referencia):\r".encode())
    spike.readline() #clear buffer
    spike.write("global error\r".encode())
    spike.readline() #clear buffer
    spike.write("error = pd(((10)*(referencia)),motion_sensor.tilt_angles()[0],vel,0.3,1,error)\r".encode())
    spike.readline() #clear buffer 
    end_Function()

    spike.write("def vuelta(direccion,velocidad,grados):\r".encode())
    spike.readline() #clear buffer
    spike.write("motor.run_to_relative_position(port.F, 313*(direccion), 550)\r".encode())
    spike.readline() #clear buffer
    spike.write("while abs(grados*10) > abs(motion_sensor.tilt_angles()[0]):\r".encode())
    spike.readline() #clear buffer
    spike.write("motor.set_duty_cycle(port.B, (100)*(velocidad))\r".encode())
    spike.readline() #clear buffer
    spike.write(chr(127).encode()) #suprimir linea
    spike.readline() #clear buffer
    spike.write("fc()\r".encode())   
    spike.readline() #clear buffer
    spike.write("return 255\r".encode())
    spike.readline() #clear buffer
    end_Function()
    end_Function()

    spike.write("def vuelta_cubos(direccion,velocidad):\r".encode())
    spike.readline() #clear buffer
    spike.write("motor.run_to_relative_position(port.F, 313*(direccion), 550)\r".encode())
    spike.readline() #clear buffer
    spike.write("while 1:\r".encode())
    spike.readline() #clear buffer
    spike.write("motor.set_duty_cycle(port.B, (100)*(velocidad))\r".encode())
    spike.readline() #clear buffer
    end_Function()
    end_Function()

    spike.write("async def vuelta_wait(direccion,velocidad,grados):\r".encode())
    spike.readline() #clear buffer
    spike.write("await motor.run_to_relative_position(port.F, 313*(direccion), 550)\r".encode())
    spike.readline() #clear buffer
    spike.write("while abs(grados*10) > abs(motion_sensor.tilt_angles()[0]):\r".encode())
    spike.readline() #clear buffer
    spike.write("motor.set_duty_cycle(port.B, (100)*(velocidad))\r".encode())
    spike.readline() #clear buffer
    spike.write(chr(127).encode()) #suprimir linea
    spike.readline() #clear buffer
    spike.write("fc()\r".encode())   
    spike.readline() #clear buffer
    end_Function()
    end_Function()

    spike.write("def vuelta_especial(direccion,velocidad,grados):\r".encode())
    spike.readline() #clear buffer
    spike.write("runloop.run(vuelta_wait(direccion,velocidad,grados))\r".encode())
    spike.readline() #clear buffer
    spike.write("return 255\r".encode())
    spike.readline() #clear buffer
    end_Function()

    spike.write("def da(vel, referencia):\r".encode())
    spike.readline() #clear buffer
    spike.write("global error\r".encode())
    spike.readline() #clear buffer
    spike.write("error = pd(motion_sensor.tilt_angles()[0],referencia,vel,0.3,1,error)\r".encode())
    spike.readline() #clear buffer
    end_Function()

    spike.write("def cubos(dis_cubo, vel, referencia):\r".encode())
    spike.readline() #clear buffer
    spike.write("global error\r".encode())
    spike.readline() #clear buffer
    spike.write("error = pd(dis_cubo,referencia,vel,90,1,error)\r".encode())
    spike.readline() #clear buffer
    end_Function()

    spike.write("def ag(vel,grados,referencia):\r".encode())
    spike.readline() #clear buffer
    spike.write("error = 0\r".encode())
    spike.readline() #clear buffer
    spike.write("motor.reset_relative_position(port.B,0)\r".encode())
    spike.readline() #clear buffer
    spike.write("while abs(grados) > abs(motor.relative_position(port.B)):\r".encode())
    spike.readline() #clear buffer
    spike.write("error = pd(motion_sensor.tilt_angles()[0],((10)*(referencia)),vel,0.3,10,error)\r".encode())
    spike.readline() #clear buffer
    spike.write(chr(127).encode()) #suprimir linea
    spike.readline() #clear buffer
    spike.write("fc()\r".encode())
    spike.readline() #clear buffer
    spike.write("return 255\r".encode())
    spike.readline() #clear buffer
    end_Function()
    end_Function()

    spike.write("def rgrados(vel,grados,referencia):\r".encode())
    spike.readline() #clear buffer
    spike.write("error = 0\r".encode())
    spike.readline() #clear buffer
    spike.write("motor.reset_relative_position(port.B,0)\r".encode())
    spike.readline() #clear buffer
    spike.write("while abs(grados) > abs(motor.relative_position(port.B)):\r".encode())
    spike.readline() #clear buffer
    spike.write("error = pd(((10)*(referencia)),motion_sensor.tilt_angles()[0],vel,0.3,10,error)\r".encode())
    spike.readline() #clear buffer
    spike.write(chr(127).encode()) #suprimir linea
    spike.readline() #clear buffer
    spike.write("fc()\r".encode())
    spike.readline() #clear buffer
    spike.write("return 255\r".encode())
    spike.readline() #clear buffer
    end_Function()
    end_Function()

#se hacen las funciones en la rasp que utilizan las funciones mandadas por serial

def centrar_vehiculo():
    spike.write("cv()\r".encode())
    spike.readline() #limpia el buffer 
    return_value = spike.readline().decode()
    if return_value == "":
        return_value = "0"
    while int(return_value) != 255:
        return_value = spike.readline().decode()
        if return_value == "":
            return_value = "0"

def Hold_motors():
    spike.write("Hold()\r".encode())
    spike.readline() #clear buffer

def Coast_motors():
    spike.write("fc()\r".encode())
    spike.readline() #clear buffer


def end_Function():
    spike.write("\r".encode())
    spike.readline()#clear 
    spike.write("\r".encode())
    spike.readline()#clear buffer
    spike.write("\r".encode())
    spike.readline()#clear buffer
    
def reset_gyro(grados):
    spike.write(("rg("+str(grados)+")\r").encode())
    spike.readline() #clear buffer

def print_gyro():
    spike.write("pg()\r".encode())
    spike.readline() #clear buffer
    return_value = spike.readline().decode()
    print("grioscopio: ",return_value)
    return int(return_value)
    
def avanzar_distancia(vel,distancia,referencia):
    #avanzar cierta distancia sensro del fente, creo que no se ocupo 
    pass

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
    Hold_motors()
    time.sleep(0.05)

#para la slaida del estacionamiento
def vuelta_especial(direccion,velocidad,grados):
    spike.write(("vuelta_especial("+str(direccion)+","+str(velocidad)+","+str(grados)+")\r").encode())
    spike.readline() #clear buffer
    return_value = spike.readline().decode()
    if return_value == "":
        return_value = "0"
    while int(return_value) != 255:
        return_value = spike.readline().decode()
        if return_value == "":
            return_value = "0"
    print("Fin de la vuelta")
    Hold_motors()

#trata de poner centrados los cubos al dar una vuelta
def vuelta_cubos(velocidad):
    try:
        while not camera_queue.full():
            print("ESPERANDO CAMARA")
            time.sleep(0.001)
        obj_camera = camera_queue.get()
        if obj_camera.label == "redbox":
            print("es un cubo rojo")
            spike.write(("vuelta_cubos("+str(der)+","+str(velocidad)+")\r").encode())
            spike.readline() #clear buffer
            while obj_camera.punto_medio_x <= 0.6:
                if camera_queue.full():
                    obj_camera = camera_queue.get()
            spike.write(chr(3).encode())
            spike.readline() #clear buffer
            spike.readline() #clear buffer
            spike.readline() #clear buffer
            spike.readline() #clear buffer
            Hold_motors()
        else:
            spike.write(("vuelta_cubos("+str(izq)+","+str(velocidad)+")\r").encode())
            spike.readline() #clear buffer
            while obj_camera.punto_medio_x >= 4:
                if camera_queue.full():
                    obj_camera = camera_queue.get()
            spike.write(chr(3).encode())
            spike.readline() #clear buffer
            spike.readline() #clear buffer
            spike.readline() #clear buffer
            spike.readline() #clear buffer
            Hold_motors()
        Coast_motors()
        centrar_vehiculo()
    except KeyboardInterrupt:
        Coast_motors()

def avanzar_detection_izquierdo_lidar(vel,referencia):
    try:
        while not vision_queue.full():
            print("ESPERANDO EL QUEVE PARA DETECTAR EL VACIO IZQUIERDO\n")
            time.sleep(0.001)
        obj = vision_queue.get()
        while obj.dist_left < 1350 or obj.dist_front > 1100:
            spike.write(("da("+str(vel)+","+str(referencia)+")\r").encode())
            spike.readline() #clear buffer
            if vision_queue.full():
                #print("obtuvo el queve")
                obj = vision_queue.get()
            #print(obj.dist_left, obj_dist_front)
        Coast_motors()
    except queue.Empty:
        pass
    except KeyboardInterrupt:
        Coast_motors()
    
def avanzar_detection_derecho_lidar(vel,referencia):
    try:
        while not vision_queue.full():
            print("ESPERANDO EL QUEVE PARA DETECTAR EL VACIO DERECHO\n")
            time.sleep(0.001)
        obj = vision_queue.get()
        while obj.dist_right < 1350 or obj.dist_front > 1100:
            spike.write(("da("+str(vel)+","+str(referencia)+")\r").encode())
            spike.readline() #clear buffer
            if vision_queue.full():
                #print("obtuvo el queve")
                obj = vision_queue.get()
            #print(obj.dist_right, obj.dist_front)
        Coast_motors()
    except queue.Empty:
        pass
    except KeyboardInterrupt:
        Coast_motors()

def avanzar_recto_grados(velocidad,grados,referencia):
    spike.write(("ag("+str(velocidad)+","+str(grados)+","+str(referencia)+")\r").encode())
    spike.readline() #clear buffer
    return_value = spike.readline().decode()
    if return_value == "":
        return_value = "0"
    while int(return_value) != 255:
        return_value = spike.readline().decode()
        if return_value == "":
            return_value = "0"
    print("Fin de los grados")
    Coast_motors()

def retroceder_recto_grados(velocidad,grados,referencia):
    spike.write(("rgrados("+str(velocidad)+","+str(grados)+","+str(referencia)+")\r").encode())
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

#sigue cubos po rl amitad si el argumento es true entonces tiene en cuenta el vacio
def seguir_cubos_mitad(vel,distancia,segundo_cubo = False):
    try:
        if segundo_cubo == True:
            while not vision_queue.full():
                a = 0
                time.sleep(0.001)
            obj_lidar = vision_queue.get()
            while not camera_queue.full():
                a = 0
                time.sleep(0.001)
            obj_camera = camera_queue.get()
            while obj_camera.area < distancia and obj_camera.label != None and obj_lidar.dist_left < 1300:
                spike.write(("cubos("+str(0.5)+","+str(vel)+","+str(round(obj_camera.punto_medio_x,3))+")\r").encode())
                spike.readline() #clear buffer
                if camera_queue.full():
                    obj_camera = camera_queue.get()
                if vision_queue.full():
                    obj_lidar = vision_queue.get()
                time.sleep(0.001)
        else:
            while not camera_queue.full():
                a = 0
                time.sleep(0.001)
            obj_camera = camera_queue.get()
            while obj_camera.area < distancia and obj_camera.label != None:
                spike.write(("cubos("+str(0.5)+","+str(vel)+","+str(round(obj_camera.punto_medio_x,3))+")\r").encode())
                spike.readline() #clear buffer
                if camera_queue.full():
                    obj_camera = camera_queue.get()
                time.sleep(0.001)
        Coast_motors()
    except queue.Empty:
        pass
    except KeyboardInterrupt:
        Coast_motors()

#para esquivar los cubos verdes
def seguir_cubos_verde(vel,distancia):
    try:
        while not camera_queue.full():
            a = 0
            time.sleep(0.001)
        obj_camera = camera_queue.get()
        while obj_camera.area < distancia and obj_camera.label != None:
            spike.write(("cubos("+str(round(obj_camera.punto_medio_x,3))+","+str(vel)+","+str(0.1)+")\r").encode())
            print(obj_camera.punto_medio_x," siguiendo verde\n")
            spike.readline() #clear buffer
            if camera_queue.full():
                obj_camera = camera_queue.get()
            #print(0.5-obj_camera.punto_medio_x)
            time.sleep(0.001)
        Coast_motors()
    except queue.Empty:
        pass
    except KeyboardInterrupt:
        Coast_motors()

#para esquivar los cubos rojos
def seguir_cubos_rojo(vel,distancia):
    try:
        while not camera_queue.full():
            a = 0
            time.sleep(0.001)
        obj_camera = camera_queue.get()
        while obj_camera.area < distancia and obj_camera.label != None:
            spike.write(("cubos("+str(round(obj_camera.punto_medio_x,3))+","+str(vel)+","+str(0.9)+")\r").encode())
            spike.readline() #clear buffer
            if camera_queue.full():
                obj_camera = camera_queue.get()
            print(0.5-obj_camera.punto_medio_x)
            time.sleep(0.001)
        Coast_motors()
    except queue.Empty:
        pass
    except KeyboardInterrupt:
        Coast_motors()

def seguir_pared_derecho(velocidad,grados):
    #es la funcion de ale que utiliza dos puntos en la pared y se corrige con el sensor derecho
    pass

def seguir_pared_izquierdo(velocidad,grados):
    #es la funcion de ale que utiliza dos puntos en la pared y se corrige con el sensor izquierdo
    pass
    
#funcion para el lidar que obtiene las medidas en angulos especificos
def RPlidar_worker():
    try:
        print("inicio lidar")
        lidar.reset()
        time.sleep(0.5)
        angles = [0]*360
        for scan in lidar.iter_scans( min_len=0, max_buf_meas=4000):
            for (_, angle, distance) in scan:
                angles[min([359,floor(angle)])] = distance
            lidar_data = RPlidar_data(angles[0],angles[90],angles[75],angles[285],angles[270],time.time())
            if vision_queue.full(): #check if the queue is full
                try:
                    #print("queue full")
                    vision_queue.get() # erase old values from the queue to have latest information 
                except vision_queue.not_empty:
                    pass
            vision_queue.put(lidar_data) #load new data on the queue
            #vision_queue.task_done()
    except RPLidarException as e:
        print("error in lidar",e)
        lidar.clean_input()

def dos_puntos_izquierda(velocidad, grados, referencia):
    while not vision_queue.full():
        print("ESPERANDO EL QUEVE PARA EL PUNTO IZQUIERDO\n")
        time.sleep(0.001)
    obj = vision_queue.get()
    y1 = obj.dist_left
    spike.write(("ag("+str(velocidad)+","+str(grados)+","+str(referencia)+")\r").encode())
    spike.readline() #clear buffer
    return_value = spike.readline().decode()
    if return_value == "":
        return_value = "0"
    while int(return_value) != 255:
        return_value = spike.readline().decode()
        if return_value == "":
            return_value = "0"
    Coast_motors()
    while not vision_queue.full():
        print("ESPERANDO EL QUEVE PARA EL PUNTO IZQUIERDO\n")
        time.sleep(0.001)
    obj = vision_queue.get()
    y2 = obj.dist_left
    variacion = y2 - y1
    pendiente = int(radianes_a_grados(np.arctan(variacion / abs(175*grados/360)))*-10)
    print(pendiente)
    return pendiente

def dos_puntos_derecha(velocidad, grados, referencia):
    while not vision_queue.full():
        print("ESPERANDO EL QUEVE PARA EL PUNTO IZQUIERDO\n")
        time.sleep(0.001)
    obj = vision_queue.get()
    y1 = obj.dist_right
    spike.write(("ag("+str(velocidad)+","+str(grados)+","+str(referencia)+")\r").encode())
    spike.readline() #clear buffer
    return_value = spike.readline().decode()
    if return_value == "":
        return_value = "0"
    while int(return_value) != 255:
        return_value = spike.readline().decode()
        if return_value == "":
            return_value = "0"
    Coast_motors()
    while not vision_queue.full():
        print("ESPERANDO EL QUEVE PARA EL PUNTO IZQUIERDO\n")
        time.sleep(0.001)
    obj = vision_queue.get()
    y2 = obj.dist_right
    variacion = y2 - y1
    pendiente = int(radianes_a_grados(np.arctan(variacion / abs(175*grados/360)))*10)
    print(pendiente)
    return pendiente    

#no se utiliza la funcion        
def cubos_salida_verde():
    try:
        while not camera_queue.full():
            print("no esta lista")
            time.sleep(0.001)
        obj_camera = camera_queue.get()
        if obj_camera.label == "greenbox":
            reset_gyro(0)
            vuelta_especial(der,60,90)
        elif obj_camera.label == "redbox":
            reset_gyro(0)
            avanzar_recto_grados(60,1000,60)
            vuelta_especial(der,60,90)
        else:
            avanzar_recto_grados(60,600,60)
            vuelta_especial(der,60,90)
            
            pass
        Coast_motors()    
    except KeyboardInterrupt:
        Coast_motors()

def esquivar_cubos_primero(is_second_cube = False):
    izq = -1
    der = 1
    seguir_cubos_mitad(60,0.8,is_second_cube)
    while not camera_queue.full():
        a = 0
        time.sleep(0.001)
    obj_camera = camera_queue.get()
    if obj_camera.label != None:
        
        if obj_camera.label == "redbox":
            seguir_cubos_rojo(60,4)
        else:
            seguir_cubos_verde(60,4)
        centrar_vehiculo()
        time.sleep(0.1)
        referencia = print_gyro()
        time.sleep(0.1)
        avanzar_recto_grados(60,401,referencia)
        reset_gyro(0)
        time.sleep(0.5)
        if obj_camera.label == "redbox":
            vuelta_grados(izq,60,referencia/10)
        else:
            vuelta_grados(der,60,referencia/10)
    centrar_vehiculo()
    reset_gyro(0)

def evalua_cubos_esquina_para_izquierda():
    avanzar_detection_izquierdo_lidar(60,0)
    time.sleep(0.5)
    while not camera_queue.full():
        print("esperando camara")
        time.sleep(0.001)
    obj_camera = camera_queue.get()
    while not vision_queue.full():
        print("esperando lidar")
        time.sleep(0.001)
    obj_lidar = vision_queue.get() 
    if obj_camera.label != None:
        if obj_lidar.dist_left > 1300 and obj_camera.label == "greenbox":
            print("verde")
            avanzar_recto_grados(60,70,0)
            vuelta_grados(izq,60,88)
            centrar_vehiculo()
            avanzar_recto_grados(60,100,90)
            reset_gyro(0)
            
        else:
            print("rojo")
            avanzar_recto_grados(60,1200,0)
            vuelta_grados(izq,60,88)
            centrar_vehiculo()
            avanzar_recto_grados(60,100,90)
            reset_gyro(0)
        Coast_motors()
        


def seccion():
    reset_gyro(0)
    esquivar_cubos_primero()
    while not camera_queue.full():
        a = 0
        time.sleep(0.001)
    obj_camera = camera_queue.get()
    while not vision_queue.full(): 
        print("no estoy listo\n")
        time.sleep(0.001)
    obj_lidar = vision_queue.get()
    print(obj_camera)
    if obj_camera.label != None:
        if obj_camera.label != None and obj_lidar.dist_left < 1300 and obj_camera.area > 0.04:
            print("hay dos cubos en la seccion \n")
            esquivar_cubos_primero(True)
        else:
            if obj_lidar.dist_left < 1300:
                reset_gyro(0)
                grados = dos_puntos_izquierda(60,500,0)
                avanzar_detection_izquierdo_lidar(60,grados)
            else:
                Coast_motors()
                
    else:
        correcion = dos_puntos_izquierda(60,500,0)
        reset_gyro(correcion)
        avanzar_detection_izquierdo_lidar(60,0)

    while not camera_queue.full():
        a = 0
        time.sleep(0.001)
    obj_camera = camera_queue.get()
    if obj_camera.label != None:
        evalua_cubos_esquina_para_izquierda()
    else:
        avanzar_recto_frente(60,65,0)
        vuelta_especial(izq,60,88)
    centrar_vehiculo()
    time.sleep(0.5)
    Coast_motors()

        

def avanzar_recto_frente(vel, distancia, referencia):
    try:
        while not vision_queue.full():
            print("ESPERANDO EL QUEVE PARA DETECTAR EL VACIO IZQUIERDO\n")
            time.sleep(0.001)
        obj = vision_queue.get()
        while obj.dist_front > (distancia*10):
            spike.write(("da("+str(vel)+","+str(referencia)+")\r").encode())
            spike.readline() #clear buffer
            if vision_queue.full():
                #print("obtuvo el queve")
                obj = vision_queue.get()
            #print(obj.dist_left, obj_dist_front)
        Coast_motors()
    except queue.Empty:
        pass
    except KeyboardInterrupt:
        Coast_motors()

# Main program thread
def main_program_worker(h):
    runing = True
    izq = -1
    der = 1
    print("presiona el boton\n")
    while(lgpio.gpio_read(h, 4) == 1):
        lgpio.gpio_write(h, 17, 1)
        time.sleep(0.05)
        lgpio.gpio_write(h, 17, 0)
        time.sleep(0.05)
    lgpio.gpio_write(h, 17, 0)
    print("empezando ....\n")
    reset_gyro(0)
    while not vision_queue.full(): 
        print("no estoy listo\n")
        time.sleep(0.001)
    obj_lidar = vision_queue.get()
    while not camera_queue.full():
        print("esperando camara")
        time.sleep(0.001)
    obj_camera = camera_queue.get()
    while runing:
    
        if obj_lidar.dist_left > 600:
            vuelta_especial(izq,60,60)
            time.sleep(1)
            if camera_queue.full():
                obj_camera = camera_queue.get()
            if obj_camera.label == "greenbox" and obj_camera.area > 1.2:
                vuelta_especial(izq,60,75)
                avanzar_recto_grados(60,700,90)
                reset_gyro(0)
                vuelta_especial(der,60,88)
                time.sleep(0.5)
                centrar_vehiculo()
                evalua_cubos_esquina_para_izquierda()
                

            elif obj_camera.label == "redbox" and obj_camera.area > 1.2:
                #time.sleep(10)
                reset_gyro(0)
                vuelta_especial(der,60,60)
                time.sleep(0.5)
                centrar_vehiculo()
                avanzar_recto_frente(60,65,-63)
                reset_gyro(0)
                vuelta_grados(izq,60,88)
                centrar_vehiculo()

            else:
                reset_gyro(0)
                vuelta_especial(der,60,60)
                time.sleep(0.5)
                centrar_vehiculo()
                avanzar_recto_frente(60,65,-63)
                reset_gyro(0)
                vuelta_grados(izq,60,88)
                centrar_vehiculo()

            seccion()
        runing = False
    
#convercion de grados a radianes
def grados_a_radianes(grados):
    grados_convertidos : int = (grados)/(180/np.pi)
    return grados_convertidos

#convercion de radianes a grados
def radianes_a_grados(radianes):
    radianes_convertidos : int = (radianes)*(180/np.pi)
    return radianes_convertidos

#funcion que calcula la correcion del giroscopio utilizando medidas del lidar y funciones trigonométricas
def correcion(correcion_del_angulo):
    if vision_queue.full():
        try:
            obj = vision_queue.get_nowait()
            H = int(obj.dist_75*10)
            CA = int(obj.dist_left*10)
            lado_faltante = np.sqrt((H**2) + (CA**2)- (2*H*CA*np.cos(grados_a_radianes(correcion_del_angulo))))
            ley_de_senos = H*np.sin(grados_a_radianes(correcion_del_angulo))
            angulo_correcion = radianes_a_grados(np.arcsin(ley_de_senos/lado_faltante))
            if radianes_a_grados(np.arcsin(CA/H)) < 75:
                angulo_correcion = 180-angulo_correcion
            else:
                angulo_correcion = angulo_correcion
            angulo_correcion = angulo_correcion-90
            print(angulo_correcion)
            return angulo_correcion
        except queue.Empty:
            pass

#funcion que habilita el acceso a los gpio
def setup_gpio():
    try:
        h = lgpio.gpiochip_open(0) #se habilita el acceso a los gpio
        lgpio.gpio_claim_input(h, 4)
        lgpio.gpio_claim_output(h, 12)
        lgpio.gpio_claim_output(h, 17)
        lgpio.gpio_write(h, 12, 1)
        lgpio.gpio_write(h, 17, 1)
        for sensor in SENSORS:
            lgpio.gpio_claim_output(h, sensor.pin_gpio_enable)
            lgpio.gpio_write(h, sensor.pin_gpio_enable, 0)
            time.sleep(0.001)
        for sensor in SENSORS:
            lgpio.gpio_write(h, sensor.pin_gpio_enable, 1)
            time.sleep(0.001)
            sensor.identifier = adafruit_vl53l1x.VL53L1X(i2c)
            print(sensor.identifier.model_info)
            sensor.identifier.distance_mode = 1
            sensor.identifier.timing_budget = 33
            sensor.identifier.roi_center = 199
            print(sensor.identifier.roi_center) 
            sensor.identifier.roi_xy = (16,8)
            print(sensor.identifier.roi_xy) 
            sensor.identifier.set_address(sensor.slave_address)
        lgpio.gpio_write(h, 12, 0)
        print("GPIO inicializados")
        return h, True

    except Exception as e:
        print(f"Error al inicializar sensores: {e}")
        return h, False

#funcion que limpia los gpio
def cleanup_gpio(h):
    try:
        for sensor in SENSORS:
            try:
                lgpio.gpio_free(h, sensor.pin_gpio_enable)
            except:
                pass
        lgpio.gpio_free(h, 4) 
        lgpio.gpio_write(h, 12, 1)
        lgpio.gpio_write(h, 17, 1)
        lgpio.gpiochip_close(h)
        print("GPIO libres")
    except:
        pass

def sensor_worker():
    for sensor in SENSORS:
        sensor.identifier.start_ranging()
    obj = Distance_data(0,0,0)
    distancias = [0,0,0]
    while 1:
        try:
            for i,sensor in enumerate(SENSORS):
                if sensor.identifier.data_ready:
                    distancias[i] = sensor.identifier.distance 
                    sensor.identifier.clear_interrupt()
            if distancias[0] != 0 or distancias[0] == None:
                obj.distance_izq = distancias[0]
            if distancias[1] != 0 or distancias[1] == None:
                obj.distance_derecho = distancias[1]
            if distancias[2] != 0 or distancias[2] == None:
                obj.distance_trasero = distancias[2]

            if distance_queue.full(): 
                try:
                    distance_queue.get() # erase old values from the queue to have latest information 
                except distance_queue.not_empty:
                    pass
            if obj.distance_izq != 0 or obj.distance_derecho != 0 or obj.distance_trasero != 0 or obj.distance_izq == None or obj.distance_derecho == None or obj.distance_trasero == None:
                distance_queue.put(obj)
                #distance_queue.task_done() 
        except Exception as e:
            print(f"Error al leer sensor: {e}")

#==========================================================================================================
#funciones de la camara            

# This is the callback function that will be called when data is available from the pipeline
def app_callback(pad, info, user_data):
    # Get the GstBuffer from the probe info
    buffer = info.get_buffer()
    # Check if the buffer is valid
    obj_camera = Camera_data(None, 0,0, 0,0)
    if buffer is None:
        return Gst.PadProbeReturn.OK
    # Using the user_data to count the number of frames
    user_data.increment()
    string_to_print = f"Frame count: {user_data.get_count()}\n"

    # Get the caps from the pad
    format, width, height = get_caps_from_pad(pad)

    # If the user_data.use_frame is set to True, we can get the video frame from the buffer
    frame = None
    if user_data.use_frame and format is not None and width is not None and height is not None:
        # Get video frame
        frame = get_numpy_from_buffer(buffer, format, width, height)

    # Get the detections from the buffer
    roi = hailo.get_roi_from_buffer(buffer)
    detections = roi.get_objects_typed(hailo.HAILO_DETECTION)
    
    # Parse the detections
    detection_count = 0
    max_area = 0
    index = 0
    
    for detection in detections:
        label = detection.get_label()
        bbox = detection.get_bbox()
        confidence = detection.get_confidence()
        # Get track ID
        track_id = 0
        track = detection.get_objects_typed(hailo.HAILO_UNIQUE_ID)
        if len(track) == 1:
            track_id = track[0].get_id()
        string_to_print += (f"Detection: ID: {track_id} Label: {label} Confidence: {confidence:.2f}\n")
        detection_count += 1
        xmin = bbox.xmin()
        ymin = bbox.ymin()
        width = bbox.width()
        height = bbox.height()
        x1 = (xmin * width)
        y1 = (ymin * height)
        x2 = ((xmin + width) * width)
        y2 = ((ymin + height) * height)
        max_area_temp = ((x2 - x1) * (y2 - y1))*1000
        if max_area_temp > max_area:
            index = detection_count
            max_area = max_area_temp
    if index != 0:
        detection = detections[index-1]
        obj_camera.label = detection.get_label()
        obj_camera.punto_medio_x = (detection.get_bbox().xmin() + detection.get_bbox().xmax())/2
        obj_camera.punto_medio_y = (detection.get_bbox().ymin() + detection.get_bbox().ymax())/2
        obj_camera.time = time.time()
        obj_camera.area = max_area
        #print(obj_camera)
    if camera_queue.full():
        try:
            camera_queue.get()
        except camera_queue.not_empty:
            pass
    camera_queue.put(obj_camera)

    if user_data.use_frame:
        # Note: using imshow will not work here, as the callback function is not running in the main thread
        # Let's print the detection count to the frame
        cv2.putText(frame, f"Detections: {detection_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        # Example of how to use the new_variable and new_function from the user_data
        # Let's print the new_variable and the result of the new_function to the frame
        cv2.putText(frame, f"{user_data.new_function()} {user_data.new_variable}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        # Convert the frame to BGR
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        user_data.set_frame(frame)

    #print(string_to_print)
    return Gst.PadProbeReturn.OK

def detectar_cubo(vel,referencia):
        try:
            while not camera_queue.full():
                print("queue full waiting detectar cubo\n")
                time.sleep(0.001)
            obj = camera_queue.get()
            while obj.area < 1:
                spike.write(("da("+str(vel)+","+str(referencia)+")\r").encode())
                spike.readline() #clear buffer
                if camera_queue.full():
                    obj = camera_queue.get()
                print("avanzando",obj.area)
                time.sleep(0.001)
                Coast_motors()
        except queue.Empty:
            pass

def main():
    global running, spike
    print("voy")
    user_data = user_app_callback_class()
    app = GStreamerDetectionApp(app_callback, user_data)
    h, success = setup_gpio()
    time.sleep(1)
    try:
        spike = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        screenSimplified()
        initialize_Libraries()
        #threading.Thread(target=sensor_worker).start()
        threading.Thread(target=RPlidar_worker).start()
        time.sleep(5)
        threading.Thread(target=main_program_worker, args=(h,)).start()
        while running:
            try:
                """if distance_queue.full():
                    obj_distance = distance_queue.get_nowait()
                    print(obj_distance.distance_izq, obj_distance.distance_derecho, obj_distance.distance_trasero)
                """
                app.run()
                 #distancias = distance_queue.get_nowait()
                time.sleep(0.0005)
                                
                #angulo_correcion = correcion(15)

                """if vision_queue.full():
                    try:
                        obj = vision_queue.get_nowait()
                        print(obj.dist_left, obj.dist_75, obj.dist_315, obj.dist_right)
                    except queue.Empty:
                        pass"""
                app.stop()
                running = False
    
            except KeyboardInterrupt:
                print("\nProgram interrupted! Cleaning up...")
                spike.write(chr(3).encode())
                spike.readline() #clear buffer
                spike.readline() #clear buffer
                spike.readline() #clear buffer
                spike.readline() #clear buffer
                Coast_motors()
                spike.close()
                print("\nStopping...")
                running = False
                cleanup_gpio(h)
                lidar.stop()
                break
            
    except Exception as e:
        print(f"Error: {e}")
        cleanup_gpio(h)
    finally:
        running = False
        lidar.stop()
        lidar.disconnect()
        cleanup_gpio(h)

if __name__ == "__main__":
    # Set up signal handler for clean exit
    project_root = Path(__file__).resolve().parent.parent
    env_file     = project_root / ".env"
    env_path_str = str(env_file)
    os.environ["HAILO_ENV_FILE"] = env_path_str
    # Create an instance of the user app callback class
    
    main()
    print("Exiting...")
