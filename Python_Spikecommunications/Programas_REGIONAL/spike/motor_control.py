from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait
import sys

# Motor B: Traccion (grande)
# Motor D: Direccion (mediano)

def limpiar_recursos_ackermann(motor_traccion_obj=None, motor_direccion_obj=None):
    print("\nLimpiando recursos Ackermann...")
    try:
        if motor_traccion_obj:
            motor_traccion_obj.brake()
        if motor_direccion_obj:
            motor_direccion_obj.brake()
        print("Motores Ackermann detenidos y recursos liberados.")
    except Exception as e:
        print(f"Error al limpiar recursos Ackermann: {e}")

print("Iniciando rutina Ackermann: Avanzar...")
motor_traccion = None
motor_direccion = None

try:
    hub = PrimeHub()
    print("Hub inicializado.")

    print("Inicializando motor de traccion (Puerto B)...")
    motor_traccion = Motor(Port.B)
    print("Motor de traccion inicializado.")

    print("Inicializando motor de direccion (Puerto D)...")
    motor_direccion = Motor(Port.D)
    print("Motor de direccion inicializado.")

    print("Iniciando secuencia de movimiento Ackermann.")

    print("Asegurando motor de direccion (D)...")
    motor_direccion.brake()
    wait(500)

    velocidad_traccion = 50
    tiempo_avance_ms = 5000
    print(f"Moviendo motor de traccion (B) adelante al {velocidad_traccion}% durante {tiempo_avance_ms / 1000}s...")
    motor_traccion.dc(velocidad_traccion)
    wait(tiempo_avance_ms)

    print("Deteniendo motor de traccion (B)...")
    motor_traccion.brake()
    wait(500)

    print("Rutina de avance Ackermann completada.")

except Exception as e:
    print(f"\nError durante la rutina Ackermann: {e}")
finally:
    print("Ejecutando limpieza final Ackermann...")
    limpiar_recursos_ackermann(motor_traccion, motor_direccion)
    print("Script Python Ackermann finalizado.")
