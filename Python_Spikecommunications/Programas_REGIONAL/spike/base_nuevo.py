"""
Módulo principal para el control del robot Spike con detección de objetos OAK.
"""
import serial
import time
import threading
import importlib.util
import sys
import os

# Agregar el directorio actual al path para importaciones locales
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# Intentar importar el detector OAK
try:
    from oak_detector import OakDetector
    OAK_AVAILABLE = True
except ImportError as e:
    print(f"Advertencia: No se pudo importar el detector OAK: {e}")
    OAK_AVAILABLE = False

# Verificar si el módulo de la cámara está disponible
try:
    from detectar_cajas import main as iniciar_camara
    CAMARA_DISPONIBLE = True
except ImportError:
    print("Advertencia: No se pudo importar el módulo de la cámara")
    CAMARA_DISPONIBLE = False

# Inicialización del puerto serie
spike = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# Variables globales
der = -1
izq = 1

# Variable para controlar el hilo de la cámara
thread_camara = None

def initialize_Libraries():
    """Inicializa las bibliotecas del robot Spike."""
    spike.write("import motor\r".encode())
    spike.readline()
    spike.write("from hub import port\r".encode())
    spike.readline()  # clear buffer
    spike.write("from hub import motion_sensor\r".encode())
    spike.readline()  # clear buffer
    spike.write("import distance_sensor\r".encode())
    spike.readline()  # clear buffer
    
    # Declarar variables globales en el Spike
    spike.write("der = -1\r".encode())
    spike.readline()  # clear buffer
    spike.write("izq = 1\r".encode())
    spike.readline()  # clear buffer
    spike.write("error = 0\r".encode())
    spike.readline()  # clear buffer
    
    # Funciones básicas del robot
    spike.write("def fr():\r".encode())
    spike.readline()
    spike.write("motor.stop(port.F, stop = motor.HOLD)\r".encode())
    spike.readline()
    spike.write("motor.stop(port.B, stop = motor.HOLD)\r".encode())
    spike.readline()
    spike.write("distancias = [distance_sensor.distance(port.C), distance_sensor.distance(port.A), distance_sensor.distance(port.E)]\r".encode())
    spike.readline()
    end_Function()
    
    spike.write("def fc():\r".encode())
    spike.readline()
    spike.write("motor.stop(port.F, stop = motor.COAST)\r".encode())
    spike.readline()
    spike.write("motor.stop(port.B, stop = motor.COAST)\r".encode())
    spike.readline()
    end_Function()

def end_Function():
    """Marca el final de una función en el Spike."""
    spike.write("\r".encode())
    spike.readline()

def Coast_motors():
    """Detiene los motores suavemente."""
    spike.write("fc()\r".encode())
    spike.readline()

def iniciar_camara_en_segundo_plano():
    """Inicia la cámara en un hilo separado."""
    if CAMARA_DISPONIBLE:
        try:
            print("Iniciando cámara OAK...")
            from detectar_cajas import main as iniciar_camara
            iniciar_camara()
        except Exception as e:
            print(f"Error al iniciar la cámara: {e}")

# Variables globales
spike = None
detector = None

# Configuración del detector OAK (reemplaza con tus valores)
OAK_CONFIG = {
    'model_id': 'wro-round-2-mefmf',  # Reemplaza con tu ID de modelo
    'version': 3,                     # Versión del modelo
    'private_api_key': 'TU_CLAVE_API_AQUI'  # Tu clave API de Roboflow
}

def ejecutar_accion_robot(accion: str, deteccion: dict = None):
    """
    Ejecuta una acción en el robot basada en la detección.
    
    Args:
        accion: Nombre de la acción a ejecutar
        deteccion: Diccionario con información de la detección (opcional)
    """
    try:
        print(f"Ejecutando acción: {accion}")
        
        if accion == "detener":
            # Detener motores
            spike.write(b"fc()\r")
        elif accion == "avanzar":
            # Avanzar a velocidad media
            spike.write(b"motor.run_for_degrees(port.B, 360, 50)\r")
        elif accion == "girar_izquierda":
            # Girar a la izquierda
            spike.write(b"motor.run_for_degrees(port.F, 90, 30)\r")
        elif accion == "girar_derecha":
            # Girar a la derecha
            spike.write(b"motor.run_for_degrees(port.F, -90, 30)\r")
        
        # Leer la respuesta del robot (opcional)
        time.sleep(0.1)
        while spike.in_waiting > 0:
            print(f"Robot: {spike.readline().decode().strip()}")
            
    except Exception as e:
        print(f"Error al ejecutar acción {accion}: {e}")

def configurar_detector():
    """Configura el detector OAK con acciones personalizadas."""
    if not OAK_AVAILABLE:
        print("El detector OAK no está disponible. Verifica las dependencias.")
        return None
    
    try:
        # Crear instancia del detector
        detector = OakDetector(
            model_id=OAK_CONFIG['model_id'],
            version=OAK_CONFIG['version'],
            private_api_key=OAK_CONFIG['private_api_key'],
            confidence_threshold=0.7
        )
        
        # Definir acciones para cada clase detectada
        def cuando_detecte_greenbox(detection):
            print(f"¡Caja verde detectada! Confianza: {detection['confidence']:.2f}")
            ejecutar_accion_robot("girar_izquierda", deteccion=detection)
        
        def cuando_detecte_redbox(detection):
            print(f"¡Caja roja detectada! Confianza: {detection['confidence']:.2f}")
            ejecutar_accion_robot("girar_derecha", deteccion=detection)
        
        # Registrar acciones
        detector.add_action("greenbox", cuando_detecte_greenbox)
        detector.add_action("redbox", cuando_detecte_redbox)
        
        return detector
        
    except Exception as e:
        print(f"Error al configurar el detector OAK: {e}")
        return None

def main():
    """Función principal del programa."""
    global spike, detector
    
    try:
        # Inicializar comunicación con el robot
        spike = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        print("Conexión con el robot establecida.")
        
        # Configurar detector OAK
        if OAK_AVAILABLE:
            detector = configurar_detector()
            if detector:
                print("Iniciando detector OAK...")
                detector.start()
            else:
                print("No se pudo iniciar el detector OAK. Continuando sin detección...")
        else:
            print("El detector OAK no está disponible. Continuando sin detección...")
        
        # Inicializar bibliotecas del robot
        initialize_Libraries()
        print("Robot inicializado. Listo para recibir comandos...")
        
        # Bucle principal
        try:
            while True:
                # Aquí puedes agregar lógica adicional de control
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nPrograma interrumpido por el usuario.")
            
    except Exception as e:
        print(f"\nError inesperado: {e}")
        
    finally:
        # Limpieza
        print("Deteniendo motores y liberando recursos...")
        ejecutar_accion_robot("detener")
        
        if detector:
            detector.stop()
            
        if spike and spike.is_open:
            spike.write(chr(3).encode())  # Envía Ctrl+C al Spike
            time.sleep(0.1)
            while spike.in_waiting > 0:  # Limpiar buffer
                spike.read(spike.in_waiting)
            spike.close()
            
        print("Programa finalizado.")

if __name__ == "__main__":
    main()

