()#!/usr/bin/env python3
"""
Librería para detección de objetos con cámara OAK y ejecución de acciones personalizadas.
"""
import cv2
import depthai as dai
import threading
from typing import Dict, Callable, Any, Optional
import time

class OakDetector:
    def __init__(self, model_id: str, version: int, private_api_key: str, 
                 confidence_threshold: float = 0.6):
        """
        Inicializa el detector OAK.
        
        Args:
            model_id: ID del modelo en Roboflow
            version: Versión del modelo
            private_api_key: Clave API privada de Roboflow
            confidence_threshold: Umbral de confianza mínimo (0-1)
        """
        self.model_id = model_id
        self.version = version
        self.private_api_key = private_api_key
        self.confidence_threshold = confidence_threshold
        self.actions = {}
        self.running = False
        self.thread = None
        
    def add_action(self, class_name: str, callback: Callable[[Dict[str, Any]], None]):
        """
        Registra una acción para una clase de objeto.
        
        Args:
            class_name: Nombre de la clase a detectar (ej: 'greenbox', 'redbox')
            callback: Función a ejecutar cuando se detecte la clase
        """
        if class_name not in self.actions:
            self.actions[class_name] = []
        self.actions[class_name].append(callback)
    
    def _process_detection(self, detection):
        """Procesa una detección y ejecuta las acciones registradas."""
        label = detection.label
        confidence = detection.confidence
        
        if confidence < self.confidence_threshold:
            return
            
        # Crear diccionario con información de la detección
        detection_info = {
            'label': label,
            'confidence': confidence,
            'bbox': {
                'x': detection.xmin,
                'y': detection.ymin,
                'width': detection.xmax - detection.xmin,
                'height': detection.ymax - detection.ymin
            },
            'center': (
                (detection.xmin + detection.xmax) / 2,
                (detection.ymin + detection.ymax) / 2
            ),
            'timestamp': time.time()
        }
        
        # Ejecutar acciones registradas para esta clase
        if label in self.actions:
            for action in self.actions[label]:
                try:
                    action(detection_info)
                except Exception as e:
                    print(f"Error en acción para {label}: {e}")
    
    def _run_detection(self):
        """Bucle principal de detección."""
        # Configurar pipeline de DepthAI
        pipeline = dai.Pipeline()
        
        # Configurar cámara
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_nn = pipeline.create(dai.node.XLinkOut)
        
        xout_rgb.setStreamName("rgb")
        xout_nn.setStreamName("nn")
        
        # Propiedades de la cámara
        cam_rgb.setPreviewSize(416, 416)
        cam_rgb.setInterleaved(False)
        cam_rgb.setFps(30)
        
        # Conectar nodos
        cam_rgb.preview.link(xout_rgb.input)
        
        # Configurar red neuronal
        detection_nn = pipeline.create(dai.node.NeuralNetwork)
        detection_nn.setBlobPath(
            f"https://app.roboflow.com/{self.model_id}/{self.version}/model.blob"
        )
        detection_nn.setNumInferenceThreads(2)
        detection_nn.input.setBlocking(False)
        
        cam_rgb.preview.link(detection_nn.input)
        detection_nn.out.link(xout_nn.input)
        
        # Iniciar dispositivo
        with dai.Device(pipeline) as device:
            print("Dispositivo OAK iniciado. Presiona 'q' para salir.")
            
            # Colas para obtener datos
            q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            q_nn = device.getOutputQueue(name="nn", maxSize=4, blocking=False)
            
            frame = None
            self.running = True
            
            while self.running:
                in_rgb = q_rgb.tryGet()
                in_nn = q_nn.tryGet()
                
                if in_rgb is not None:
                    frame = in_rgb.getCvFrame()
                
                if in_nn is not None:
                    detections = in_nn.detections
                    for detection in detections:
                        self._process_detection(detection)
                
                # Mostrar vista previa
                if frame is not None:
                    # Dibujar detecciones
                    for detection in detections if 'detections' in locals() else []:
                        if detection.confidence >= self.confidence_threshold:
                            bbox = [
                                int(detection.xmin * frame.shape[1]),
                                int(detection.ymin * frame.shape[0]),
                                int((detection.xmax - detection.xmin) * frame.shape[1]),
                                int((detection.ymax - detection.ymin) * frame.shape[0])
                            ]
                            cv2.rectangle(
                                frame, 
                                (bbox[0], bbox[1]), 
                                (bbox[0] + bbox[2], bbox[1] + bbox[3]), 
                                (0, 255, 0), 2
                            )
                            cv2.putText(
                                frame, 
                                f"{detection.label} {detection.confidence:.2f}",
                                (bbox[0], bbox[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 
                                0.5, 
                                (0, 255, 0), 
                                2
                            )
                    
                    cv2.imshow("OAK Detection", frame)
                
                # Salir con 'q'
                if cv2.waitKey(1) == ord('q'):
                    self.stop()
    
    def start(self):
        """Inicia el detector en un hilo separado."""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._run_detection, daemon=True)
            self.thread.start()
    
    def stop(self):
        """Detiene el detector."""
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        cv2.destroyAllWindows()

# Ejemplo de uso
if __name__ == "__main__":
    # Configuración
    MODEL_ID = "wro-round-2-mefmf"  # Reemplaza con tu ID de modelo
    VERSION = 3
    PRIVATE_API_KEY = "TU_CLAVE_API_AQUI"  # Reemplaza con tu clave API
    
    # Crear detector
    detector = OakDetector(
        model_id=MODEL_ID,
        version=VERSION,
        private_api_key=PRIVATE_API_KEY,
        confidence_threshold=0.7
    )
    
    # Definir acciones personalizadas
    def cuando_detecte_greenbox(detection):
        print(f"¡Caja verde detectada! Confianza: {detection['confidence']:.2f}")
        print(f"Posición: {detection['center']}")
        # Aquí puedes agregar código para controlar el robot
        # Por ejemplo: robot.girar_izquierda()
    
    def cuando_detecte_redbox(detection):
        print(f"¡Caja roja detectada! Confianza: {detection['confidence']:.2f}")
        print(f"Posición: {detection['center']}")
        # Aquí puedes agregar código para controlar el robot
        # Por ejemplo: robot.girar_derecha()
    
    # Registrar acciones
    detector.add_action("greenbox", cuando_detecte_greenbox)
    detector.add_action("redbox", cuando_detecte_redbox)
    
    # Iniciar detección
    try:
        detector.start()
        # Mantener el programa en ejecución
        while detector.running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nDeteniendo...")
    finally:
        detector.stop()
