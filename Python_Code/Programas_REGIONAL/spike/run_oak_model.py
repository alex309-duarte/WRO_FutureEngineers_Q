from roboflowoak import RoboflowOak
import cv2
import time
import numpy as np

if __name__ == '__main__':
    # --- CONFIGURACIÓN ---
    # Reemplaza estos valores con los tuyos:
    MODEL_ID = "wro-round-2-mefmf"  # El ID de tu proyecto/dataset de Roboflow
    MODEL_VERSION = "3"             # La versión de tu modelo
    
    # ¡¡¡IMPORTANTE!!! Usa tu CLAVE API PRIVADA aquí
    PRIVATE_API_KEY = "1QaioZ6lSVket4h0AjJP" 
    
    CONFIDENCE_THRESHOLD = 0.5 # Umbral de confianza para mostrar detecciones (0.0 a 1.0)
    OVERLAP_THRESHOLD = 0.5    # Umbral de superposición para Non-Max Suppression (0.0 a 1.0)

    # Cambia a False si tu cámara OAK no tiene sensor de profundidad (ej. OAK-1)
    # Déjalo en True para cámaras como OAK-D, OAK-D Lite, OAK-D S2, etc.
    HAS_DEPTH = True 
    # --- FIN DE LA CONFIGURACIÓN ---

    if PRIVATE_API_KEY == "TU_CLAVE_API_PRIVADA_AQUI":
        print("ERROR: Por favor, reemplaza 'TU_CLAVE_API_PRIVADA_AQUI' con tu verdadera Clave API Privada de Roboflow.")
        exit()

    print(f"Inicializando RoboflowOak con:")
    print(f"  Modelo ID: {MODEL_ID}")
    print(f"  Versión del Modelo: {MODEL_VERSION}")
    print(f"  Usando profundidad: {HAS_DEPTH}")
    print(f"  Umbral de Confianza: {CONFIDENCE_THRESHOLD}")
    print(f"  Umbral de Superposición: {OVERLAP_THRESHOLD}")

    try:
        rf = RoboflowOak(
            model=MODEL_ID,
            confidence=CONFIDENCE_THRESHOLD,
            overlap=OVERLAP_THRESHOLD,
            version=MODEL_VERSION,
            api_key=PRIVATE_API_KEY, # Aquí se usa la Clave API Privada
            rgb=True,
            depth=HAS_DEPTH,
            device=None, # Especifica MXID si tienes múltiples cámaras OAK conectadas
            blocking=True
        )
        print("RoboflowOak inicializado correctamente. Iniciando bucle de detección...")

        while True:
            t_start = time.time()
            
            result, frame_with_detections, raw_frame, depth_map = rf.detect()
            
            predictions = result["predictions"]
            
            # --- Procesamiento y visualización ---
            # Imprimir FPS
            t_end = time.time()
            fps = 1 / (t_end - t_start) if (t_end - t_start) > 0 else 0
            cv2.putText(frame_with_detections, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            # --- Calcular y mostrar distancia para cada detección ---
            if HAS_DEPTH and depth_map is not None:
                # Ensure depth_map has 2 dimensions (H, W)
                if depth_map.ndim == 2:
                    for pred in predictions: # predictions is result["predictions"]
                        # pred.bbox is [xmin, ymin, xmax, ymax] in pixel coordinates.
                        # pred.class_name is the string name of the class.
                        
                        center_x = pred.x
                        center_y = pred.y
                        width = pred.width
                        height = pred.height

                        xmin = int(center_x - width / 2)
                        ymin = int(center_y - height / 2)
                        xmax = int(center_x + width / 2)
                        ymax = int(center_y + height / 2)

                        # Clamp coordinates to be within the depth_map dimensions
                        h_depth, w_depth = depth_map.shape
                        # Ensure xmin_clamped < xmax_clamped and ymin_clamped < ymax_clamped for ROI
                        xmin_clamped = max(0, min(xmin, w_depth - 1))
                        ymin_clamped = max(0, min(ymin, h_depth - 1))
                        # Add 1 to ensure roi is at least 1x1, prevents xmax_clamped <= xmin_clamped
                        xmax_clamped = max(xmin_clamped + 1, min(xmax, w_depth -1)) 
                        ymax_clamped = max(ymin_clamped + 1, min(ymax, h_depth -1))
                        
                        # Proceed only if the clamped bbox has a valid area after clamping
                        if xmax_clamped > xmin_clamped and ymax_clamped > ymin_clamped:
                            depth_roi = depth_map[ymin_clamped:ymax_clamped, xmin_clamped:xmax_clamped]
                            # Filter out invalid depth readings (often 0 or max_range for stereo)
                            # Values are in mm. Filter out 0 and very large values (e.g. > 30 meters)
                            valid_depth_values = depth_roi[ (depth_roi > 0) & (depth_roi < 30000) ] 

                            if valid_depth_values.size > 0:
                                median_depth_mm = np.median(valid_depth_values)
                                distance_cm = median_depth_mm / 10.0 # Convert mm to cm
                                
                                # Display the distance on frame_with_detections
                                text_x_pos = xmin # Use original xmin for text positioning
                                text_y_pos = ymin - 7 # A bit above the box
                                if text_y_pos < 10: # If too high, put it below or inside
                                    text_y_pos = ymin + 15 

                                cv2.putText(frame_with_detections, f"{pred.class_name}: {distance_cm:.1f}cm", # Display in cm
                                            (text_x_pos, text_y_pos), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 2)
            # --- Fin de cálculo de distancia ---

            # Mostrar el frame con las detecciones
            cv2.imshow("Roboflow OAK Detections", frame_with_detections)

            # Mostrar mapa de profundidad si está habilitado y disponible
            if HAS_DEPTH and depth_map is not None:
                depth_display = depth_map.copy()
                # Normalizar para visualización si hay valores válidos
                if np.any(depth_display):
                    max_val = np.max(depth_display)
                    if max_val > 0:
                        depth_display = (depth_display / max_val * 255).astype(np.uint8)
                        cv2.imshow("Depth Map", depth_display)
                    else:
                        # Si todos los valores son 0 o el mapa está vacío, muestra una imagen negra
                        cv2.imshow("Depth Map", np.zeros_like(depth_display, dtype=np.uint8))
                else:
                    # Si depth_map es None o está vacío, muestra una imagen negra (ajusta el tamaño si es necesario)
                    # Intenta obtener el tamaño del raw_frame si está disponible
                    h, w = (400, 640) # Tamaño por defecto
                    if raw_frame is not None:
                        h, w, _ = raw_frame.shape
                    cv2.imshow("Depth Map", np.zeros((h, w), dtype=np.uint8))


            # Salir con la tecla 'q'
            if cv2.waitKey(1) == ord('q'):
                print("Saliendo del bucle de detección...")
                break
                
    except Exception as e:
        print(f"Ocurrió un error durante la ejecución: {e}")
        import traceback
        traceback.print_exc()

    finally:
        print("Cerrando todas las ventanas de OpenCV.")
        cv2.destroyAllWindows()
        # Considera si necesitas alguna limpieza adicional para el objeto rf,
        # aunque usualmente las librerías manejan esto en sus destructores.
        print("Programa terminado.")
