#!/usr/bin/env python3
"""
Spike Prime + Raspberry Pi – Obstacle Challenge
================================================
Combina:
 • Detección de señales (pilares verdes/rojos) con OAK-D mediante RoboflowOak
 • Seguimiento de carril usando sensores ultrasónicos HC-SR04
 • Control de motores del Spike Prime por puerto serie

Flujo general:
 1) Hilo Vision: detecta pilar más cercano → publica (color, distancia)
 2) Hilos Ultrasónicos: actualizan distancias izquierda / frente / derecha
 3) Bucle principal:
    – Determina "lado de carril" (mantener izq./der.) según color del último pilar
    – Mantiene distancia lateral objetivo usando PD simple
    – Controla avance y ligera corrección de giro en Spike

Requisitos:
  • roboflowoak, depthai, cv2, lgpio
  • Puerto serie libre (/dev/ttyACM0)
  • Tres HC-SR04 en pines BCM 13,19,26 (izq,frente,der)
"""

import time
import threading
import queue
import sys
HEADLESS = ('--headless' in sys.argv or '-H' in sys.argv)  # Ejecutar con --headless para desactivar GUI
from dataclasses import dataclass
from typing import Optional

# --------------- Visión (Roboflow OAK) -----------------

from roboflowoak import RoboflowOak  # type: ignore
import cv2
import numpy as np

# Clases que pueden aparecer en Roboflow; extiende según tu dataset
authorized_classes = {
    "red": "RIGHT",      # Ej. "red"
    "redbox": "RIGHT",   # Ej. "redbox"
    "red_pillar": "RIGHT",
    "green": "LEFT",
    "greenbox": "LEFT",
    "green_pillar": "LEFT",
}

def class_to_lane(label:str) -> Optional[int]:
    """Devuelve -1 si es rojo, +1 si es verde, None si desconocido."""
    label_lower = label.lower()
    if label_lower in authorized_classes:
        return -1 if authorized_classes[label_lower] == "RIGHT" else 1
    # fallback: heurística por substring
    if "red" in label_lower:
        return -1
    if "green" in label_lower:
        return 1
    return None

@dataclass
class PillarObservation:
    color: str           # "red" | "green"
    distance_cm: float   # En centímetros
    center_offset: float # -1.0 izquierda, 0 centro, +1.0 derecha
    ts: float            # Marca de tiempo

VISION_CONFIDENCE = 0.45
VISION_OVERLAP   = 0.3
VISION_HAS_DEPTH = True   # Cámara OAK-D
MODEL_ID = "wro-round-2-mefmf"
MODEL_VERSION = "3"
PRIVATE_API_KEY = "1QaioZ6lSVket4h0AjJP"  # <-- Sustituir por clave privada real

vision_queue: "queue.Queue[PillarObservation]" = queue.Queue(maxsize=1)


def vision_worker():
    rf = RoboflowOak(
        model=MODEL_ID,
        version=MODEL_VERSION,
        api_key=PRIVATE_API_KEY,
        confidence=VISION_CONFIDENCE,
        overlap=VISION_OVERLAP,
        rgb=True,
        depth=VISION_HAS_DEPTH,
        blocking=True,
    )
    while running:
        try:
            result, frame_vis, _raw, depth_map = rf.detect()
            preds = result["predictions"]

            # Buscar la detección más cercana con clase válida
            best: Optional[PillarObservation] = None
            h_frame, w_frame = frame_vis.shape[:2] if frame_vis is not None else (0,0)
            for p in preds:
                # Sólo consideramos clases reconocidas, pero mostramos todo para debug
                if p.class_name not in authorized_classes:
                    print(f"[Vision] Clase desconocida {p.class_name} – ignorada")
                    continue
                dist_cm = None
                if VISION_HAS_DEPTH and depth_map is not None and depth_map.ndim == 2:
                    x0 = int(p.x - p.width/2)
                    y0 = int(p.y - p.height/2)
                    x1 = int(p.x + p.width/2)
                    y1 = int(p.y + p.height/2)
                    h, w = depth_map.shape
                    x0 = max(0, min(x0, w-1)); x1 = max(x0+1, min(x1, w))
                    y0 = max(0, min(y0, h-1)); y1 = max(y0+1, min(y1, h))
                    roi = depth_map[y0:y1, x0:x1]
                    valid = roi[(roi>0)&(roi<30000)]
                    if valid.size:
                        dist_cm = float(np.median(valid)/10.0)
                if dist_cm is None:
                    continue
                offset = (p.x - w_frame/2)/(w_frame/2) if w_frame else 0.0
                obs = PillarObservation(color=p.class_name, distance_cm=dist_cm, center_offset=offset, ts=time.time())
                if best is None or obs.distance_cm < best.distance_cm:
                    best = obs

            if best is not None:
                if vision_queue.full():
                    try:
                        vision_queue.get_nowait()
                    except queue.Empty:
                        pass
                vision_queue.put(best)

            # Visualización opcional
            if not HEADLESS:
                cv2.imshow("Vision", frame_vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        except Exception as e:
            print("[Vision] Error:", e)
            time.sleep(0.5)

    if not HEADLESS:
        cv2.destroyAllWindows()

# --------------- Ultrasónicos HC-SR04 -----------------

import lgpio
SPEED_OF_SOUND = 34300/1_000_000  # cm/us

@dataclass
class UltrasonicSensor:
    name: str
    pin: int
    last: Optional[float] = None
    last_ts: float = 0.0

SENSORS = [
    UltrasonicSensor("izq",   13),
    UltrasonicSensor("front", 19),
    UltrasonicSensor("der",   26),
]

MIN_INTERVAL = 0.04  # s
TIMEOUT = 0.06       # s

ultra_lock = threading.Lock()


def get_distance(h:int, pin:int) -> Optional[float]:
    try:
        lgpio.gpio_write(h, pin, 0)
        time.sleep(0.000002)
        lgpio.gpio_write(h, pin, 1)
        time.sleep(0.000010)
        lgpio.gpio_write(h, pin, 0)

        timeout = time.time() + TIMEOUT
        while lgpio.gpio_read(h, pin)==0:
            if time.time()>timeout:
                return None
        start = time.time()
        while lgpio.gpio_read(h, pin)==1:
            if time.time()>timeout:
                return None
        end = time.time()
        dist = (end-start)*SPEED_OF_SOUND*1e6/2  # cm
        return dist if 2<dist<400 else None
    except Exception:
        return None


def ultrasonic_worker(h:int, sensor:UltrasonicSensor):
    while running:
        now = time.time()
        if now - sensor.last_ts >= MIN_INTERVAL:
            d = get_distance(h, sensor.pin)
            if d is not None:
                sensor.last = d
                sensor.last_ts = now
        time.sleep(0.002)

# --------------- Spike Prime Serial Control -----------------

import serial, os, pty, subprocess

SPIKE_PORT = "/dev/ttyACM0"
SPIKE_BAUD = 115200

class Spike:
    def __init__(self, port=SPIKE_PORT, baud=SPIKE_BAUD):
        self.port=port; self.baud=baud; self.ser: Optional[serial.Serial] = None

    def connect(self):
        self._kill_screen()
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        self._upload_basics()
        print("[Spike] Conectado y funciones básicas cargadas.")

    def _kill_screen(self):
        """Intenta cerrar cualquier sesión *screen* u otro proceso que bloquee el puerto."""
        # 1) Mata procesos screen que incluyan el puerto en su línea de comando
        try:
            subprocess.call(["pkill", "-f", f"screen.*{self.port}"])
        except Exception:
            pass

        # 2) Espera un momento para que el SO libere el descriptor
        time.sleep(0.5)

        # 3) Método de respaldo (similar al script original del usuario)
        if not os.path.exists(self.port):
            return
        try:
            master_fd, slave_fd = pty.openpty()
            proc = subprocess.Popen(
                ["screen", self.port, str(self.baud)],
                stdin=slave_fd, stdout=slave_fd, stderr=slave_fd, close_fds=True
            )
            time.sleep(1.0)
            # Secuencia: Ctrl+C, luego Ctrl+A  k  y  para terminar la sesión screen
            os.write(master_fd, b"\x03")  # Ctrl+C
            time.sleep(0.1)
            os.write(master_fd, b"\x01")  # Ctrl+A
            time.sleep(0.1)
            os.write(master_fd, b"k")
            time.sleep(0.1)
            os.write(master_fd, b"y")
            time.sleep(0.1)
            proc.terminate()
            os.close(master_fd)
            os.close(slave_fd)
        except Exception:
            pass

    def send(self, cmd:str):
        if not self.ser: return
        self.ser.write(f"{cmd}\r".encode())
        self.ser.read_until()  # eco

    def _upload_basics(self):
        """Reutiliza initialize_Libraries() del script original para
        cargar todas las funciones de control que ya funcionan."""
        import importlib.util, os, types
        path = os.path.join(os.path.dirname(__file__), "spike_ultrasonic_twoPoints.py")
        spec = importlib.util.spec_from_file_location("spike_two", path)
        mod = importlib.util.module_from_spec(spec)  # type: ignore
        spec.loader.exec_module(mod)  # type: ignore
        # Sobrescribe el objeto 'spike' en el módulo con nuestro Serial ya abierto
        mod.spike = self.ser  # type: ignore
        # Ejecuta exactamente la misma rutina de carga de funciones
        if hasattr(mod, "initialize_Libraries"):
            mod.initialize_Libraries()  # type: ignore
        else:
            raise RuntimeError("initialize_Libraries no encontrada en spike_ultrasonic_twoPoints.py")

    def advance(self, v:float, ref:int):
        # usa función da(velocidad, referencia) del script original
        self.send(f"da({v},{ref})")

    def halt(self):
        self.send("fc()")

    def close(self):
        if self.ser and self.ser.is_open:
            self.halt()
            self.ser.close()
            print("[Spike] Desconectado.")

# --------------- Control Principal -----------------

def choose_lane(obs:PillarObservation) -> int:
    """Devuelve -1 si debe mantener derecha, +1 si izquierda."""
    lane = class_to_lane(obs.color)
    if lane is None:
        # valor por defecto: mantener derecha si no se reconoce
        lane = -1
    return lane

running = True


def main():
    global running
    # GPIO init
    try:
        h = lgpio.gpiochip_open(0)
        for s in SENSORS:
            lgpio.gpio_claim_output(h, s.pin)
        print("GPIO listo.")
    except Exception as e:
        print("GPIO error:", e); return

    # Spike init
    spike = Spike()
    try:
        spike.connect()
    except Exception as e:
        print("No se pudo conectar al Spike:", e); return

    # Threads ultrasónicos
    for s in SENSORS:
        threading.Thread(target=ultrasonic_worker, args=(h,s), daemon=True).start()

    # Thread visión
    threading.Thread(target=vision_worker, daemon=True).start()

    lane = 0  # 0 = no definido, -1 derecha, +1 izquierda
    target_side_dist = 18.0  # cm a mantener respecto pared
    base_speed = 0.5
    kp_side = 0.02

    try:
        while running:
            # Actualizar lane si hay nueva observación
            try:
                obs = vision_queue.get_nowait()
                lane = choose_lane(obs)
                print(f"\n[Visión] {obs.color.upper()} dist={obs.distance_cm:.0f}cm, offset={obs.center_offset:+.2f} → lane={'RIGHT' if lane==-1 else 'LEFT'}")
            except queue.Empty:
                pass

            # Tomar lecturas actuales de ultrasónicos
            with ultra_lock:
                left = next(s for s in SENSORS if s.name=="izq").last
                right = next(s for s in SENSORS if s.name=="der").last

            if lane==0:
                # sin información → avanzar recto
                spike.advance(base_speed, 0)
            else:
                side_dist = left if lane==1 else right
                if side_dist is None:
                    spike.advance(base_speed, obs.center_offset*10) if 'obs' in locals() else spike.advance(base_speed, 0)
                else:
                    error = (target_side_dist - side_dist) * lane  # signo correcto
                    # Mezcla corrección por distancia lateral y el offset de visión
                    ref_angle = error * kp_side + (obs.center_offset if 'obs' in locals() else 0)*5
                    ref_angle = max(-10, min(10, ref_angle))
                    spike.advance(base_speed, ref_angle)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nCancelado por usuario.")
    finally:
        running=False
        spike.close()
        lgpio.gpiochip_close(h)
        print("Finalizado.")

if __name__ == "__main__":
    main()
