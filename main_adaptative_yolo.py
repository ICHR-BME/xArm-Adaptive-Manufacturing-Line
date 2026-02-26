import numpy as np
import pandas as pd
import cv2
import time
import keyboard
import traceback
import sys
from PIL import Image
from xarm.wrapper import XArmAPI
from transformers import pipeline
from sklearn.linear_model import LinearRegression
from ultralytics import YOLO  # <--- IMPORTANTE: Librería para YOLO

# ================= CONFIGURACIÓN =================
# Ajusta esta ruta a donde tengas tu modelo entrenado
MODEL_PATH = r"C:\Users\xiarh\Downloads\detect\detect\train\weights\best.pt" 
# =================================================

class RobotMain(object):
    def __init__(self, robot_ip, csv_file_xy):
        self.alive = True
        self._arm = XArmAPI(robot_ip, baud_checkset=False)
        self._tcp_speed = 100
        self._tcp_acc = 2000
        
        # --- CALIBRACIÓN ---
        self.cal_x_m, self.cal_x_b = 0.491, 215.36
        self.cal_y_m, self.cal_y_b = 0.4795, 41.167
        self.swap_axes = True 

        # Valores Z (Fórmula calibrada con tus datos recientes)
        self.cal_z_m = -0.5778
        self.cal_z_b = 305.95

        # --- SEGURIDAD ---
        self.z_safe_travel = 300  # Altura segura para viajar

        self._robot_init()
        self._load_calibration_xy(csv_file_xy)

    def _robot_init(self):
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)
        time.sleep(1)
        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)

    def _load_calibration_xy(self, csv_file):
        try:
            df = pd.read_csv(csv_file)
            vals = df.values
            X_img, Y_img = vals[:, 0].reshape(-1, 1), vals[:, 1].reshape(-1, 1)
            X_rob, Y_rob = vals[:, 2], vals[:, 3]

            model_xx = LinearRegression().fit(X_img, X_rob)
            model_xy = LinearRegression().fit(Y_img, X_rob)
            
            if model_xy.score(Y_img, X_rob) > model_xx.score(X_img, X_rob):
                self.swap_axes = True
                self.cal_x_m = model_xy.coef_[0]
                self.cal_x_b = model_xy.intercept_
                model_yx = LinearRegression().fit(X_img, Y_rob)
                self.cal_y_m = model_yx.coef_[0]
                self.cal_y_b = model_yx.intercept_
            else:
                self.swap_axes = False
                self.cal_x_m = model_xx.coef_[0]
                self.cal_x_b = model_xx.intercept_
                model_yy = LinearRegression().fit(Y_img, Y_rob)
                self.cal_y_m = model_yy.coef_[0]
                self.cal_y_b = model_yy.intercept_
            print(" > Calibración XY cargada correctamente.")
        except Exception:
            print(" > Usando valores XY por defecto.")

    def _error_warn_changed_callback(self, data):
        if data and data["error_code"] != 0:
            self.alive = False
            print(f"[ERROR] Code: {data['error_code']}")

    def pick(self, x_pix, y_pix, z_real):
        # Calcular XY
        if self.swap_axes:
            x_robot = round(self.cal_x_m * y_pix + self.cal_x_b, 4)
            y_robot = round(self.cal_y_m * x_pix + self.cal_y_b, 4)
        else:
            x_robot = round(self.cal_x_m * x_pix + self.cal_x_b, 4)
            y_robot = round(self.cal_y_m * y_pix + self.cal_y_b, 4)
        
        z_target = z_real 

        try:
            print(f"🦾 PICK: ({x_robot}, {y_robot}, {z_target})")
            
            # 1. Subir (Seguridad)
            self._arm.set_position(z=self.z_safe_travel, speed=self._tcp_speed, wait=True)
            # 2. Viajar
            self._arm.set_position(*[x_robot, y_robot, self.z_safe_travel, -179.8, 0.0, 0.1], speed=self._tcp_speed, wait=True)
            # 3. Bajar (Lento)
            self._arm.set_position(*[x_robot, y_robot, z_target, -179.8, 0.0, 0.1], speed=50, wait=True)
            # 4. Agarrar
            time.sleep(0.5)
            self._arm.close_lite6_gripper()
            time.sleep(1)
            # 5. Salir
            self._arm.set_position(*[x_robot, y_robot, self.z_safe_travel, -179.8, 0.0, 0.1], speed=self._tcp_speed, wait=True)
            
        except Exception as e:
            print(f"Error en Pick: {e}")

    def place(self):
        try:
            x_home, y_home = 150, -200 
            self._arm.set_position(*[x_home, y_home, self.z_safe_travel, -179.8, 0.0, 0.1], speed=self._tcp_speed, wait=True)
            self._arm.set_position(*[x_home, y_home, 50, -179.8, 0.0, 0.1], speed=50, wait=True)
            self._arm.open_lite6_gripper()
            time.sleep(1)
            self._arm.set_position(*[x_home, y_home, self.z_safe_travel, -179.8, 0.0, 0.1], speed=self._tcp_speed, wait=True)
        except Exception:
            pass

# ==========================================
# FUNCIONES DE VISIÓN
# ==========================================

def get_top_view_data(cap):
    """Cámara TOP: Detecta contornos simples para XY."""
    ret, frame = cap.read()
    if not ret: return None, 0, 0, False
    
    # Recorte (Top View)
    x_crop, y_crop = 200, 0
    w_crop, h_crop = 300, 250
    if y_crop+h_crop <= frame.shape[0] and x_crop+w_crop <= frame.shape[1]:
        frame_roi = frame[y_crop:y_crop+h_crop, x_crop:x_crop+w_crop]
    else:
        frame_roi = frame
        x_crop, y_crop = 0, 0

    gray = cv2.cvtColor(frame_roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    large_contours = [c for c in contours if cv2.contourArea(c) > 400]
    cv2.drawContours(frame_roi, large_contours, -1, (0, 255, 0), 2)
    
    best_px, best_py = 0, 0
    found = False

    if large_contours:
        cnt = max(large_contours, key=cv2.contourArea)
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            best_px = cx + x_crop 
            best_py = cy + y_crop
            cv2.circle(frame, (best_px, best_py), 5, (0, 0, 255), -1)
            found = True

    return frame, best_px, best_py, found

def get_side_view_z_yolo(cap, model):
    """
    Cámara LATERAL (Con YOLO): Detecta objeto y calcula Z.
    """
    ret, frame = cap.read()
    if not ret: return None, 20

    # 1. Inferencia YOLO
    # conf=0.4 para filtrar detecciones débiles
    results = model(frame, conf=0.4, verbose=False)
    
    z_mm = 100 # Valor seguro por defecto
    detected = False

    for r in results:
        boxes = r.boxes
        if len(boxes) > 0:
            # Tomamos la caja con mayor confianza
            box = boxes[0] 
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            
            # Calculamos el CENTRO Y de la caja (igual que en la calibración)
            centro_y = (y1 + y2) / 2
            centro_x = (x1 + x2) / 2

            # --- FÓRMULA Z (Tus datos calibrados) ---
            z_mm = (-0.5778 * centro_y) + 305.95
            # ----------------------------------------

            # Limites de seguridad
            z_mm = max(40, min(z_mm, 230))
            detected = True
            
            # Dibujar caja y etiqueta
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, f"Z: {z_mm:.1f}mm", (int(x1), int(y1)-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            break # Solo procesamos el primer objeto encontrado
    
    if not detected:
        cv2.putText(frame, "Esperando pieza...", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    return frame, z_mm

# ==========================================
# MAIN LOOP
# ==========================================
if __name__ == "__main__":
    print(" Cargando YOLO y Conectando Robot...")
    
    # 1. Cargar YOLO
    try:
        yolo_model = YOLO(MODEL_PATH)
    except Exception as e:
        print(f" Error cargando YOLO: {e}")
        print("Verifica la ruta en MODEL_PATH")
        sys.exit()

    # 2. Iniciar Robot
    robot = RobotMain("192.168.0.184", csv_file_xy="datos_calibracion_exactos_4.csv")
    
    # 3. Iniciar Cámaras
    cap_top = cv2.VideoCapture(0)  # Cámara Superior
    cap_side = cv2.VideoCapture(1) # Cámara Lateral (Ajusta si es necesario)

    print("\n SISTEMA LISTO (YOLO MODE). Presiona 'K' para ejecutar.\n")

    while True:
        frame_top, px, py, found_top = get_top_view_data(cap_top)
        frame_side, z_real = get_side_view_z_yolo(cap_side, yolo_model)

        if frame_top is not None: cv2.imshow("Top View (XY)", frame_top)
        if frame_side is not None: cv2.imshow("Side View (YOLO Z)", frame_side)

        if keyboard.is_pressed('k') and found_top:
            print(f" Iniciando ciclo: Destino XY({px}, {py}) | Altura Z({z_real:.1f})")
            robot.pick(px, py, z_real)
            robot.place()
            time.sleep(1)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap_top.release()
    cap_side.release()
    cv2.destroyAllWindows()