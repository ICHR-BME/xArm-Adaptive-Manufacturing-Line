import numpy as np
import pandas as pd
import cv2
import time
import keyboard
import traceback
import sys
from xarm.wrapper import XArmAPI
from sklearn.linear_model import LinearRegression

# ==========================================
#  COORDENADAS DE ENTREGA (BASE GIRATORIA)
# ==========================================
SCANNER_X = 260.2
SCANNER_Y = -300.4
SCANNER_Z = 175.2   
SCANNER_ROLL  = -178.4
SCANNER_PITCH = -2.9
SCANNER_YAW   = -0.2
# ==========================================

# Altura segura cerca de la base del robot (Home temporal)
Z_HOME_SEGURO = 220.0
Z_OFFSET_AGARRE = 15.0

class RobotMain(object):
    def __init__(self, robot_ip, csv_file_xy):
        self.alive = True
        self._arm = XArmAPI(robot_ip, baud_checkset=False)
        self._tcp_speed = 100
        self._tcp_acc = 2000
        
        # Calibration defaults 
        self.cal_x_m, self.cal_x_b = 0.491, 215.36
        self.cal_y_m, self.cal_y_b = 0.4795, 41.167
        self.swap_axes = True 
        self.cal_z_m, self.cal_z_b = -0.5778, 305.95 

        self._robot_init()
        self._load_calibration_xy(csv_file_xy)

    def _robot_init(self):
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)
        time.sleep(1)
        self._arm.set_collision_sensitivity(3)

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
                self.cal_x_m, self.cal_x_b = model_xy.coef_[0], model_xy.intercept_
                model_yx = LinearRegression().fit(X_img, Y_rob)
                self.cal_y_m, self.cal_y_b = model_yx.coef_[0], model_yx.intercept_
            else:
                self.swap_axes = False
                self.cal_x_m, self.cal_x_b = model_xx.coef_[0], model_xx.intercept_
                model_yy = LinearRegression().fit(Y_img, Y_rob)
                self.cal_y_m, self.cal_y_b = model_yy.coef_[0], model_yy.intercept_
            print(" > Calibration loaded.")
        except:
            print(" > Default.")

    def recover_from_error(self):
        print("Recoveering from error...")
        self._arm.clean_error()
        self._arm.clean_warn()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)
        time.sleep(1)
        self._arm.set_position(z=20, relative=True, wait=True)

    def pick(self, x_pix, y_pix, z_vision):
        # 1. Calcular XY
        if self.swap_axes:
            x_robot = round(self.cal_x_m * y_pix + self.cal_x_b, 4)
            y_robot = round(self.cal_y_m * x_pix + self.cal_y_b, 4)
        else:
            x_robot = round(self.cal_x_m * x_pix + self.cal_x_b, 4)
            y_robot = round(self.cal_y_m * y_pix + self.cal_y_b, 4)
        
        # 2. Calcular Z
        z_target = z_vision + Z_OFFSET_AGARRE
        if z_target < 130: z_target = 130 

        # 3. Punto de RETRACCIÓN (Cerca de la base para ganar fuerza)
        # Esto es vital para que no falle al subir
        X_RETRACT = 180
        Y_RETRACT = 0

        try:
            print(f"🦾 PICK en ({x_robot}, {y_robot}, {z_target})")
            
            # A) Ir a posición PREVIA (Alta y Retraída)
            self._arm.set_position(x=X_RETRACT, y=Y_RETRACT, z=Z_HOME_SEGURO, 
                                   roll=-179.8, pitch=0, yaw=0.1, speed=self._tcp_speed, wait=True)

            # B) Viajar a la pieza (Desde arriba)
            self._arm.set_position(x=x_robot, y=y_robot, z=Z_HOME_SEGURO, 
                                   roll=-179.8, pitch=0, yaw=0.1, speed=self._tcp_speed, wait=True)

            # C) Bajar a la pieza
            self._arm.set_position(x=x_robot, y=y_robot, z=z_target, 
                                   roll=-179.8, pitch=0, yaw=0.1, speed=40, mvacc=1000, wait=True)
            
            # D) Agarrar
            time.sleep(0.5)
            self._arm.close_lite6_gripper()
            time.sleep(0.8)

            # E) >>> LA SOLUCIÓN <<< 
            # 1. Subir solo un poquito en VERTICAL (Relativo)
            print("Downloading...")
            self._arm.set_position(z=60, relative=True, speed=50, wait=True)
            
            # 2. RETRAER EL BRAZO HACIA LA BASE (Aquí es donde gana altura real)
            # Al traer el brazo al centro (X=180), ganamos capacidad para subir a Z=220
            print("Retracting to safe zone...")
            self._arm.set_position(x=X_RETRACT, y=Y_RETRACT, z=Z_HOME_SEGURO, 
                                   roll=-179.8, pitch=0, yaw=0.1, speed=80, wait=True)
            
        except Exception as e:
            print(f"Error Pick: {e}")
            self.recover_from_error()

    def place(self):
        try:
            print(f"Gping to the scanner")
            
            # Como ya estamos retraídos (en X=180, Y=0), el viaje al scanner es más fácil
            
            # 1. Viajar ALTO hacia Scanner
            self._arm.set_position(*[SCANNER_X, SCANNER_Y, Z_HOME_SEGURO, SCANNER_ROLL, SCANNER_PITCH, SCANNER_YAW], 
                                   speed=self._tcp_speed, wait=True)
            
            # 2. Bajar
            print(f"Placing...")
            self._arm.set_position(*[SCANNER_X, SCANNER_Y, SCANNER_Z, SCANNER_ROLL, SCANNER_PITCH, SCANNER_YAW], 
                                   speed=40, mvacc=1000, wait=True)
            
            # 3. Soltar
            time.sleep(0.5)
            self._arm.open_lite6_gripper()
            time.sleep(1)
            
            # 4. Salir (Subir de nuevo)
            self._arm.set_position(*[SCANNER_X, SCANNER_Y, Z_HOME_SEGURO, SCANNER_ROLL, SCANNER_PITCH, SCANNER_YAW], 
                                   speed=self._tcp_speed, wait=True)
            
            print("Cicle completed.")

        except Exception as e:
            print(f"Error Place: {e}")
            self.recover_from_error()

# ==========================================
# VISIÓN
# ==========================================
def get_top_view_data(cap):
    ret, frame = cap.read()
    if not ret: return None, 0, 0, False
    x_crop, y_crop, w_crop, h_crop = 200, 0, 300, 250
    if y_crop+h_crop <= frame.shape[0] and x_crop+w_crop <= frame.shape[1]:
        frame_roi = frame[y_crop:y_crop+h_crop, x_crop:x_crop+w_crop]
    else: frame_roi = frame; x_crop=0; y_crop=0

    gray = cv2.cvtColor(frame_roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, np.ones((5,5), np.uint8))
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    large_contours = [c for c in contours if cv2.contourArea(c) > 400]
    
    cv2.drawContours(frame_roi, large_contours, -1, (0, 255, 0), 2)
    best_px, best_py, found = 0, 0, False

    if large_contours:
        cnt = max(large_contours, key=cv2.contourArea)
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            best_px = int(M["m10"] / M["m00"]) + x_crop
            best_py = int(M["m01"] / M["m00"]) + y_crop
            cv2.circle(frame, (best_px, best_py), 5, (0, 0, 255), -1)
            found = True
    return frame, best_px, best_py, found

def get_side_view_z(cap):
    ret, frame = cap.read()
    if not ret: return None, 20
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([0, 100, 50]), np.array([10, 255, 255])) + \
           cv2.inRange(hsv, np.array([170, 100, 50]), np.array([180, 255, 255]))
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    z_mm = 130 
    if contours:
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) > 300:
            x, y, w, h = cv2.boundingRect(c)
            centro_y = y + h // 2
            z_mm = max(40, min((-0.5778 * centro_y) + 305.95, 220))
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 255, 0), 2)
            cv2.putText(frame, f"Z: {z_mm:.1f}", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return frame, z_mm

if __name__ == "__main__":
    # Usa tu IP
    robot = RobotMain("192.168.0.184", csv_file_xy="datos_calibracion_exactos_4.csv")
    cap_top = cv2.VideoCapture(1) 
    cap_side = cv2.VideoCapture(0) 

    print("\nSYSTEM v10 (RETRACT MODE). [K] to execute.\n")

    while True:
        frame_top, px, py, found = get_top_view_data(cap_top)
        frame_side, z_real = get_side_view_z(cap_side)

        if frame_top is not None: cv2.imshow("Top", frame_top)
        if frame_side is not None: cv2.imshow("Side", frame_side)

        if keyboard.is_pressed('k') and found:
            print(f"Action Initiated")
            robot.pick(px, py, z_real)
            robot.place()
            time.sleep(1)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap_top.release()
    cap_side.release()
    cv2.destroyAllWindows()