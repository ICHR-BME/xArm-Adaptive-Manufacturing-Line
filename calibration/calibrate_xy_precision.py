import cv2
import time
import random
import csv
import numpy as np
from xarm.wrapper import XArmAPI
from ultralytics import YOLO

# ================= CONFIGURACIÓN =================
ROBOT_IP = "192.168.0.184"      
MODEL_PATH = r"C:\Users\xiarh\Downloads\detect\detect\train\weights\best.pt"

# ZONA DE VISIÓN
X_CROP, Y_CROP = 200, 0                                                  
ANCHO, ALTO = 300, 250                                            

# PARÁMETROS
TOTAL_PUNTOS = 100      
TOLERANCIA_PIXELES = 0  
VELOCIDAD_MM_S = 50    
PASO_MAXIMO_MM = 2.0    
PASO_MINIMO_MM = 0.1    

NOMBRE_ARCHIVO = "datos_calibracion_exactos_4.csv"

def main():
    print(f"--- CALIBRACIÓN V4 (FIX VISUAL) ---")
   
    with open(NOMBRE_ARCHIVO, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Pixel_Target_X", "Pixel_Target_Y", "Robot_X", "Robot_Y"])

    arm = XArmAPI(ROBOT_IP)
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(1)

    model = YOLO(MODEL_PATH)
    cap = cv2.VideoCapture(0)

    print(">>> ROBOT LISTO. SOLUCIÓN: YOLO ve antes de dibujar el punto rojo.")
    time.sleep(3)

    puntos_guardados = 0

    while puntos_guardados < TOTAL_PUNTOS:
        target_x = random.randint(30, ANCHO - 30)
        target_y = random.randint(30, ALTO - 30)
       
        print(f"\n>>> PUNTO {puntos_guardados + 1}/{TOTAL_PUNTOS} -> Meta: ({target_x}, {target_y})")
       
        intentos = 0
        alineado = False
       
        while not alineado:
            ret, frame = cap.read()
            if not ret: break

            # Recorte
            img = frame[Y_CROP:Y_CROP+ALTO, X_CROP:X_CROP+ANCHO].copy()

            # --- CAMBIO IMPORTANTE AQUI ---
            # 1. PRIMERO detectamos (la imagen está limpia, sin punto rojo)
            results = model(img, conf=0.3, verbose=False)
           
            screw_x, screw_y = -1, -1
            detectado = False

            for r in results:
                boxes = r.boxes
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0]
                    screw_x = int((x1 + x2) / 2)
                    screw_y = int((y1 + y2) / 2)
                    detectado = True
                    break

            # 2. DESPUÉS dibujamos los marcadores (solo para tus ojos)
            # Meta (Rojo)
            cv2.circle(img, (target_x, target_y), 2, (0, 0, 255), -1)
           
            # Tornillo (Verde) si se encontró
            if detectado:
                cv2.circle(img, (screw_x, screw_y), 2, (0, 255, 0), -1)

            # --- FIN DEL CAMBIO ---

            # Lógica de Control (Exactamente igual a la que te funcionó)
            if detectado:
                error_x = screw_x - target_x
                error_y = screw_y - target_y
               
                if error_x == 0 and error_y == 0:
                    alineado = True
                    print("   🎯 ¡CERO ABSOLUTO! Guardando...")
                   
                    code, pos = arm.get_position()
                    if code == 0:
                        with open(NOMBRE_ARCHIVO, mode='a', newline='') as file:
                            writer = csv.writer(file)
                            writer.writerow([target_x, target_y, pos[0], pos[1]])
                       
                        puntos_guardados += 1
                        cv2.rectangle(img, (0,0), (ANCHO, ALTO), (0,255,0), 3)
                        cv2.imshow("Calibracion V4", img)
                        cv2.waitKey(200)
                    else:
                        print("Error leyendo robot")

                else:
                    intentos += 1
                    if intentos > 300: # Subí un poco los intentos por si acaso
                        print("⚠️ Demasiada oscilación. Saltando...")
                        break

                    distancia = max(abs(error_x), abs(error_y))
                   
                    if distancia > 15:
                        paso = PASO_MAXIMO_MM
                    else:
                        paso = PASO_MINIMO_MM

                    move_x = 0
                    move_y = 0

                    if error_x != 0:
                        move_y = -paso if error_x > 0 else paso
                       
                    if error_y != 0:
                        move_x = -paso if error_y > 0 else paso

                    arm.set_position(x=move_x, y=move_y, relative=True, speed=VELOCIDAD_MM_S, wait=True)
           
            else:
                # Si no detecta (porque tapaste la camara o algo), avisa
                cv2.putText(img, "NO VEO EL TORNILLO", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

            cv2.imshow("Calibracion V4", img)
            if cv2.waitKey(1) == ord('q'):
                arm.disconnect()
                return

    arm.disconnect()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()