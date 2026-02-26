import cv2
import time
import csv
import numpy as np
from xarm.wrapper import XArmAPI
from sklearn.linear_model import LinearRegression

# ================= CONFIGURACIÓN =================
ROBOT_IP = "192.168.0.184"      
INDICE_CAMARA_LATERAL = 2  # Cambia a 0 o 2 si no detecta la cámara correcta

# LÍMITES DE SEGURIDAD (MM)
Z_MIN = 30    # Altura mínima (cerca de la cama)
Z_MAX = 200   # Altura máxima (arriba)
PASO_MM = 5   # Cada cuántos mm tomar una foto

# RANGO DE COLOR ROJO (HSV) - Ajusta si no detecta tu pieza
LOWER_RED1 = np.array([0, 100, 50])
UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([170, 100, 50])
UPPER_RED2 = np.array([180, 255, 255])

NOMBRE_ARCHIVO = "datos_calibracion_z.csv"

def detectar_pieza_roja(frame):
    """Retorna la coordenada Y del centro de la pieza roja."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Combinar las dos gamas de rojo (0-10 y 170-180)
    mask1 = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
    mask2 = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
    mask = mask1 + mask2
    
    # Limpiar ruido
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # Tomar el contorno más grande (la pieza)
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) > 500: # Filtro de tamaño mínimo
            x, y, w, h = cv2.boundingRect(c)
            centro_y = y + h // 2
            # Dibujar para visualización
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.circle(frame, (x + w//2, centro_y), 5, (255, 0, 0), -1)
            return centro_y, True
            
    return -1, False

def main():
    print(f"--- CALIBRACIÓN DE PROFUNDIDAD (EJE Z) ---")
    
    # 1. Conectar Robot
    arm = XArmAPI(ROBOT_IP)
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(1)
    
    # 2. Conectar Cámara
    cap = cv2.VideoCapture(INDICE_CAMARA_LATERAL)
    if not cap.isOpened():
        print(f"Error: No se pudo abrir la cámara {INDICE_CAMARA_LATERAL}")
        return

    # Preparar CSV
    with open(NOMBRE_ARCHIVO, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Pixel_Y", "Robot_Z"])

    print(f">>> INICIANDO BARRIDO Z: De {Z_MIN}mm a {Z_MAX}mm")
    
    # Mover a posición inicial segura (X, Y fijos, Z inicial)
    # Asumimos una posición X,Y donde la cámara vea la pieza. Ajusta estos 200, 0 si es necesario.
    arm.set_position(x=200, y=0, z=Z_MAX, roll=-179.8, pitch=0, yaw=0, speed=50, wait=True)
    time.sleep(1)

    datos_pixel = []
    datos_z = []

    # 3. Bucle de Escaneo (Bajando desde Z_MAX hasta Z_MIN)
    # Bajamos en lugar de subir para evitar chocar con la cama al inicio
    for z_actual in range(Z_MAX, Z_MIN, -PASO_MM):
        print(f"Moviendo a Z={z_actual}...", end="\r")
        
        # Mover Robot
        arm.set_position(z=z_actual, speed=50, wait=True)
        time.sleep(0.5) # Esperar a que se estabilice la imagen
        
        # Capturar y Detectar
        ret, frame = cap.read()
        if not ret: break
        
        pixel_y, detectado = detectar_pieza_roja(frame)
        
        # Visualización
        cv2.putText(frame, f"Z Robot: {z_actual}mm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        if detectado:
            cv2.putText(frame, f"Pixel Y: {pixel_y}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Guardar dato
            with open(NOMBRE_ARCHIVO, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([pixel_y, z_actual])
            
            datos_pixel.append(pixel_y)
            datos_z.append(z_actual)
        else:
            cv2.putText(frame, "NO DETECTADO", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("Calibracion Z", frame)
        if cv2.waitKey(100) == ord('q'):
            break

    # 4. Calcular Regresión Automática
    if len(datos_pixel) > 5:
        print("\n\n>>> CALCULANDO FÓRMULA...")
        X = np.array(datos_pixel).reshape(-1, 1) # Pixeles (Entrada)
        y = np.array(datos_z)                    # Altura Real (Salida)
        
        modelo = LinearRegression()
        modelo.fit(X, y)
        
        m = modelo.coef_[0] # Pendiente (mm por pixel)
        b = modelo.intercept_ # Offset (mm)
        
        print("="*40)
        print(f" RESULTADOS DE CALIBRACIÓN Z")
        print("="*40)
        print(f" Pendiente (mm/pixel): {m:.5f}")
        print(f" Offset (b):           {b:.5f}")
        print("-" * 40)
        print(f" FÓRMULA FINAL PARA TU CÓDIGO:")
        print(f" z_mm = ({m:.5f} * pixel_y) + {b:.5f}")
        print("="*40)
    else:
        print("\n⚠️ No se recolectaron suficientes datos para calcular la fórmula.")

    arm.disconnect()
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()