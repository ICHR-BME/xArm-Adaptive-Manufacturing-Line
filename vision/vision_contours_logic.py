import numpy as np
import snap7
from PIL import Image, ImageTk
import customtkinter as ctk
from colordict import ColorDict
import cv2
import matplotlib.pyplot as plt
from transformers import pipeline
import math
import sys
import time
import threading
import keyboard
import traceback
from xarm.wrapper import XArmAPI

#path="C:/Users/Fernanda De la Cruz/Downloads/Verano (3)/Verano/prueba_profundidad.jpg"

class RobotMain(object):
    """Robot Main Class"""
    def __init__(self, robot, **kwargs):
        self.alive = True
        self._arm = robot
        self._ignore_exit_state = False
        self._tcp_speed = 100
        self._tcp_acc = 2000
        self._angle_speed = 20
        self._angle_acc = 500
        self._vars = {}
        self._funcs = {}
        self._robot_init()

    # Robot init
    def _robot_init(self):
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)
        time.sleep(1)
        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.register_state_changed_callback(self._state_changed_callback)

    # Register error/warn changed callback
    def _error_warn_changed_callback(self, data):
        if data and data['error_code'] != 0:
            self.alive = False
            self.pprint('err={}, quit'.format(data['error_code']))
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

    # Register state changed callback
    def _state_changed_callback(self, data):
        if not self._ignore_exit_state and data and data['state'] == 4:
            self.alive = False
            self.pprint('state=4, quit')
            self._arm.release_state_changed_callback(self._state_changed_callback)

    def _check_code(self, code, label):
        if not self.is_alive or code != 0:
            self.alive = False
            ret1 = self._arm.get_state()
            ret2 = self._arm.get_err_warn_code()
            self.pprint('{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}'.format(label, code, self._arm.connected, self._arm.state, self._arm.error_code, ret1, ret2))
        return self.is_alive

    @staticmethod
    def pprint(*args, **kwargs):
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1], ' '.join(map(str, args))))
        except:
            print(*args, **kwargs)

    @property
    def arm(self):
        return self._arm

    @property
    def VARS(self):
        return self._vars

    @property
    def FUNCS(self):
        return self._funcs

    @property
    def is_alive(self):
        if self.alive and self._arm.connected and self._arm.error_code == 0:
            if self._ignore_exit_state:
                return True
            if self._arm.state == 5:
                cnt = 0
                while self._arm.state == 5 and cnt < 5:
                    cnt += 1
                    time.sleep(0.1)
            return self._arm.state < 4
        else:
            return False

    # Robot Main Run
    def run(self,c):
        try:
            print("corriendo")
            orden = ["Square","Square_Base","Hexagon","Hexagon_Base"]                #orden = [ "Circle","Circle_Base","Square","Star"]
            for etiqueta_prioritaria in orden:
                print(c)
                for pieza in piezas:
                    if pieza['etiqueta'] == etiqueta_prioritaria:
                        print("voy por: ", pieza['etiqueta'])
                        if c%2 == 0:
                            print("pick")
                            for j in tamaños:
                                if j['etiqueta'] == etiqueta_prioritaria:
                                    cz = int(j['tamaño']) +85
                            x_px, y_px = pieza['centroide']          # orden correcto: (x, y)
                            x = 0.3314*y_px + 206.31                  # usa SIEMPRE los píxeles originales
                            y = 9.5766*x_px - 7.2599

                            print(pieza['etiqueta'],x,y,cz)
                            code = self._arm.set_position(*[x, y, 200.0, -179.8, 0.0, 0.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                            if not self._check_code(code, 'set_position'):
                                return
                            code = self._arm.set_position(*[x, y, cz, -179.8, 0.0, 0.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                            if not self._check_code(code, 'set_position'):
                                return
                            time.sleep(2)
                            code = self._arm.close_lite6_gripper()
                            if not self._check_code(code, 'open_lite6_gripper'):
                                        return
                            time.sleep(3)
                            code = self._arm.set_position(*[x, y, 200.0, -179.8, 0.0, 0.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                            if not self._check_code(code, 'set_position'):
                                    return
                            code = self._arm.set_position(*[197, 2, 200.0, -179.8, 0.0, 0.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                            if not self._check_code(code, 'set_position'):
                                    return
                            time.sleep(5)
                            c += 1
                            
                        else:
                            print("place")
                            cz = alt_max +85+10
                            x_px, y_px = pieza['centroide']          # orden correcto: (x, y)
                            x = 0.3314*y_px + 206.31                  # usa SIEMPRE los píxeles originales
                            y = 9.5766*x_px - 7.2599
                            print(pieza['etiqueta'],x,y,cz)
                            code = self._arm.set_position(*[x, y, 200.0, -179.8, 0.0, 0.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                            if not self._check_code(code, 'set_position'):
                                return
                            code = self._arm.set_position(*[x, y, cz, -179.8, 0.0, 0.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                            if not self._check_code(code, 'set_position'):
                                    return
                            time.sleep(2)
                            code = self._arm.open_lite6_gripper()
                            if not self._check_code(code, 'open_lite6_gripper'):
                                            return
                            time.sleep(2)
                            code = self._arm.set_position(*[x, y, 200.0, -179.8, 0.0, 0.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                            if not self._check_code(code, 'set_position'):
                                        return
                            code = self._arm.set_position(*[197, 2, 200.0, -179.8, 0.0, 0.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                            if not self._check_code(code, 'set_position'):
                                        return
                            time.sleep(5)
                            c += 1
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        finally:
            self.alive = True
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
            self._arm.release_state_changed_callback(self._state_changed_callback)
    def pick(self,x,y,cz):
        try:           
            x_px, y_px = x, y
            x = 0.3314*y_px + 206.31
            y = 9.5766*x_px - 7.2599

            #print(pieza['etiqueta'],x,y,cz)
            code = self._arm.set_position(*[x, y, 200.0, -179.8, 0.0, 0.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[x, y, cz, -179.8, 0.0, 0.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.close_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                        return
            time.sleep(3)
            code = self._arm.set_position(*[x, y, 200.0, -179.8, 0.0, 0.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                    return
            code = self._arm.set_position(*[197, -211, 200.0, -179.8, 0.0, 0.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                    return
            time.sleep(5) 
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        finally:
            self.alive = True
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
            self._arm.release_state_changed_callback(self._state_changed_callback)

    # --- Rutina preestablecida (ajusta a tu celda) ---
    PRESET_A = (275.1, 209.4, 205.1)   # punto 1: aproximación (alto)
    PRESET_B = (275.1, 209.4, 180.8)   # punto 2: mismo XY, baja para agarrar
    PRESET_C = (275.1, 209.4, 205.1)   # punto 3: destino (alto)
    PRESET_D = (275.1, 209.4, 205.1)   # punto 3: destino (alto)
    PRESET_E = (180.8, -324.4, 244.9)   # punto 3: destino (alto)
    PRESET_F = (180.8, -324.4, 135.3)   # punto 3: destino (alto)
    PRESET_G = (198.3, 1.7, 200.6)   # punto 3: destino (alto)

    def go_preset_routine(self):
        """Ir a A, abrir; bajar a B, cerrar; mover a C y D."""
        try:
            # 1) Ir a A (alto) y abrir gripper
            x, y, z = self.PRESET_A
            code = self._arm.set_position(*[x, y, z, -179.8, 0.0, 0.1],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'preset: move A'): return
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'preset: open'): return
            time.sleep(0.3)

            # 2) Bajar a B y cerrar gripper
            x, y, z = self.PRESET_B
            code = self._arm.set_position(*[x, y, z, -179.8, 0.0, 0.1],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'preset: move B'): return
            code = self._arm.close_lite6_gripper()
            if not self._check_code(code, 'preset: close'): return
            time.sleep(0.3)

            # 3) Mover a C (alto)
            x, y, z = self.PRESET_C
            code = self._arm.set_position(*[x, y, z, -179.8, 0.0, 0.1],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'preset: move C'): return

            # 4) Mover a D (alto)
            x, y, z = self.PRESET_D
            code = self._arm.set_position(*[x, y, z, -179.8, 0.0, 0.1],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'preset: move D'): return

            # 4) Mover a D (alto)
            x, y, z = self.PRESET_E
            code = self._arm.set_position(*[x, y, z, -179.8, 0.0, 0.1],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'preset: move E'): return

            x, y, z = self.PRESET_F
            code = self._arm.set_position(*[x, y, z, -179.8, 0.0, 0.1],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'preset: move F'): return
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'preset: open'): return
            time.sleep(0.3)

            x, y, z = self.PRESET_G
            code = self._arm.set_position(*[x, y, z, -179.8, 0.0, 0.1],
                                      speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'preset: move E'): return

      


        except Exception as e:
            self.pprint('MainException: {}'.format(e))

    def place(self, x, y, cz):
        try:
            x_px, y_px = x, y
            x = 0.3314*y_px + 206.31
            y = 9.5766*x_px - 7.2599

            code = self._arm.set_position(*[x, y, 200.0, -179.8, 0.0, 0.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[x, y, cz, -179.8, 0.0, 0.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[x, y, 200.0, -179.8, 0.0, 0.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[197, -211, 200.0, -179.8, 0.0, 0.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(5)
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        finally:
            self.alive = True
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
            self._arm.release_state_changed_callback(self._state_changed_callback)



def recortar_imagen(x,y,ancho,alto):
    global inicio
    cap=cv2.VideoCapture(1)
    ret,frame=cap.read()
    if not cap.isOpened():
         print("Error al abrir la cámara")
         exit()
    if not inicio or keyboard.is_pressed("i"):
        if not ret:
            print("Error al capturar la imagen")
            exit()
        cv2.imwrite("foto_impresora.jpg", frame)
        img=cv2.imread("foto_impresora.jpg")
        menos_brillo=cv2.subtract(img, np.array([50.0, 50.0, 50.0], dtype=np.uint8))
        if img is None:
            raise FileNotFoundError("No se pudo cargar la imagen")
        
        inicio=True
    return menos_brillo[y:y+alto,x:x+ancho]

def estimar_profundidad(img):
    img_rgb=cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    img_pil=Image.fromarray(img_rgb)
    pipe=pipeline(task="depth-estimation",model="LiheYoung/depth-anything-large-hf")
    resultado=pipe(img_pil)
    return np.array(resultado["depth"])

def mostrar_profundidad(img,depth):
    fig,axes=plt.subplots(1,2,figsize=(12,6))
    axes[0].imshow(cv2.cvtColor(img,cv2.COLOR_BGR2RGB))
    axes[0].set_title("Imagen original")
    axes[0].axis("off")

    axes[1].imshow(depth,cmap="inferno")
    axes[1].set_title("Estimacion de profundidad")
    axes[1].axis("off")

    plt.tight_layout()
    plt.show()

def detectar_contornos(img, depth):
    profundidades = []
    puntos_medios = []
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray2=cv2.imshow("Gray scale", gray)
    blur = cv2.GaussianBlur(gray, (9, 9), 0)
    blur2=cv2.imshow("Blurred image", blur)
    edges = cv2.Canny(blur, 0, 250)
    edges2=cv2.imshow("Canny edges", edges)
    kernel = np.ones((5, 5), np.uint8)
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    closed2=cv2.imshow("Canny closed", closed)

    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_area = 80
    large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
    contorno_img = img.copy()
    cv2.drawContours(contorno_img, large_contours, -1, (0,255,0), 3)
    cv2.imshow("Contornos filtrados", contorno_img)

    umbral_pendiente = 0.1
    min_puntos = 1

    for i, cnt in enumerate(large_contours):
        cnt = cnt.reshape(-1, 2)  # Asegurar forma (n, 2)
        if len(cnt) < 2:
            continue

        pendientes = []
        puntos = cnt

        for j in range(len(puntos) - 1):
            x1, y1 = puntos[j]
            x2, y2 = puntos[j + 1]
            if x2 - x1 != 0:
                m = (y2 - y1) / (x2 - x1)
            else:
                m = float('inf')
            pendientes.append(m)

        # Agrupar segmentos con pendientes similares
        segmentos = []
        grupo_actual = [0]
        for j in range(1, len(pendientes)):
            if abs(pendientes[j] - pendientes[grupo_actual[-1]]) < umbral_pendiente:
                grupo_actual.append(j)
            else:
                if len(grupo_actual) >= min_puntos:
                    segmentos.append(grupo_actual.copy())
                grupo_actual = [j]
        if len(grupo_actual) >= min_puntos:
            segmentos.append(grupo_actual)

        # Dibujar los segmentos detectados y guardar puntos medios y profundidades
        for grupo in segmentos:
            idx_ini = grupo[0]
            idx_fin = grupo[-1] + 1
            if idx_fin >= len(puntos):  # evitar overflow
                continue

            p_ini = tuple(puntos[idx_ini])
            p_fin = tuple(puntos[idx_fin])

            midx = int((p_ini[0] + p_fin[0]) / 2)
            midy = int((p_ini[1] + p_fin[1]) / 2)

            if 0 <= midx < depth.shape[1] and 0 <= midy < depth.shape[0]:
                profundidad = depth[midy, midx]
            else:
                profundidad = None

            cv2.line(contorno_img, p_ini, p_fin, (255, 0, 0), 2)
            cv2.circle(contorno_img, (midx, midy), 4, (0, 255, 255), -1)

            puntos_medios.append((midx, midy))
            profundidades.append(profundidad)

            print(f"Contorno {i} | Línea de {len(grupo)} puntos | Pendiente: {pendientes[grupo[0]]:.2f} | Profundidad: {profundidad}")

    cv2.imshow("Contornos y líneas detectadas", contorno_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return profundidades, puntos_medios

inicio=False
tecla=False
contador=0
c=0
x,y,ancho,alto=120,9,439,350 





if __name__=="__main__":
    robot_main=RobotMain(XArmAPI('192.168.0.150', baud_checkset=False))

    img=recortar_imagen(x,y,ancho,alto)
    
    # Iniciar rutina preestablecida al tomar la foto
    


    depth=estimar_profundidad(img)
    mostrar_profundidad(img, depth)
    profundidades,puntos_medios=detectar_contornos(img, depth)
    print(f"Profundidades:{profundidades}")
    print("Puntos medios:",puntos_medios)
    
    for idx, (x_f, y_f) in enumerate(puntos_medios):
        profundidad = profundidades[idx]
        x_place = 0
        y_place = 0
        z_place = 95

    if profundidad is None or not isinstance(profundidad, (int, float)):
        profundidad = z_place  # valor por defecto

    print(f"Coordenadas: ({x_f}, {y_f}), Profundidad: {profundidad}")
     #robot_main.pick(x_f, y_f, float(profundidad))
    profundidad = float(profundidad) + 10
    robot_main.go_preset_routine()
     #robot_main.place(x_f, y_f, profundidad)
   # robot_main.pick(x_place, y_place, z_place)
   
