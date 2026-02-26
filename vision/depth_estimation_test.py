import numpy as np
from PIL import Image
import cv2
import matplotlib.pyplot as plt
from transformers import pipeline
import keyboard
from pynput.mouse import Controller

# --- Setup ---
mouse = Controller()
inicio = False
red = (0, 255, 0)
mpos = (0, 0) # Initialize mpos here to avoid the NameError

cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("Error al abrir la camara")
    exit()

# --- Initial Depth Capture ---
# We use a simple loop to wait for the user to press 'i'
print("Presiona 'i' para capturar la imagen de calibración...")
while not inicio:
    ret, frame = cap.read()
    cv2.imshow("Preview - Presiona 'i' para capturar", frame)
    if keyboard.is_pressed("i"):
        img = frame.copy()
        inicio = True
    if cv2.waitKey(1) & 0xFF == ord('q'):
        exit()

cv2.destroyAllWindows()

# --- Depth Estimation (Heavy Part) ---
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img_pil = Image.fromarray(img_rgb)
pipe = pipeline(task="depth-estimation", model="LiheYoung/depth-anything-large-hf")
resultado = pipe(img_pil)
depth = np.array(resultado["depth"])

# --- Main Video Loop ---
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 1. Update mouse position from the pynput Controller
    current_pos = mouse.position 
    # Note: These are screen coordinates. You may need to offset them 
    # to match the window position if you want precision.
    mpos = (int(current_pos[0]), int(current_pos[1]))

    # 2. Draw the circle (safety check to ensure mpos fits in the frame)
    # Using try/except or manual clamping is good practice in mechatronics
    try:
        cv2.circle(frame, (mpos[0] % frame.shape[1], mpos[1] % frame.shape[0]), 5, red, -1)
    except:
        pass

    # 3. Show the video using OpenCV (much faster for loops than Matplotlib)
    cv2.imshow("Video Feed", frame)
    
    # Show the depth map in a separate window
    # Normalize depth for visualization
    depth_viz = cv2.applyColorMap((depth * 255 / depth.max()).astype(np.uint8), cv2.COLORMAP_INFERNO)
    cv2.imshow("Depth Map", depth_viz)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()