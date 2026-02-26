import cv2

cap=cv2.VideoCapture(0)
x=200
y=0
ancho=300
alto=250
rojo=(0,0,255)
while True:
    ret,frame=cap.read()
    if not ret:
        print("No frame")
        break
    img=frame[y:y+alto,x:x+ancho]
    cv2.circle(img,(66,91), 5, rojo,1)
    cv2.imshow("camara en vivo", img)

    if cv2.waitKey(1) & 0xFF==ord("q"):
        break
cap.release()
cv2.destroyAllWindows()
