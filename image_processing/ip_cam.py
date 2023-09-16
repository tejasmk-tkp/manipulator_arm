import cv2
import numpy as np

#Stream Camera
cap = cv2.VideoCapture(2)

#Loop Frames
while True:
    ret, frame = cap.read()
    if not ret:
        print("No more frames")
        break
    cv2.imshow('Capture', frame)
    if cv2.waitKey(1) == ord('q'):
        break

#Close Window
cap.release()
cv2.destroyAllWindows()