import cv2

cap = cv2.VideoCapture(0)

if cap.isOpened():
    print('yay')

else:
    print('aw')

cap.release()