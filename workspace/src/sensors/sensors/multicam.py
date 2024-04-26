import cv2
import sys

def retrieve_camera_indexes():

    max_tested = 10
    num_cameras = 0
    usable_camera_indexes = []
    for i in range(max_tested):
        cap = cv2.VideoCapture(i)
        if cap is None or not cap.isOpened():
            cap.release()
            continue
        else:
            usable_camera_indexes.append(i)
        num_cameras += 1
        cap.release()
        
    return usable_camera_indexes

print("Available Cameras:", retrieve_camera_indexes())

cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,1080)

cap2 = cv2.VideoCapture(0)

while True:
    
    ret, frame = cap.read()
    if not ret:
        continue

    ret, frame2 = cap2.read()

    cv2.imshow('Webcam', frame)
    cv2.imshow('Webcam main', frame2)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()