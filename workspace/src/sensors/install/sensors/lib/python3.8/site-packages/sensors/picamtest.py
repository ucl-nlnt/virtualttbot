import picamera

try:
    with picamera.PiCamera() as picam:
        print("Camera exists")
except Exception as e:
    print(e)
    print("No Camera Detected.")