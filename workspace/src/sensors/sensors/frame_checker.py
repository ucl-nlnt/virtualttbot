import lzma_interfacer
import os
import cv2

f_paths = os.listdir('datalogs')

for file in f_paths:

    data = lzma_interfacer.decompress_lzma_frame(os.path.join(os.getcwd(),'datalogs',file))

    raspi_image = None
    webcam_image = None

    for j,i in enumerate(data['states']):
        
        print(j)
        f_data = i['frame_data']
        if f_data != None:
            raspi_image = lzma_interfacer.decompress_image_from_base64(f_data)
        else:
            continue
        
        if 'webcam_data' in i.keys():
            webcam_image = lzma_interfacer.decompress_image_from_base64(i['webcam_data'])

        if raspi_image is not None:
            cv2.imshow("raspi", raspi_image)

        if webcam_image is not None:
            cv2.imshow("webcam", webcam_image)

        if cv2.waitKey(0) & 0xFF == ord(' '):
            continue  # Move to the next iteration of the loop

        # Optional: break the loop if another specific key (e.g., 'q') is pressed
        elif cv2.waitKey(0) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()