import lzma
import json
import base64
import cv2
import numpy as np

def decompress_lzma_frame(lzma_file_path):

    with open(lzma_file_path, 'rb') as f:
        data = json.loads(lzma.decompress(f.read()))

    return data

def compress_and_save_lzma_frame(dictionary, file_path):

    with open(file_path, 'wb') as f:
        f.write(lzma.compress(json.dumps(dictionary).encode('utf-8')))

def decompress_image_from_base64(b64_string):

    img_bytes = base64.b64decode(b64_string)
    img_array = np.frombuffer(img_bytes, dtype=np.uint8)
    
    return cv2.imdecode(img_array, cv2.IMREAD_COLOR)