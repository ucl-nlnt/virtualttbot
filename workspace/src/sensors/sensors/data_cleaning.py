import os
import zlib
import json
from custom_types import DatalogType


class Cleaner:
    def __init__(self):
        self.datalogs_path = os.path.join(
            os.getcwd(), "./workspace/src/sensors/sensors/datalogs")

        self.viewDatalogs()

    def viewDatalogs(self):
        files = os.listdir(self.datalogs_path)
        # for file in files:
        file = files[0]
        file_path = os.path.join(self.datalogs_path, file)
        bytes = self.readFile(file_path)
        data: DatalogType = self.decompressBytes(bytes)
        states = data['states']
        state = states[0]
        print(state['frame_data'])

    def readFile(self, path):
        with open(path, 'rb') as f:
            compressed = f.read()
            return compressed

    def decompressBytes(self, bytes):
        decompressed = json.loads(zlib.decompress(bytes).decode('utf-8'))
        return decompressed


cleaner = Cleaner()
