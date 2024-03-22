import os
import zlib
import json


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
        print(file, file_path)
        bytes = self.readFile(file_path)
        data = self.decompressBytes(bytes)
        print(json.dumps(data, separators=(',', ':')), )

    def readFile(self, path):
        with open(path, 'rb') as f:
            compressed = f.read()
            return compressed

    def decompressBytes(self, bytes):
        decompressed = zlib.decompress(bytes).decode('utf-8')
        return decompressed


cleaner = Cleaner()
