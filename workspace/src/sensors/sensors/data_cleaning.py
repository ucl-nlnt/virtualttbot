import os
import zlib
import json
from custom_types import DatalogType, DatalogsList, FloatList
import time
import numpy as np


class Cleaner:
    def __init__(self, include_images=False, include_laserscanrawdata=False):
        self.datalogs_path = os.path.join(
            os.getcwd(), "./workspace/src/sensors/sensors/datalogs")

        self.include_images = include_images
        self.include_laserscanrawdata = include_laserscanrawdata
        self.datalogs = self.viewDatalogs()

    def moving_average(data: FloatList, window_size: int):
        weights = np.repeat(1.0, window_size) / window_size
        return np.convolve(data, weights, 'valid')

    def viewDatalogs(self):
        """
        compiles all data into a single dictionary
        """
        files = os.listdir(self.datalogs_path)
        files = [files[0]]
        datalogs: DatalogsList = []
        for file in files:
            if (file.endswith(".compressed")):
                file_path = os.path.join(self.datalogs_path, file)
                bytes = self.readFile(file_path)
                data: DatalogType = self.decompressBytes(bytes)
                if (self.include_images == False):
                    for state in data['states']:
                        state['frame_data'] = ''

                if (self.include_laserscanrawdata == False):
                    for state in data['states']:
                        state['laser_scan']['ranges'] = []
                        state['laser_scan']['intensities'] = []

                datalogs.append(data)

        return datalogs

    def saveDatalogs(self, compress=False):
        """
        saves datalogs into a single compressed or uncompressed json file
        """
        if len(self.datalogs) > 0:
            data = {"datalogs": self.datalogs}
            file_name = f'{int(time.time())}.json'
            file_path = os.path.join(self.datalogs_path, file_name)
            with open(file_path, "w") as json_file:
                json.dump(data, json_file)

    def trimDatalogs(self, threshold=0.01):
        """
        remove unwanted datalogs at the start and end of datalog stream
        """

        pass

    def cleanDatalogs(self):
        """
        clean unwanted datalogs from the datalog stream
        """
        pass

    def readFile(self, path):
        """
        returns the raw contents of a file
        """
        with open(path, 'rb') as f:
            compressed = f.read()
            return compressed

    def decompressBytes(self, bytes):
        """
        decompresses the contents of a file
        """
        decompressed = json.loads(zlib.decompress(bytes).decode('utf-8'))
        return decompressed


cleaner = Cleaner(include_images=False)

# save datalogs to a json file
cleaner.saveDatalogs()
