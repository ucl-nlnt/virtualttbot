import os
import zlib
import json
from custom_types import DatalogType, DatalogsList, FloatList, StatesList
import time
import numpy as np
import matplotlib.pyplot as plt


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

    def findFirstTwist(self, states: StatesList):
        for index, state in enumerate(states):
            linear_sum = sum(state["twist"]["linear"])
            angular_sum = sum(state["twist"]["angular"])
            if linear_sum != 0 or angular_sum != 0:
                return index
        return 0

    def findLastTwist(self, states: StatesList):
        for index in range(len(states) - 1, -1, -1):  # Iterate in reverse order
            state = states[index]
            linear_sum = sum(state["twist"]["linear"])
            angular_sum = sum(state["twist"]["angular"])
            if linear_sum != 0 or angular_sum != 0:
                return index
        return len(states)-1

    def trimDatalog(self, datalog: DatalogType, padding=3):
        """
        remove unwanted datalogs at the start and end of datalog stream
        """

        # find first instance with a non-zero linear or angular twist
        firstTwistIndex = self.findFirstTwist(datalog['states'])
        firstIndex = max(firstTwistIndex - padding, 0)

        # find last instance with a non-zero linear or angular twist
        lastTwistIndex = self.findLastTwist(datalog['states'])
        lastIndex = min(lastTwistIndex - padding, len(datalog['states'])-1)

        datalog['states'] = datalog['states'][firstIndex:lastIndex]

        return datalog

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

    def plotDatalog(self, datalog: DatalogType):

        length = len(datalog['states'])
        x = np.linspace(0, 2*np.pi, length)

        # plot imu linear acceleration x,y,z
        fig, (ax1, ax2) = plt.subplots(2)
        fig.suptitle(
            f"IMU: {datalog['natural_language_prompt']}")
        ax1.set_title("linear acceleration")
        linear_acce_x = [i['imu']['linear_acceleration'][0]
                         for i in datalog['states']]
        linear_acce_z = [i['imu']['linear_acceleration'][1]
                         for i in datalog['states']]
        linear_acce_y = [i['imu']['linear_acceleration'][2]
                         for i in datalog['states']]
        ax1.plot(x, linear_acce_x, label="x")
        ax1.plot(x, linear_acce_z, label="z")
        ax1.plot(x, linear_acce_y, label="y")

        ax2.set_title("angular velocity")
        angular_velo_x = [i['imu']['angular_velocity'][0]
                          for i in datalog['states']]
        angular_velo_z = [i['imu']['angular_velocity'][1]
                          for i in datalog['states']]
        angular_velo_y = [i['imu']['angular_velocity'][2]
                          for i in datalog['states']]
        ax2.plot(x, angular_velo_x, label="x")
        ax2.plot(x, angular_velo_z, label="z")
        ax2.plot(x, angular_velo_y, label="y")

        ax1.legend()
        ax2.legend()

        # plot twist linear x,y,z
        fig, (ax1, ax2) = plt.subplots(2)
        fig.suptitle(
            f"Twist: {datalog['natural_language_prompt']}")

        ax1.set_title("linear twist")
        twist_linear_x = [i['twist']['linear'][0]
                          for i in datalog['states']]
        twist_linear_z = [i['twist']['linear'][1]
                          for i in datalog['states']]
        twist_linear_y = [i['twist']['linear'][2]
                          for i in datalog['states']]
        ax1.plot(x, twist_linear_x, label="x")
        ax1.plot(x, twist_linear_z, label="z")
        ax1.plot(x, twist_linear_y, label="y")

        ax2.set_title("angular twist")
        twist_angular_x = [i['twist']['angular'][0]
                           for i in datalog['states']]
        twist_angular_z = [i['twist']['angular'][1]
                           for i in datalog['states']]
        twist_angular_y = [i['twist']['angular'][2]
                           for i in datalog['states']]
        ax2.plot(x, twist_angular_x, label="x")
        ax2.plot(x, twist_angular_z, label="z")
        ax2.plot(x, twist_angular_y, label="y")

        ax1.legend()
        ax2.legend()

        plt.show(block=False)

    def decompressBytes(self, bytes):
        """
        decompresses the contents of a file
        """
        decompressed = json.loads(zlib.decompress(bytes).decode('utf-8'))
        return decompressed


cleaner = Cleaner(include_images=False)

datalog = cleaner.datalogs[0]
cleaner.plotDatalog(datalog=datalog)
trimmed_datalog = cleaner.trimDatalog(datalog)
cleaner.plotDatalog(datalog=trimmed_datalog)


# this is a hack to make matplotlib work, do not remove this
plt.show()

# save datalogs to a json file
# cleaner.saveDatalogs()
