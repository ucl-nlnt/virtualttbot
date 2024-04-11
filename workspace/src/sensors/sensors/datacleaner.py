import os

currpath = os.path.join(os.getcwd(),'datalogs')
file_list = os.listdir(currpath)

for i in file_list:
    print(i)


def load_and_decompress(filename:str):

    fname = os.path.join(os.getcwd(),'datalogs',filename)
    with open(fname,'rb') as f:
        print(f.decode())

load_and_decompress(file_list[0])

linear_x_ms
angular_z_rads