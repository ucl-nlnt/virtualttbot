import lzma_interfacer
import os

paths = [os.path.join(os.getcwd(), 'datalogs', i) for i in os.listdir('datalogs')]

for path in paths:

    data = lzma_interfacer.decompress_lzma_frame(path)
    
    start = 0
    
    for i in data['states']:
        print(i['id'])
        if i['id'] < start:
            print(f'out-of-order frame detected, {i["id"]} | {start}')
        else:
            start = i['id']

    start = 0