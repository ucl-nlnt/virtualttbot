import socket
import threading

# Server code
# Receive code through socket in subprocess

server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_sock.bind(('0.0.0.0', 50000))
server_sock.listen(0)
print('Server Listening on port 50000')

client_sock, addr = server_sock.accept()
print('Connected received from: ', addr)

while True:

    # Algorithm:
    # 1.) Send data size
    # 2.) Wait for data of size
    # 3.) Repeat

    data_size = client_sock.recv(8)
    if not data_size:
        continue

    # data_size = 8-int size, max of 99999999 bytes
    data_size = data_size.decode()
    if data_size[0] == '@':
        data_size = data_size[1:] # remove @

        if 'X' in data_size:
            data_size = int(data_size[:data_size.index('X')])
        else:
            data_size = int(data_size)

    else:
        continue

    client_sock.send(b'OK GO')
    print('reply sent')

    data_string = b''
    print(data_size)
    while (data_size):
        if data_size > 1024: # send in 1024-byte chunks
            tempt = client_sock.recv(1024)
            print(tempt)
            data_string += tempt
            data_size -= 1024
        else:
            tempt = client_sock.recv(data_size)
            print(tempt)
            data_string += tempt
            data_size = 0

    print(data_string.decode())