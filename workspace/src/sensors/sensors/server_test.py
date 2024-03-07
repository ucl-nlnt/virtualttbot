from KNetworking import DataBridgeClient_TCP, DataBridgeServer_TCP, time

server = DataBridgeServer_TCP(50000, True, enable_acks=False)

x = 0

import random
import string

def generate_random_string(length):
    # Generates a random string of the specified length
    letters = string.ascii_letters + string.digits  # Includes both letters and numbers
    random_string = ''.join(random.choice(letters) for i in range(length))
    return random_string

print('creating test message')
test = generate_random_string(10000000) # test data
while True:
    print('waiting for data from client')
    data = server.receive_data()
    x += 1
    print("sending data to client")
    server.send_data(test.encode())
    print(x)