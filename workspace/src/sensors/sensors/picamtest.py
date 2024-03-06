from KNetworking import DataBridgeClient_TCP, DataBridgeServer_TCP

server = DataBridgeServer_TCP(50000, True, enable_acks=False)

x = 0

import random
import string

def generate_random_string(length):
    # Generates a random string of the specified length
    letters = string.ascii_letters + string.digits  # Includes both letters and numbers
    random_string = ''.join(random.choice(letters) for i in range(length))
    return random_string

test = generate_random_string(64000)
while True:
    data = server.receive_data()
    print(f"{data}: {x}")
    x += 1
    server.send_data(test.encode())