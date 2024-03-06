from KNetworking import DataBridgeClient_TCP

client = DataBridgeClient_TCP(50000, "localhost")

import random
import string

def generate_random_string(length):
    # Generates a random string of the specified length
    letters = string.ascii_letters + string.digits  # Includes both letters and numbers
    random_string = ''.join(random.choice(letters) for i in range(length))
    return random_string
test = generate_random_string(64000)
while True:
    client.send_data(test.encode())
    data = client.receive_data().decode()
    print(data)
