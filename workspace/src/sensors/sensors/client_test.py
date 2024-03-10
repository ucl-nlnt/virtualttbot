import TwistedKNetwork

import random
import string

from twisted.internet import reactor, protocol
from twisted.internet.defer import Deferred
from twisted.internet.protocol import ClientFactory
import sys

import threading
def generate_random_string(length):
    # Generates a random string of the specified length
    letters = string.ascii_letters + string.digits  # Includes both letters and numbers
    random_string = ''.join(random.choice(letters) for i in range(length))
    return random_string

# data = generate_random_string(10000000)



    
to_send_variable = None
def start_logger():
    global to_send_variable
    while True:
        to_send_variable = input("Client << ")
        if to_send_variable == "$SHUTDOWN": sys.exit(0) # stop the process
        pass

client_thread = threading.Thread(target=start_logger)
client_thread.start()

ip = 'localhost'
port = 8000

communicator = TwistedKNetwork.ExternalCommunicator()
factory = TwistedKNetwork.CustomClientFactory(communicator=communicator)
print("Starting Twisted client...")
reactor.connectTCP(ip, port, factory)
reactor.run()

