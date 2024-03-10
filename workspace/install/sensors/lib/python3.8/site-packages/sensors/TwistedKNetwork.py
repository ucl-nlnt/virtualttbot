from twisted.internet import reactor, protocol
from twisted.internet.defer import Deferred
from twisted.internet.protocol import ClientFactory

class ExternalCommunicator:
    def __init__(self):
        self.deferred = Deferred()

    def send_data(self, data):
        if self.client:
            self.client.sendData(data)
        else:
            print("Client not connected")

    def set_client(self, client):
        self.client = client
        self.client.external_communicator = self

    def receive_data(self, data):
        # You can add your logic here to process received data
        print("Received data:", data)
        self.deferred.callback(data)  # Notify any listeners

class CustomClient(protocol.Protocol):
    def connectionMade(self):
        self.factory.communicator.set_client(self)
        print("Connection made")

    def dataReceived(self, data):
        print("Data received:", data.decode())
        if hasattr(self, 'external_communicator'):
            self.external_communicator.receive_data(data)

    def sendData(self, data):
        self.transport.write(data)
        print("Data sent:", data)

class CustomClientFactory(ClientFactory):
    protocol = CustomClient

    def __init__(self, communicator):
        self.communicator = communicator

    def clientConnectionFailed(self, connector, reason):
        print('Connection failed:', reason.getErrorMessage())
        reactor.stop()

    def clientConnectionLost(self, connector, reason):
        print('Connection lost:', reason.getErrorMessage())
        reactor.stop()

# Usage example:
#communicator = ExternalCommunicator()
#factory = CustomClientFactory(communicator)
#reactor.connectTCP('localhost', 8000, factory)
#reactor.run()

# From outside the client object, after reactor.run() has been called in a different thread:
# communicator.send_data(b'Hello, world!')

from twisted.internet import reactor, protocol
from twisted.internet.defer import Deferred

class ServerExternalCommunicator:
    def __init__(self):
        self.clients = []
        self.deferred = Deferred()

    def broadcast(self, message):
        for client in self.clients:
            client.sendData(message)

    def register_client(self, client):
        self.clients.append(client)
        client.external_communicator = self

    def unregister_client(self, client):
        self.clients.remove(client)

    def receive_data(self, data):
        # This method can be expanded to process received data in various ways
        print("Server received:", data)
        self.deferred.callback(data)  # Notify any listeners

class CustomServer(protocol.Protocol):
    def connectionMade(self):
        self.factory.communicator.register_client(self)
        print("Client connected")

    def connectionLost(self, reason):
        self.factory.communicator.unregister_client(self)
        print("Client disconnected")

    def dataReceived(self, data):
        print("Received from client:", data.decode())
        if hasattr(self, 'external_communicator'):
            self.external_communicator.receive_data(data)

    def sendData(self, data):
        self.transport.write(data)
        print("Server sent:", data)

class CustomServerFactory(protocol.Factory):
    protocol = CustomServer

    def __init__(self, communicator):
        print("TCP Listener is starting up...")
        self.communicator = communicator

# Usage example:
#communicator = ServerExternalCommunicator()
#factory = CustomServerFactory(communicator)
#reactor.listenTCP(8000, factory)
#reactor.run()

# From outside, to send data back to all connected clients:
# communicator.broadcast(b'Hello from server!')
