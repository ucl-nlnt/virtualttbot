import TwistedKNetwork

from twisted.internet import reactor, protocol
from twisted.internet.defer import Deferred
from twisted.internet.protocol import ClientFactory

communicator = TwistedKNetwork.ServerExternalCommunicator()
factory = TwistedKNetwork.CustomServerFactory(communicator=communicator)
reactor.listenTCP(8000, factory)
reactor.run()