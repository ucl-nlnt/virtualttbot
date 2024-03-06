from KNetworking import DataBridgeClient_TCP, DataBridgeServer_TCP

server = DataBridgeServer_TCP(50000, True, enable_acks=False)

x = 0
while True:
    data = server.receive_data()
    print(f"{data}: {x}")
    x += 1
    server.send_data('lmao'.encode())