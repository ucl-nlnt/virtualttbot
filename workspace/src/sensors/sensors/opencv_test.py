from KNetworking import DataBridgeClient_TCP

client = DataBridgeClient_TCP(50000, "localhost")

while True:
    client.send_data("fuk u".encode())
    data = client.receive_data().decode()
    print(data)
    