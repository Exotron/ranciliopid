from websocket import create_connection
ws = create_connection("ws://rancilio.local:81")
ws.send("power")
print("Sent")
print("Receiving...")
result =  ws.recv()
print("Received '%s'" % result)
ws.close()