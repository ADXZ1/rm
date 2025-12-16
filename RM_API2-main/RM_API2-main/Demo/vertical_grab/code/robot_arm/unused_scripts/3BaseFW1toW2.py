
from moveBaseclient import TCPClient

def moveRobotBaseTo(id,cid):
    # Specify the server's IP and port
    server_host = '192.168.1.216'
    server_port = 12345

    # Specify the client's bind port
    client_bind_port = 54321  # Example: Bind the client to port 54321

    # Create the client instance with the bind port
    client = TCPClient(host=server_host, port=server_port, bind_port=client_bind_port)

    # Connect to the server
    client.connect()

    # Example command
    command = {"cmd": 1, "id": id, "cid": cid}

    # Send command
    client.send_command(command)

    # Listen for responses
    client.listen_for_responses()

    # Close connection
    client.close()

    print("Move robot base to position", id, cid)

def moveRobotBase_slowly(valueY):
      # Specify the server's IP and port
    server_host = '192.168.1.216'
    server_port = 12345
    # Specify the client's bind port
    client_bind_port = 54321  # Example: Bind the client to port 54321
    # Create the client instance with the bind port
    client = TCPClient(host=server_host, port=server_port, bind_port=client_bind_port)

    # Connect to the server
    client.connect()

    # Example command
    command = {"cmd": 2, "id": valueY, "cid": 0}

    # Send command
    client.send_command(command)

    # Listen for responses
    client.listen_for_responses()

    # Close connection
    client.close()

    print("Move robot base to position", id, 0)

if __name__ == "__main__":
    moveRobotBase_slowly(0)
    moveRobotBaseTo(3,1)
    moveRobotBase_slowly(0.2)