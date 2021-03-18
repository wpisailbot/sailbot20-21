"""
Teensy stand-in made to allow the BBB to communicate with the computer and to receive messages that the Teensy would send back.
This has been minimally tested. The message format is correct, but the program is not interactive.
"""
import socket
import sys
import TrimTabMessages_pb2 as tt # This is placed in this folder by generateProtos.sh. You can also put this here manually.


# Create a TCP/IP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Bind the socket to the port
# Connect the socket to the port where the server is listening
OWN_IP = '192.168.0.14'
TRIM_PORT = 50000
server_address = (OWN_IP, TRIM_PORT)
print("connecting to ", server_address)
sock.connect(server_address)

# Create protobuf message container for the message to send
data = tt.ApparentWind_Trim()
# Initialize the data to send. Pretty arbitrary
data.apparent_wind = 5

# Create protubuf message container for the received message
receivedData = tt.TrimState()

while True:
    # Encode message. Python uses strings for serialization, so this isn't necessarily a printout.
    stringified = data.SerializeToString()
    # Send message over the socket
    sock.sendall(stringified)

    # Update data to send, just to show that the updated message is sent. Change this to suit your needs.
    data.apparent_wind = data.apparent_wind + 10.0125
    
    # Receive reply. Using a general 32 bytes seems to work well. Can use a signalling system using known character sequences if this breaks.
    reply = sock.recv(32)
    # Decode reply from bytes to protobuf structure
    receivedData.ParseFromString(reply)
    # Print the received data
    print(receivedData.control_angle)
    print(receivedData.state)