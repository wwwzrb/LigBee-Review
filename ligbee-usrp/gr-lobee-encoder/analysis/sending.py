import socket
import time

UDP_IP = "127.0.0.1"
UDP_PORT = 40868
# MESSAGE = "012345678901234567"
# MESSAGE = "123456123456123456123456123456123456"
# MESSAGE = "123456"
MESSAGE = "0000"

print("UDP target IP:", UDP_IP)
print("UDP target port:", UDP_PORT)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP

my_bytes = bytearray()
message_list = [0, 0, 0, 222, 173, 190, 239]
for i in range(len(message_list)):
    my_bytes.append(message_list[i])
while True:
    # sock.sendto(my_bytes, (UDP_IP, UDP_PORT))
    # print("bytes array:", my_bytes)
    sock.sendto(MESSAGE.encode('utf-8'), (UDP_IP, UDP_PORT))
    print("message:", MESSAGE)
    time.sleep(.05)

