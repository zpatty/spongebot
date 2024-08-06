import socket

HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
PORT = 10000  # Port to listen on (non-privileged ports are > 1023)

s =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen()
conn, addr = s.accept()
print(conn)