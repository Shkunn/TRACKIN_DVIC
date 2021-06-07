import socket
import argparse

IP               = "172.21.72.175"
PORT             = 5000

parser = argparse.ArgumentParser()
parser.add_argument("msg")

args   = parser.parse_args()
msg    = str(args.msg)
cc     = msg.encode()

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as opened_socket:
    opened_socket.setblocking(0)
    opened_socket.sendto(cc, (IP, PORT))