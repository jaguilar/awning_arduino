#! /usr/bin/env python3

import socket, threading

HOST = '192.168.4.22'
PORT = 51234 

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(4)
clients = [] #list of clients connected
lock = threading.Lock()


class chatServer(threading.Thread):
    def __init__(self, socket, address):
        threading.Thread.__init__(self)
        self.socket = socket
        self.address = address

    def run(self):
        lock.acquire()
        clients.append(self)
        lock.release()
        print('%s:%s connected.' % self.address)
        while True:
            data = self.socket.recv(4)
            if not data:
                break
            for c in clients:
                c.socket.send(b"pong")
                print("pong")
        self.socket.close()
        print('%s:%s disconnected.' % self.address)
        lock.acquire()
        clients.remove(self)
        lock.release()

while True: # wait for socket to connect
    # send socket to chatserver and start monitoring
    chatServer(*s.accept()).start()
