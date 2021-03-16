#!/usr/bin/env python2.7
"""
This file contains server which handles multiple clients and manage ros service requests.
"""
#handling ros service exception
import socket
import os
from thread import *

ServerSideSocket = socket.socket()
host = '127.0.0.1'
port = 2004
ThreadCount = 0
try:
    ServerSideSocket.bind((host, port))
except socket.error as e:
    print(str(e))

print('Socket is listening..')
ServerSideSocket.listen(5)
global ros_service_status
ros_service_status="free"


def multi_threaded_client(connection):
    """
    This function create multiple threads for multiple clients
    :param connection: New Connection request
    :return: null
    """
    connection.send(str.encode('Server is working:'))
    global ros_service_status
    
    while True:
        data = connection.recv(2048)
        response =data.decode('utf-8')
        if not data:
            break
        print(response)
        if response=="busy":
            ros_service_status=response
        elif response=="free":
            ros_service_status=response
        else:
            response=ros_service_status
        print(response)
        connection.sendall(str.encode(str(ros_service_status)))
    connection.close()

while True:
    Client, address = ServerSideSocket.accept()
    print('Connected to: ' + address[0] + ':' + str(address[1]))
    start_new_thread(multi_threaded_client, (Client, ))
    ThreadCount += 1
    print('Thread Number: ' + str(ThreadCount))
