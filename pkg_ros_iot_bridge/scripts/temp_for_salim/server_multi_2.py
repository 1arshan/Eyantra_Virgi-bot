#!/usr/bin/env python2.7

#handling ros service exception
import socket
import os
from thread import *
from get_sheet import get_data_from_inventory_sheet,get_data_from_sheet
import json

ServerSideSocket = socket.socket()
host = '127.0.0.1'
port = 2010
ThreadCount = 0
try:
    ServerSideSocket.bind((host, port))
except socket.error as e:
    print(str(e))

print('Socket is listening..')
ServerSideSocket.listen(15)
global ros_service_status
ros_service_status="free"


def multi_threaded_client(connection):
    connection.send(str.encode('Server is working:'))
    global ros_service_status
    match_box_color_with_index ={}
    
    while True:
        data = connection.recv(1024)
        response =data.decode('utf-8')
        if not data:
            break
        #print(response)
        
        if response=="inventory":
            match_box_color_with_index=get_data_from_inventory_sheet()
            print("inventory",match_box_color_with_index) 
            connection.sendall(str.encode(str(match_box_color_with_index)))        
        else:
            #response=str(response)
            print("response: ",response)
            response=json.loads(response)
            order_to_be_procced,max_order_id,order_info =get_data_from_sheet(response["max_order_id"],response["order_info"])
            data_to_be_send={}
            data_to_be_send["order_to_be_procced"]=order_to_be_procced
            #print("order_to_be_procced",data_to_be_send["order_to_be_procced"])
            data_to_be_send["max_order_id"]=max_order_id
            data_to_be_send["order_info"]=order_info
            print("what data is sending",data_to_be_send)
            connection.sendall(str.encode(json.dumps(data_to_be_send)))   
        #print(ros_service_status)
        
    connection.close()

while True:
    Client, address = ServerSideSocket.accept()
    print('Connected to: ' + address[0] + ':' + str(address[1]))
    start_new_thread(multi_threaded_client, (Client, ))
    ThreadCount += 1
    print('Thread Number: ' + str(ThreadCount))

ServerSideSocket.close()