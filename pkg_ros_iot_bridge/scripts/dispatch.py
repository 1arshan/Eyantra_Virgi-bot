#! /usr/bin/env python2.7
# import paho.mqtt.client as mqtt
import time
import rospy
import datetime
from IotClient.iot_client import IotClient # tesmporry testing

def get_item_details(item):
    """
    This function finds packgen details using item
    :param item: item is a string containing packgen content type
    :return: it returns dict of details of packgen.
    """
    details = {'Estimated Time of Delivery':'Na','priority':'Na','cost':'Na','item':'Na'}

    if item == 'Medicines' or item == 'Medicine':
        details = {'priority':'HP','cost':'450','Estimated Time of Delivery':'1','item':'Medicines'}
    elif item == 'Food':
        details = {'priority':'MP','cost':'250','Estimated Time of Delivery':'3',"item":'Food'}
    elif item == 'Clothes':
        details = {'priority':'LP','cost':'150','Estimated Time of Delivery':'5','item':'Clothes'}

    return details


def get_dispatch_string(order_to_be_procced):
    """
    This function create a coma separated string of packgen information to send data on the inventory sheet.
    :param color:It is color of packgen.It can be red,yellow and green.
    :param i:i is index of column of packgen in the shelf.
    :param j:j is index of row of packgen in the shelf.
    :return: returns a string containing each column details of inventory sheet separated by comas.
    """
    order_details_sheet = get_item_details(order_to_be_procced[2])
    dispatch_date = datetime.datetime.now()
    dispatch_time_str = dispatch_date.strftime("%a")+' '+dispatch_date.strftime("%b")+' '+dispatch_date.strftime("%d")+' '+dispatch_date.strftime("%Y")+' - '+dispatch_date.strftime("%X")
    print(dispatch_time_str,'DispatchTime')
    dipatch_order_string = 'OrdersDispatched,'+order_to_be_procced[0]+','+order_to_be_procced[1]+','+order_details_sheet['item']+','+order_details_sheet['priority']+',1,'+order_details_sheet['cost']+',Yes,'+dispatch_time_str
    return dipatch_order_string


def pushdata():
    orders = [
        ('1003','Amritsar','Medicines'),
        ('1001','Delhi','Medicines'),
        ('2002','Kolkata','Food'),
        ('2003','Agartala','Food'),
        ('1002','Hyderabad','Medicines'),
        ('2001','Chennai','Food'),
        ('3001','Mumbai','Clothes'),
        ('3002','Pune','Clothes'),
        ('3003','Bangalore','Clothes')
        ]
    time_gap = [10, 117,241,95,196,146,206,146,235]  
    i = 0;            
    global action_client
    for order in orders:
        str = get_dispatch_string(order)
        goal_handle1 = action_client.send_goal("mqtt", "pub", action_client._config_mqtt_pub_topic, str)
        action_client._goal_handles['1'] = goal_handle1
        time.sleep(time_gap[i])
        i = i+1

if __name__ == '__main__':
    rospy.init_node('dispatchorder',anonymous=True)

    global action_client
    action_client = IotClient()

    pushdata()

print("Out of Loop. Exiting..")

