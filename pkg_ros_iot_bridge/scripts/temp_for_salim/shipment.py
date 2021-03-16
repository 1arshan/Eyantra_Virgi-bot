#! /usr/bin/env python2.7
# import paho.mqtt.client as mqtt
import time
import rospy
import datetime
from IotClient.iot_client import IotClient # tesmporry testing




def get_time_str(estimated_time_of_delivery):
    """
    This function return a time string of now plus estimated_time_of_delivery days
    :param estimated_time_of_delivery: Number of days to be added in time right now.
    :return: String of time.
    """
    x = datetime.datetime.now() + datetime.timedelta(days=int(estimated_time_of_delivery))

    return x.strftime("%a")+' '+x.strftime("%b")+' '+x.strftime("%d")+' '+x.strftime("%Y")+' - '+x.strftime("%X")


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


def get_shipment_string(order_shipped):
    """
    This function create a coma separated string of packgen information to send data on the inventory sheet.
    :param color:It is color of packgen.It can be red,yellow and green.
    :param i:i is index of column of packgen in the shelf.
    :param j:j is index of row of packgen in the shelf.
    :return: returns a string containing each column details of inventory sheet separated by comas.
    """
    order_id = order_shipped[0]
    order_city = order_shipped[1]
    order_details = get_item_details(order_shipped[2])
    # {'item':'Medicines','sku_alpha':'R','priority':'HP','cost':'450','Estimated Time of Delivery':'1'}
    shipped_str = 'OrdersShipped,'+order_id+','+order_city+','+order_details['item']+','+order_details['priority']+',1,'+order_details['cost']+',Yes,'+get_time_str(0)+','+get_time_str(order_details['Estimated Time of Delivery'])+','+order_details['Estimated Time of Delivery']
    return shipped_str


def pushdata():
    orders = [
        ("2002",'Kolkata','Food'),
        ("3002",'Pune','Clothes'),
        ("1001",'Delhi','Medicines'),
        ("2003",'Agartala','Food'),
        ("2001",'Chennai','Food'),
        ("1002",'Hyderabad','Medicines'),
        ("1003",'Amritsar','Medicines'),
        ("3001",'Mumbai','Clothes'),
        ("3003",'Bangalore','Clothes')
        ]
    time_gap = [10,167,174,170,199,183,143,186,144]  
    i = 0;            
    global action_client
    for order in orders:
        str = get_shipment_string(order)
        print(str)
        goal_handle1 = action_client.send_goal("mqtt", "pub", action_client._config_mqtt_pub_topic, str)
        action_client._goal_handles['1'] = goal_handle1
        time.sleep(time_gap[i])
        time.sleep(4)
        i = i+1

if __name__ == '__main__':
    rospy.init_node('shippedorder',anonymous=True)

    global action_client
    action_client = IotClient()

    pushdata()

print("Out of Loop. Exiting..")

