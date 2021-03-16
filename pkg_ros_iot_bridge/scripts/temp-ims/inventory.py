#! /usr/bin/env python2.7
# import paho.mqtt.client as mqtt
import time
import rospy
import datetime
from IotClient.iot_client import IotClient # tesmporry testing


def get_item_details(color):
  """
    This function returns specification of Packgen based on color.
    :param color: It is color of the Packgen! Its possible values are red,yellow and green.
    :return:details,details is a dict containing information of packgen.It contains item,sku_alpha,priority,cost and Estimated Time of Delivery.
  """

  details = {'item':'Na','sku_alpha':'Na','priority':'Na','cost':'Na','Estimated Time of Delivery':'Na'}

  if color == 'red':
    details = {'item':'Medicines','sku_alpha':'R','priority':'HP','cost':'450','Estimated Time of Delivery':'1'}
  elif color == 'yellow':
    details = {'item':'Food','sku_alpha':'Y','priority':'MP','cost':'250','Estimated Time of Delivery':'3'}
  elif color == 'green':
    details = {'item':'Clothes','sku_alpha':'G','priority':'LP','cost':'150','Estimated Time of Delivery':'5'}

  return details


def get_inventory_string(color, i, j):
  """
  This function create a coma separated string of packgen information to send data on the inventory sheet.
  :param color:It is color of packgen.It can be red,yellow and green.
  :param i:i is index of column of packgen in the shelf.
  :param j:j is index of row of packgen in the shelf.
  :return: returns a string containing each column details of inventory sheet separated by comas.
  """
  details = get_item_details(color)
  date_now = datetime.datetime.now()
  storage_number = 'R'+str(i)+' C'+str(j)
  return 'Inventory,'+details['sku_alpha']+str(i)+str(j)+date_now.strftime("%m")+date_now.strftime("%y")+','+details['item']+','+details['priority']+','+storage_number+','+details['cost']+',1'


def pushdata():
    inventory = [
        ('red',0,0),
        ('green',0,1),
        ('yellow',0,2),
        ('yellow',1,0),
        ('green',1,1),
        ('red',1,2),
        ('yellow',2,0),
        ('green',2,1),
        ('red',2,2),
        ('yellow',3,0),
        ('red',3,1),
        ('green',3,2)
                ]
    global action_client
    for inv in inventory:
        time.sleep(8)
        str = get_inventory_string(inv[0],inv[1],inv[2])
        goal_handle1 = action_client.send_goal("mqtt", "pub", action_client._config_mqtt_pub_topic, str)
        action_client._goal_handles['1'] = goal_handle1
    

if __name__ == '__main__':
    rospy.init_node('inventory',anonymous=True)

    global action_client
    action_client = IotClient()

    pushdata()

print("Out of Loop. Exiting..")

