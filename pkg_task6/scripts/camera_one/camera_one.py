#! /usr/bin/env python2.7
"""
    This file contains get_qr_data,get_item_details,get_inventory_string and Camera1.
"""

import rospy
import cv2
import datetime
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode

def get_qr_data(arg_image):
    """
        This function decode data from qr code and return package info.
        :param arg_image: Details of Packagen in QR code encoded form.
        :return String of Decoded information of Packagen.
    """

    qr_result = decode(arg_image)
    for i in range(len(qr_result)):
          print(qr_result[i].data)
    if len(qr_result) > 0:
          return qr_result[0].data
    else :
          return 'NA'


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


class Camera1:
  """
  This is a class for finding

   details of each packgen present in the sheet, using camera1 module.
  """

  def __init__(self,temp):
    """
    This is constructor of class Camera1
    :param temp: A variable to unregister subscriber after execution.
    """
    self.bridge = CvBridge()
    self.temp = temp
    self.list = {}
    self. inventory = []

  def callback(self,data):
        """
        This function is callback of camera1 module.
        :param data: result of camera1
        :return: null
        """
        cv_image = []
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          rospy.logerr(e)

        image = cv_image
        l={}
        for i in range(4):
          for j in range(3):
            down=160*i
            side=180*j
            crop_image=image[280+down:440+down,90+side:270+side]
            colr = get_qr_data(crop_image)
            l['packagen'+str(i)+str(j)]= colr
            self.inventory.append(get_inventory_string(colr, i, j))
            rospy.loginfo(get_qr_data(crop_image))
        print(l,self.inventory,'Callback') 
        self.list = l  
        self.temp.unregister()
        cv2.waitKey(3)
        
