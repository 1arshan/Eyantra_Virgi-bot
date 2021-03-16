#!/usr/bin/env python2.7 

# ROS Node - Action Server - IoT ROS Bridge

import rospy
import actionlib
import threading
import requests
import re #to split msg receive from action client
import json
from pkg_ros_iot_bridge.msg import msgRosIotAction  # Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotGoal  # Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult  # Message Class that is used for Result Messages
from pkg_ros_iot_bridge.msg import msgRosIotFeedback  # Message Class that is used for Feedback Messages
from pkg_ros_iot_bridge.msg import msgMqttSub  # Message Class for MQTT Subscription Messages,communicate with pkg_task1



from pyiot import iot  # Custom Python Module to perform MQTT Tasks
import paho.mqtt.client as mqtt #for mqtt
import heapq as hq #heap
from IotClient.iot_client import IotClient # tesmporry testing
#from pkg_task5.scripts.IotClient.iot_client import IotClient
import datetime
import json


class IotRosBridgeActionServer:

    # Constructor
    def __init__(self):
        # Initialize the Action Server
        self._as = actionlib.ActionServer('/action_iot_ros',
                                          msgRosIotAction,
                                          self.on_goal,
                                          self.on_cancel,
                                          auto_start=False)

        '''
            * self.on_goal - It is the function pointer which points to a function which will be called
                             when the Action Server receives a Goal.

            * self.on_cancel - It is the function pointer which points to a function which will be called
                             when the Action Server receives a Cancel Request.
        '''
        #sheet url id add
        # Read and Store IoT Configuration data from Parameter Server
        param_config_iot = rospy.get_param('config_iot')
        self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']
        self._config_mqtt_qos = param_config_iot['mqtt']['qos']
        self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']
        self._config_google_apps_spread_sheet_id = param_config_iot['google_apps']['spread_sheet_id']
        self._keys = param_config_iot['keys']
        #print(param_config_iot)

        # Initialize ROS Topic Publication
        # Incoming message from MQTT Subscription will be published on a ROS Topic (/ros_iot_bridge/mqtt/sub).
        # ROS Nodes can subscribe to this ROS Topic (/ros_iot_bridge/mqtt/sub) to get messages from MQTT Subscription.
        self._handle_ros_pub = rospy.Publisher(self._config_mqtt_sub_cb_ros_topic, msgMqttSub, queue_size=10)

        # Subscribe to MQTT Topic (eyrc/xYzqLm/iot_to_ros) which is defined in 'config_iot_ros.yaml'.
        # self.mqtt_sub_callback() function will be called when there is a message from MQTT Subscription.
        ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                              self._config_mqtt_server_url,
                                              self._config_mqtt_server_port,
                                              self._config_mqtt_sub_topic,
                                              self._config_mqtt_qos)
        if ret == 0:
            rospy.loginfo("MQTT Subscribe Thread Started")
        else:
            rospy.logerr("Failed to start MQTT Subscribe Thread")

        # Start the Action Server
        self._as.start()

        rospy.loginfo("Started ROS-IoT Bridge Action Server.")

    # This is a callback function for MQTT Subscriptions
    def mqtt_sub_callback(self, client, userdata, message):
        
        payload = str(message.payload)
        #payload = str(.format(message.payload.decode()))
        #print("payload",payload)

        print("[INFO] Topic: {}".format(message.topic) )
        print("[INFO] Message Recieved: {}".format(message.payload.decode()))

        
        print(json.loads(payload),'this')
        order_json = json.loads(payload)
        # {u'city': u'Mumbai', u'order_time': u'2021-03-02 08:14:33', u'order_id': u'3001', u'lon': u'72.8777 E', u'qty': u'1', u'item': u'Clothes', u'lat': u'19.0760 N'}
        
        # - Order ID
        # - Order Date and Time
        # - Item
        # - Priority
        # - Order Quantity
        # - City
        # - Longitude
        # - Latitude
        # - Cost
        
        if order_json[u'item'].encode('utf-8')=="Medicines" or order_json[u'item'].encode('utf-8')=="Medicine":
            priority ='HP' #0
            cost = '450'
        elif order_json[u'item'].encode('utf-8')=="Food":
            priority='MP' #1
            cost = '250'
        elif order_json[u'item'].encode('utf-8')=="Clothes":
            priority ='LP'
            cost = '150' #2

        order_to_be_publish = 'IncomingOrders,'+order_json[u'order_id'].encode('utf-8')+','+order_json[u'order_time'].encode('utf-8')+','+order_json[u'item'].encode('utf-8')+','+priority+','+order_json[u'qty'].encode('utf-8')+','+order_json[u'city'].encode('utf-8')+','+order_json[u'lon'].encode('utf-8')+','+order_json[u'lat'].encode('utf-8')+','+cost

        #sending ur5_1 msg to pick the required box

        # single_order_info=payload.split(',')
        # global order_no
        # order_no = order_no+1
        # order_no = int(single_order_info[6])


        # if single_order_info[0]=="Medicines":
        #     priority ='HP' #0
        # elif single_order_info[0]=="Food":
        #     priority='MP' #1
        # else:
        #     priority ='LP' #2
           
        # tup=(priority,order_no,single_order_info[0])  #to make heap
        # global order_info
        # # global match_box_color_with_index
        
        # hq.heappush(order_info,tup) #always have highest priority upward
        # print("before sending order",order_info)            
        # order_to_be_procced =hq.heappop(order_info) #order with highest priority
        # print("aftersending order",order_info)

        # for key, value in match_box_color_with_index.items():
        #     print(key,value,order_to_be_procced[2])
        #     if order_to_be_procced[2] == value:
        #         box_to_be_pick=key  #key pta chl gyi
        #         match_box_color_with_index.pop(key)
        #         break        
                  
        # print("match_box_color_with_index: ",match_box_color_with_index)
        
        global action_client #IOT clinet 
        # incomingDate = datetime.datetime.now()
        # incomingTimeStr = incomingDate.strftime("%a")+' '+incomingDate.strftime("%b")+' '+incomingDate.strftime("%d")+' '+incomingDate.strftime("%Y")+' - '+incomingDate.strftime("%X")
        # order_to_be_publish='IncomingOrders,'+str(order_no)+','+str(incomingTimeStr)+','+ single_order_info[0]+','+priority+','+single_order_info[2]+','+single_order_info[1]+','+single_order_info[5]+','+single_order_info[4]+','+single_order_info[3]
        # print(order_to_be_publish,incomingTimeStr)
        action_client.send_goal("mqtt", "pub", action_client._config_mqtt_pub_topic,order_to_be_publish) #send to IOT bridge

        


    # This function will be called when Action Server receives a Goal
    def on_goal(self, goal_handle):
        goal = goal_handle.get_goal()

        rospy.loginfo("Received new goal from Client")
        rospy.loginfo(goal)  # action
        
        #---pushign to google shhet
        # defining our sheet name in the 'id' variable and the the column where we want to update the value
        temp =goal.message
        # [+-]?\d+
        # x = re.findall(r"[\w']+", temp)
        # x = re.findall(r"[+-]?\d+", temp)
        x = temp.split(',')
        # global match_box_color_with_index #to store box color
        print(x)
        # if x[0] == 'Inventory':
        #     box_index =x[1]
        #     box_index=box_index[1:3]
        #     print(box_index,"box")
        #     match_box_color_with_index.update({box_index:x[2]}) # dic which will match storage number with box item
        #     print(match_box_color_with_index,"Ggg")
        #     print("color with index",match_box_color_with_index)
        i = 1
        parameters = {"id":x[0],'Team Id':'VB#1637','Unique Id':"VBfetjmi"}
        #print(self._keys)
        sheet_name = x[0]
        keys = self._keys[sheet_name]
        for j in keys:
            parameters.update({j:x[i]})
            i = i+1
        print(parameters,'parameters')
        # print(parameters,'Params')    
        # parameters = {"id":"Sheet1", "turtle_x":x[0],"turtle_y":x[1],"turtle_theta":x[2]}
        # parameters = {"id":"task1", "turtle_x":x[0],"turtle_y":x[1],"turtle_theta":x[2],"team_id":"VB_1637","unique_id":"VB_1637"}

        URL = "https://script.google.com/macros/s/"+self._config_google_apps_spread_sheet_id+"/exec"

        response = requests.get(URL, params=parameters)

        print(response.content)

        # Validate incoming goal parameters
        if goal.protocol == "mqtt":

            if (goal.mode == "pub") or (goal.mode == "sub"):
                goal_handle.set_accepted()

                # Start a new thread to process new goal from the client (For Asynchronous Processing of Goals)
                # 'self.process_goal' - is the function pointer which points to a function that will process incoming Goals
                thread = threading.Thread(name="worker",
                                          target=self.process_goal,
                                          args=(goal_handle,))
                thread.start()

            else:
                goal_handle.set_rejected()
                return

        else:
            goal_handle.set_rejected()
            return

    # This function is called is a separate thread to process Goal.
    def process_goal(self, goal_handle):

        flag_success = False
        result = msgRosIotResult()

        goal_id = goal_handle.get_goal_id()
        rospy.loginfo("Processing goal : " + str(goal_id.id))

        goal = goal_handle.get_goal()

        # Goal Processing
        if goal.protocol == "mqtt":
            rospy.logwarn("MQTT")

            if goal.mode == "pub":
                rospy.logwarn("MQTT PUB Goal ID: " + str(goal_id.id))

                rospy.logwarn(goal.topic + " > " + goal.message)

                ret = iot.mqtt_publish(self._config_mqtt_server_url,
                                       self._config_mqtt_server_port,
                                       goal.topic,
                                       goal.message,
                                       self._config_mqtt_qos)

                if ret == 0:
                    rospy.loginfo("MQTT Publish Successful.")
                    result.flag_success = True
                else:
                    rospy.logerr("MQTT Failed to Publish")
                    result.flag_success = False

            elif goal.mode == "sub":
                rospy.logwarn("MQTT SUB Goal ID: " + str(goal_id.id))
                rospy.logwarn(goal.topic)

                ret = iot.mqtt_subscribe_thread_start(self.mqtt_sub_callback,
                                                      self._config_mqtt_server_url,
                                                      self._config_mqtt_server_port,
                                                      goal.topic,
                                                      self._config_mqtt_qos)
                if (ret == 0):
                    rospy.loginfo("MQTT Subscribe Thread Started")
                    result.flag_success = True
                else:
                    rospy.logerr("Failed to start MQTT Subscribe Thread")
                    result.flag_success = False

        rospy.loginfo("Send goal result to client")
        if (result.flag_success == True):
            rospy.loginfo("Succeeded abc")
            goal_handle.set_succeeded(result)
            # goal_handle.set_succeeded(result)
        else:
            rospy.loginfo("Goal Failed. Aborting.")
            goal_handle.set_aborted(result)

        rospy.loginfo("Goal ID: " + str(goal_id.id) + " Goal Processing Done.")

    # This function will be called when Goal Cancel request is send to the Action Server
    def on_cancel(self, goal_handle):
        rospy.loginfo("Received cancel request.")
        goal_id = goal_handle.get_goal_id()


# Main
def main():
    rospy.init_node('node_action_server_ros_iot_bridge')

    action_server = IotRosBridgeActionServer()
    global order_info #store information of order
    global order_no #order no
    order_no = 1000
    global action_client
    action_client = IotClient() 
    # global match_box_color_with_index #box colr with index
    # match_box_color_with_index={} 
    # match_box_color_with_index={'02': 'Clothes', '10': 'Clothes', '00': 'Medicines',
    #                             '01': 'Food', '20': 'Clothes', '21': 'Medicines',
    #                             '22': 'Food', '32': 'Medicines', '31': 'Clothes',
    #                             '30': 'Food', '12': 'Medicines', '11': 'Food'}  # testing
    # order_no=1000
    order_info=[]
    hq.heapify(order_info)

    rospy.spin()


if __name__ == '__main__':
    global action_client    
    main()

