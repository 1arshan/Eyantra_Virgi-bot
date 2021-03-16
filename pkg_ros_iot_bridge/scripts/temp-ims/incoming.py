
import paho.mqtt.client as mqtt
import time
import rospy
from IotClient.iot_client import IotClient
# from iot_client.iot_client import IotClient
import datetime

#broker_url = "broker.mqttdashboard.com"
#broker_port =1883
#pub_message = ""
#pub_topic = "/eyrc/vb/VBfetjmm/orders"

item_list =["Food","Clothes","Food","Medicine","Food","Clothes","Medicine","Medicine","Clothes"] #

cost=["250","150","250","450","250","150","450","450","150"] #
city_list=["Kolkata","Pune","Agartala","Delhi","Chennai","Mumbai","Amritsar","Hyderabad","Bangalore"]
longitude =["88.3639 E","73.8567 E","91.2868 E","77.1025 E","80.2707 E","72.8777 E","74.8723 E","78.4867 E","77.5946 E"] #


latitude =[
"22.5726 N",
"18.5204 N",
"23.8315 N",
"28.7041 N",
"13.0827 N",
"19.0760 N",
"31.6340 N",
"17.3850 N",
"12.9716 N"] #

order_quatity =1
order_id =["2002","3002","2003","1001","2001","3001","1003","1002","3003"] #

priority=["MP","LP","MP","HP","MP","LP","HP","HP","LP"] #

result =0

"""
def on_publish(client, userdata, mid):
    print("--- Publisher ---")
    print("[INFO] Topic: {}".format(pub_topic))
    print("[INFO] Message Published: {}".format(pub_message))
    print("msg receive: ",)
    print("------------")"""
"""
pub_client = mqtt.Client()
pub_client.on_publish = on_publish
pub_client.connect(broker_url, broker_port)
"""
def push_data():

    for i in range(9):
        # pub_message=item_list[i%3]+","+city_list[i%9]+","+str(order_quatity)+","+cost[i%3]+","+latitude[i%9]+","+longitude[i%9]
        incomingDate = datetime.datetime.now()
        incomingTimeStr = incomingDate.strftime("%a")+' '+incomingDate.strftime("%b")+' '+incomingDate.strftime("%d")+' '+incomingDate.strftime("%Y")+' - '+incomingDate.strftime("%X")
        #incomingTimeStr=incomingDate
        pub_message="IncomingOrders,{},{},{},{},{},{},{},{},{}".format(order_id[i%9],incomingTimeStr,item_list[i%9],priority[i%9],order_quatity,city_list[i%9],longitude[i%9],latitude[i%9],cost[i%9])
        action_client = IotClient()
        action_client.send_goal("mqtt", "pub", action_client._config_mqtt_pub_topic,pub_message)
        time.sleep(2)
    
    return 1

print("Out of Loop. Exiting..")

#       incomingDate = datetime.datetime.now()
        # incomingTimeStr = incomingDate.strftime("%a")+' '+incomingDate.strftime("%b")+' '+incomingDate.strftime("%d")+' '+incomingDate.strftime("%Y")+' - '+incomingDate.strftime("%X")
        # order_to_be_publish='IncomingOrders,'+str(order_no)+','+str(incomingTimeStr)+','+ single_order_info[0]+','+priority+','+single_order_info[2]+','+single_order_info[1]+','+single_order_info[5]+','+single_order_info[4]+','+single_order_info[3]
        # print(order_to_be_publish,incomingTimeStr)


def main():
    rospy.init_node('node_task5_ur5_1', anonymous=True)
    """if result==0:
        result=1
        push_data()"""
    #push_data()
    #time.sleep(10)
    #rospy.spin()
    action_client = IotClient()
    timgap = [10, 53, 98, 36, 35, 115, 42, 32, 39]
    
    for i in range(0,9,1):
        # pub_message=item_list[i%3]+","+city_list[i%9]+","+str(order_quatity)+","+cost[i%3]+","+latitude[i%9]+","+longitude[i%9]
        incomingDate = datetime.datetime.now()
        incomingTimeStr = incomingDate.strftime("%a")+' '+incomingDate.strftime("%b")+' '+incomingDate.strftime("%d")+' '+incomingDate.strftime("%Y")+' - '+incomingDate.strftime("%X")
        #incomingTimeStr=incomingDate
        pub_message="IncomingOrders,{},{},{},{},{},{},{},{},{}".format(order_id[i%9],incomingTimeStr,item_list[i%9],priority[i%9],order_quatity,city_list[i%9],longitude[i%9],latitude[i%9],cost[i%9])
        rospy.sleep(timgap[i])
        rospy.sleep(4)
        goal_handle1=action_client.send_goal("mqtt", "pub", action_client._config_mqtt_pub_topic,pub_message)
        action_client._goal_handles['1'] = goal_handle1
        #time.sleep(4)
    rospy.spin()

if __name__ == '__main__':
    main()