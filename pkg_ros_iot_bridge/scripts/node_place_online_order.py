
import paho.mqtt.client as mqtt
import time
import rospy

broker_url = "broker.mqttdashboard.com"
broker_port =1883
pub_message = ""
pub_topic = "/eyrc/vb/VBfetjmi/orders"
item_list =["Clothes","Medicines","Food"]
cost=["150","450","250"]
city_list=["Mumbai","Chennai","Delhi","Amroha","Allahabad","Moradabad","Kanpur","Lucknow","Varanasi"]
latitude =["19.076 N","13.0827 N","28.7041 N","28.9052 N","25.4358 N","28.8386 N","26.4499 N","26.8467 N","25.3176 N"]
longitude =["72.8777 E","80.27 E","77.1025 E","78.4673 E","81.8463 E","78.7733 E","80.3319 E","80.9462 E","82.9739 E"]
order_quatity =1

def on_publish(client, userdata, mid):
    print("--- Publisher ---")
    print("[INFO] Topic: {}".format(pub_topic))
    print("[INFO] Message Published: {}".format(pub_message))
    print("msg receive: ",)
    print("------------")

pub_client = mqtt.Client()
pub_client.on_publish = on_publish
pub_client.connect(broker_url, broker_port)

for i in range(9):
    # pub_message=item_list[i%3]+","+city_list[i%9]+","+str(order_quatity)+","+cost[i%3]+","+latitude[i%9]+","+longitude[i%9]
    pub_message='{"city": "Mumbai", "order_time": "2021-03-02 08:14:33", "order_id": "3001", "lon": "72.8777 E", "qty": "1", "item": "Clothes", "lat": "19.0760 N"}'
    print(i,pub_message,'Ggg')
    pub_client.publish(topic=pub_topic, payload=pub_message, qos=0, retain=False)
    time.sleep(5)

print("Out of Loop. Exiting..")

