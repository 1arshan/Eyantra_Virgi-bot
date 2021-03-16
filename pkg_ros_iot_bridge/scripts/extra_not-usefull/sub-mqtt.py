import paho.mqtt.client as mqtt

broker_url = "broker.mqttdashboard.com"
broker_port = 1883

def on_connect(client, userdata, flags, rc):
    print("[INFO] Connected With Result Code: " + str(rc))

def on_message(client, userdata, message):
    print("--- Subscriber ---")
    payload =str(message.payload)
    print("[INFO] Topic: {}".format(message.topic) )
    print("[INFO] Message Recieved: {}".format(message.payload.decode()))
    single_order_info=payload.split(',')
    global order_no
    order_no=order_no+1

    if single_order_info[0]=="Medicines":
        priority =0
        color ="red"
    elif single_order_info[0]=="Food":
        priority=1
        color ="yellow"
    else:
        priority =2
        color ="green"
    list_to_dictionary ={"item":single_order_info[0],"color":color,"order_no":order_no}
    global order_info
    order_info.append(single_order_info)
    print(order_info)

    print("------------")

sub_client = mqtt.Client()
sub_client.on_connect = on_connect
sub_client.on_message = on_message
sub_client.connect(broker_url, broker_port)
global order_info
global order_no
order_no=0
order_info=[]
sub_client.subscribe("/eyrc/vb/VBfetjmi/orders", qos=0)

sub_client.loop_forever()

print("Out of Loop. Exiting..")
