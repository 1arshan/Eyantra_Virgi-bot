#! /usr/bin/env python2.7

import requests
import json 
import heapq as hq #heap


def check_order(order_id,order_info):
    for i in order_info:
        if i[1] == order_id:   
            return True

    return False    

def check_if_dispatched(order_id):
    # URL = "https://spreadsheets.google.com/feeds/list/1rianYVvWCIJeoa17Jlrg7GZTUwuI_SG3KaKaaHtgGvY/4/public/full?alt=json"    ##eyrc.vb.1637@gmail.com
    URL = "https://spreadsheets.google.com/feeds/list/1QTyFVQA0YheuERNtD7Vq1ASVJl6tQ4rPGh65vFpExhg/4/public/full?alt=json"    ##vb1637eyrc@gmail.com
    #URL = "https://spreadsheets.google.com/feeds/list/1Twkrdg5QvlTRH15SLgWfh8tom5Pxjp-6QphH_s3vPIk/4/public/full?alt=json"    ##1637vbeyrc@gmail.com
    response = requests.get(URL) #order
    data =response.content
    res = json.loads(data)

    if u'entry' in  res["feed"]:
        res2 = res["feed"][u'entry']
    else: 
        return False
    for x in res2:
        content =x[u'content']
        content =content[u'$t']
        Dict = dict((a.strip(), b.strip()) 
            for a, b in (element.split(': ')  
            for element in content.split(', ')))
        if order_id == Dict[u'orderid'].encode('utf-8'):
            return True
    return False        

def get_data_from_sheet(max_order_id,order_info):
    # URL = "https://spreadsheets.google.com/feeds/list/1rianYVvWCIJeoa17Jlrg7GZTUwuI_SG3KaKaaHtgGvY/3/public/full?alt=json"    ##eyrc.vb.1637@gmail.com
    URL = "https://spreadsheets.google.com/feeds/list/1QTyFVQA0YheuERNtD7Vq1ASVJl6tQ4rPGh65vFpExhg/3/public/full?alt=json"    ##vb1637eyrc@gmail.com
    #URL = "https://spreadsheets.google.com/feeds/list/1Twkrdg5QvlTRH15SLgWfh8tom5Pxjp-6QphH_s3vPIk/3/public/full?alt=json"    ##1637vbeyrc@gmail.com
    
    response = requests.get(URL) #order
    data =response.content
    res = json.loads(data)
    if u'entry' in  res["feed"]:
        #print("entry present")
        res2 = res["feed"][u'entry']
    else: 
        order_to_be_procced=()
        #print("no data present")
        return order_to_be_procced,max_order_id,order_info
    res2 = res["feed"][u'entry']
    #order_info=[]
    hq.heapify(order_info)
    #max_order_id =0
    for x in res2:
        content =x[u'content']
        content =content[u'$t']
        Dict = dict((a.strip(), b.strip()) 
            for a, b in (element.split(': ')  
            for element in content.split(', ')))
        
        if Dict[u'item']=="Medicines" or Dict[u'item']=="Medicine":
            Dict[u'priority'] =0 #0
            color ="red"
        elif Dict[u'item']=="Food":
            Dict[u'priority']=1 #1
            color ="yellow"
        else:
            Dict[u'priority'] =2 #2
            color ="green"
        # if max_order_id < int(Dict[u'orderid']): 
        order_id_encoded = Dict[u'orderid'].encode('utf-8')
        if  not check_order(Dict[u'orderid'],order_info) and not check_if_dispatched(order_id_encoded): 
            max_order_id=int(Dict[u'orderid'])
            tup=(Dict[u'priority'],Dict[u'orderid'],Dict[u'item'],Dict[u'city'])
            hq.heappush(order_info,tup) #always have highest priority upward
    #print(order_info)
    if len(order_info)>0:
        order_to_be_procced =hq.heappop(order_info) #order with highest priority
    else:
        order_to_be_procced=()
    print("order_to_be_procced",order_to_be_procced)
    print("order_info: ", order_info) 
    return order_to_be_procced,max_order_id,order_info

"""    
order_info=[]
hq.heapify(order_info)
max_order_id =0
#order_to_be_procced,max_order_id,order_info =get_data_from_sheet(0,order_info)
#print(order_to_be_procced, max_order_id)

for i in range(8):
    order_to_be_procced,max_order_id,order_info =get_data_from_sheet(max_order_id,order_info)
    print(order_to_be_procced, max_order_id)

"""

def get_data_from_inventory_sheet():
   
    # URL = "https://spreadsheets.google.com/feeds/list/1rianYVvWCIJeoa17Jlrg7GZTUwuI_SG3KaKaaHtgGvY/2/public/full?alt=json"    ##eyrc.vb.1637@gmail.com
    URL = "https://spreadsheets.google.com/feeds/list/1QTyFVQA0YheuERNtD7Vq1ASVJl6tQ4rPGh65vFpExhg/2/public/full?alt=json"    ##vb1637eyrc@gmail.com
    #URL = "https://spreadsheets.google.com/feeds/list/1Twkrdg5QvlTRH15SLgWfh8tom5Pxjp-6QphH_s3vPIk/2/public/full?alt=json"      ##1637vbeyrc@gmail.com
   
    response = requests.get(URL) #inventory
    data =response.content
    res = json.loads(data)
    if u'entry' in  res["feed"]:
        res2 = res["feed"][u'entry']
    else: 
        match_box_color_with_index ={}
        return match_box_color_with_index
    res2 = res["feed"][u'entry']
    match_box_color_with_index ={}
    for x in res2:
        content =x[u'content']
        content =content[u'$t']
        Dict = dict((a.strip(), b.strip()) 
            for a, b in (element.split(': ')  
            for element in content.split(', ')))
        box_index =Dict[u'sku']
        box_index=box_index[1:3]
        
        match_box_color_with_index.update({box_index.encode("utf-8"):Dict[u'item'].encode("utf-8")}) # dic which will match storage number with box item
    #print(match_box_color_with_index)
    return match_box_color_with_index
    

check_if_dispatched('2002') 