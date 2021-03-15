# Eyantra_Virgi-bot
Eyantra 2021 project theme Virgi Bot
### Project Working Video: https://www.youtube.com/watch?v=ZQNKmDvd1Eg

Our work:
## Initial Position:

<img src="/image_readme/initial.png">
<img src="/image_readme/bot1+shelf.png">
<img src="/image_readme/bot2.png">
<p align="center">overview of simulation scene</p>

### Making Inventory :
Every item in warehouse has a QR code on it so by using camera and pyzbar(python library for decoding QR code) update inventory sheet on GSheet.

<img src="/image_readme/Inventory.png">
<p align="center">GSheet of Inventory</p>


### Handling Incoming Order:
Order will be start publishing on MQTT(lightweight messaging protocol) Topic one by one with random time interval. Order are of different priority so if more than one order in queue then order with height priority will be proceed.
After finalizing order to be proceed a bot by using inventory will go to the box , pick it up and place it on conveyor belt.

<img src="/image_readme/IncomingOrders.png">
<p align="center">GSheet of IncomingOrders</p>


### Dispatch Order:
After item place on conveyor belt an email will be send to customer informing about their order being pick up and ready for dispatch.GSheet of Dispatch order is also updated
Then another bot will pick item from conveyor and place them on different bin for delivery to different location.Another mail also send informing their order is shipped.

<img src="/image_readme/OrdersDispatched.png">
<p align="center">GSheet of OrdersDispatched</p>


### Creating Dashboard:
Realtime information regarding inventory,incoming order, dispatch and shipped time is updated on Dashboard.

<img src="/image_readme/Dashboard.png">
<p align="center">GSheet of Dashboard</p>


### Use: 
Robot Operating System for controlling bot and gazebo for simulation ,Python :for operating ROS ; Javascript and HTML for creating Dashboard.
