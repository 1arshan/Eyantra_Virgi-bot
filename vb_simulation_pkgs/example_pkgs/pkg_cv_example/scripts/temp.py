#! /usr/bin/env python2.7

import rospy
import cv2
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from pyzbar.pyzbar import decode

class Camera1:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)

  
  def get_qr_data(self, arg_image):
    qr_result = decode(arg_image)
    #print(qr_result)
    for i in range(len(qr_result)):
      print(qr_result[i].data)
    if ( len( qr_result ) > 0):
      #print(qr_result[0])
      return (qr_result[0].data)
    else :
      return ('NA')

  
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)

    (rows,cols,channels) = cv_image.shape
    
    image = cv_image
    l=[]
    # Resize a 720x1280 image to 360x640 to fit it on the screen
    #resized_image = cv2.resize(image, (720/2, 1280/2)) 
    for i in range(5):
      for j in range(4):
        down=160*i
        side=180*j
        #down =160
        #side =180
        crop_image=image[280+down:440+down,90+side:270+side]
        l.append(self.get_qr_data(crop_image))
        #print(l)
        rospy.loginfo(self.get_qr_data(crop_image))
        

    #crop_image=image[280:440,90:270] #top:bottom,right:left ---pkg00
    crop_image=image[280:440,450:630] #top:bottom,right:left ---pkg01
    #crop_image=image[760:920,100:340] #top:bottom,right:left ---pkg00
    cv2.imshow("/eyrc/vb/camera_1/image_raw1", image)
    cv2.imshow("/eyrc/vb/camera_1/image_raw", crop_image)
    #rospy.loginfo(self.get_qr_data(crop_image))
    #print(image)
    
    cv2.waitKey(3)

def main(args):
  
  rospy.init_node('node_eg1_read_camera', anonymous=True)

  ic = Camera1()
  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo("Shutting down")
  
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
