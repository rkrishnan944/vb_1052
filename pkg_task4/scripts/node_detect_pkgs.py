#!/usr/bin/env python

import rospy
import cv2
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from pyzbar.pyzbar import decode
from pkg_task4.msg import color
global list1
list1 = []

class Camera1:

  def __init__(self):

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)

  def get_qr_data(self,arg):

    result = decode(arg)
    if len(result)>0:
      return result[0].data
    else:
      return None

  def reshape(self,the_list, r, c): 
      """Reshape the 1D list into a m row x n column list""" 
      if r*c != len(the_list): 
          raise ValueError('Invalid new shape')  
      return [the_list[tr*c:(tr+1)*c] for tr in range(0,r)] 

  # The received camera image is segmented using openCV such that the image only contains "image" of shelf and nothing else,
  # thus easing the process of pkg detection. Now the image is segmented into equal parts using  simple 2-loops and image segmentation , 
  # passed into the qr code detector which returns the color. 
  # The color is stored into a 2D-array patter, whose index is also the package name suffix.For ex : [['red','yelloe','green'], ......]
  def callback(self,data):
    global list1
    
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)

    (rows,cols,channels) = cv_image.shape
    
    image = cv_image[300:920,100:620]

    imgheight = image.shape[0]
    imgwidth = image.shape[1]

    y = 0
    x = 0
    y1 = 0
    x1 = 0
    M = imgheight//4
    N = imgwidth//3

    # 4 row 3 column
    
    # list1 = []  
    for y in range(0,imgwidth,M):
        for x in range(0,imgwidth,N):

            y1 = y + M
            x1 = x + N 

            cv2.rectangle(image, (x, y), (x1, y1), (0, 255, 0))      
            value = self.get_qr_data(image[y:y1,x:x1])
            if value is not None:
                list1.append(value)

    print(self.reshape(list1, 4, 3))
    self.image_sub.unregister()
    # return 1
    
def main(args):
  global list1
  
  rospy.init_node('node_eg3_qr_decode', anonymous=True)

  ic = Camera1()
  
  rospy.sleep(.01)
  # The array is published so other nodes can get this array data and use it for further processing.
  while (1):
    var_message_handler = rospy.Publisher("eyrc/vb/color_data_to_array_data",color,queue_size = 10) 
    message = color()
    message.array = list1
    var_message_handler.publish(message)

  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
