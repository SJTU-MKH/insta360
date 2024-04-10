import numpy as np
import rospy
from sensor_msgs.msg import Image
import cv2


def imgmsg_to_cv2(img_msg):
    if img_msg.encoding != "bgr8":
        rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
    dtype = np.dtype("uint8") # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                    dtype=dtype, buffer=img_msg.data)

    image_opencv = image_opencv.byteswap().newbyteorder()
    image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_BGR2RGB)
    return image_opencv


def image_callback(msg):
    print("Received an image!")
    img = imgmsg_to_cv2(msg)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    cv2.imwrite("image.jpg", img)



rospy.init_node('image_processing_node')
rospy.Subscriber('/insta360/image_get', Image, image_callback, buff_size=10)
rospy.spin()


