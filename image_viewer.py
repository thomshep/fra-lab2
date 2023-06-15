import cv2, rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def read_image_data(data):
    print("entre")
    try:
        cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
    cv2.imshow("Video", cv_image)
    cv2.waitKey(3)


rospy.init_node('img_viewer')

rospy.Subscriber("/usb_cam/image_raw", Image, read_image_data)
rospy.spin()