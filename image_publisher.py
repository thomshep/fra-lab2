import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

rospy.init_node('img_publisher')
pub = rospy.Publisher("/usb_cam/image_raw", Image,queue_size=10)

image_path = 'images/doblar_derecha.png'  # Reemplaza con la ruta y el nombre de tu imagen
image = cv2.imread(image_path)

bridge = CvBridge()

image_msg = bridge.cv2_to_imgmsg(image, encoding="bgr8")

while True:
    pub.publish(image_msg)

    rospy.sleep(2)