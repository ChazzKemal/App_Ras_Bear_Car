# Code for subscribing to ros camera topic using the cv_bridge by Addison Sears-Collins (https://automaticaddison.com)


from math import sqrt
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from sensor_msgs.msg import CameraInfo
from breye_interface.msg import ObjectPoses
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import time
import numpy as np
from std_msgs.msg import String
import os

path = os.getcwd()

min_conf = 0.6
nmsThreshold = 0.4

# chose the full or scaled down version of the net
net = cv2.dnn.readNet("src/breye/breye/yolov3-tiny.weights", "src/breye/breye/yolov3-tiny.cfg")
# net = cv2.dnn.readNet("src/breye/breye/yolov3.weights", "src/breye/breye/yolov3.cfg")

classes = []
with open("src/breye/breye/coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()

output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]  # chaned

colors = np.random.uniform(0, 255, size=(len(classes), 3))

depth_img = np.empty([1280, 720])
fx, cx, fy, cy = 0, 0, 0, 0


class DetectAndShow(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_subscriber')

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.color_subscription = self.create_subscription(
            Image,
            '/color/image_raw',
            self.color_callback,
            10)

        self.depth_subscription = self.create_subscription(
            Image,
            'aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/color/camera_info',
            self.camera_info_callback,
            10
        )

        self.pose_publisher = self.create_publisher(ObjectPoses, 'object_poses', 10)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        self.publisher_ = self.create_publisher(String, '/breye', 10)

        self.last_update = time.time()

    def color_callback(self, data):
        # Running the net only with 2 Hz
        if time.time() - self.last_update > 0.5:
            self.last_update = time.time()
            # Convert ROS Image message to OpenCV image
            current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

            img = current_frame

            height, width, channels = img.shape
            blob = cv2.dnn.blobFromImage(img, 0.00392, (320, 320), (0, 0, 0), True, crop=False)
            net.setInput(blob)
            outs = net.forward(output_layers)

            self.visualize(outs, img)

    def depth_callback(self, data):
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='16UC1')

        # Display image
        dist = current_frame
        dist = np.clip(dist, 0, 5000)

        global depth_img
        depth_img = dist

    def camera_info_callback(self, data):
        global fx, cx, fy, cy
        fx = data.k[0]
        cx = data.k[2]
        fy = data.k[4]
        cy = data.k[5]

    def visualize(self, data, img):
        # Showing information on the screen

        class_ids = []
        confidences = []
        boxes = []
        boxes2 = []

        global depth_img, fx, cx, fy, cy
        height = np.shape(img)[0]
        width = np.shape(img)[1]
        height_depth = np.shape(depth_img)[0]
        width_depth = np.shape(depth_img)[1]

        for out in data:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > min_conf:
                    # Object detected
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    # Rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h, center_x, center_y])
                    boxes2.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes2, confidences, min_conf, nmsThreshold)

        msg = ObjectPoses()
        msg_label = []
        msg_confidences = []
        msg_x = []
        msg_y = []

        font = cv2.FONT_HERSHEY_PLAIN
        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h, center_x, center_y = boxes[i]
                depth_center_x = int((center_x) * (width_depth / width))
                depth_center_y = int((center_y) * (height_depth / height))
                color = colors[i]

                # Selecting the 81 pixels around the center of detected object to get a more robust depth data
                depth_arr = depth_img[depth_center_y - 4:depth_center_y + 4, depth_center_x - 4:depth_center_x + 4]
                # Excluding Values with 0 for the average distance
                depth = np.average(np.ma.masked_where(depth_arr == 0, depth_arr))

                if depth:
                    qy = depth / sqrt(1 + ((x - cx) / fx) ** 2 + ((y - cy) / fy) ** 2)
                    qx = (x - cx) / fx * qy
                else:
                    qy = 0.0
                    qx = 0.0

                msg_label.append(classes[class_ids[i]])
                msg_confidences.append(confidences[i])
                msg_x.append(qx)
                msg_y.append(qy)

                text2 = f"distance: {depth / 1000:.2f} m"
                text3 = f"x: {qx / 1000:.2f} m"
                text4 = f"y: {qy / 1000:.2f} m"
                text = f"{classes[class_ids[i]]}: {confidences[i]:.2f}"
                cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)
                cv2.putText(img, text, (x, y + 30), font, 2, color, 2)
                cv2.putText(img, text2, (x, y + 60), font, 2, color, 2)
                cv2.putText(img, text3, (x, y + 90), font, 2, color, 2)
                cv2.putText(img, text4, (x, y + 120), font, 2, color, 2)

        msg.label = msg_label
        msg.confidence = msg_confidences
        msg.x = msg_x
        msg.y = msg_y

        if len(msg.label) > 0:
            print("publishing")
            self.pose_publisher.publish(msg)

        cv2.imshow("Image", img)
        cv2.waitKey(1)


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    detect_and_show = DetectAndShow()

    # Spin the node so the callback function is called.
    rclpy.spin(detect_and_show)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    detect_and_show.destroy_node()

    # Closes all the frames
    cv2.destroyAllWindows()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
