import time
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from breye_interface.msg import ObjectPoses
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

# list of states the bearcar can be in
class States:
    IDLE = 1
    ROAMING = 2
    DRIVING_TO_PASSENGER = 3
    WAITING_FOR_DESTINATION_INPUT = 4
    DRIVING_TO_DESTINATION = 5
    WAITING_FOR_PASSENGER_TO_GET_OUT = 6
    BLOCKED = 7


class Bearbrain(Node):

    # gives information while navigating
    # for us the position is most important, since didn't
    # implement getting it from the /tf topic
    def navigation_callback(self, msg):
        self.x = msg.feedback.current_pose.pose.position.x
        self.y = msg.feedback.current_pose.pose.position.y
        # print(msg.feedback)
        # self.publish_to_bearbrain_topic(str(self.y))
        return

    # this corresponds to the "publish point" feature in rviz
    def published_points_callback(self, msg):
        self.new_point_published = True
        self.published_point_x = msg.point.x
        self.published_point_y = msg.point.y
        print("received new point!")
        print(self.new_point_published)
        return

    # this gives us the object positions from "breye"
    def object_poses_callback(self, msg):
        self.new_object_detected = True
        self.object_name = msg.label[0]
        self.object_pos_x = msg.x[0]
        self.object_pos_y = msg.y[0]
        # print(self.object_name)

    # helper function to set a navigation goal
    def go_to(self, x, y):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.
        self.goal = goal_pose
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            print("waiting for server...")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, self.navigation_callback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()
        self.result_future = self.goal_handle.get_result_async()
        return True

    # helper function to see if we arrived at the desired position
    def is_nav_complete(self):
        # self.publish_to_bearbrain_topic(str(self.goal.pose.position.y))
        # self.publish_to_bearbrain_topic(str(self.y))

        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=1.0)
        if self.result_future.result():
            return True
        else:
            return False

    # helper function to publish debug data
    def publish_to_bearbrain_topic(self, text):
        msg = String()
        msg.data = text
        print(text)
        self.bearbrain_publisher.publish(msg)

    # main function representing the state machine form the sds document
    def main_loop(self):
        # TODO the threading is necessary in order for the callback functions to trigger.
        # TODO However it seems like the threads can't access the original "self" object.
        # TODO This issue is not solved yet
        threading.Timer(5, self.main_loop).start()

        # print()
        # print(self.x)
        # print(self.y)
        # print(self.state)
        # print()

        if self.state == States.IDLE:
            # go to a random position
            # TODO roaming locations static for better demonstration in simulation
            if self.go_to(0.0, 11.0) is True:
                self.state = States.ROAMING
                self.publish_to_bearbrain_topic('started roaming...')
        elif self.state == States.ROAMING:
            # look out for a new passenger
            # TODO dont hardcode passenger location (object position to global map transform needed)
            if self.new_object_detected:
                if self.object_name == "car" or self.object_name == "truck":
                    self.state = States.DRIVING_TO_PASSENGER
                    self.publish_to_bearbrain_topic('detected passenger. driving towards passenger...')
                    self.go_to(0.0, 20.0)
        elif self.state == States.DRIVING_TO_PASSENGER:
            # check if we arrived at the passenger
            if self.is_nav_complete() is True:
                self.state = States.WAITING_FOR_DESTINATION_INPUT
                self.publish_to_bearbrain_topic('arrived at passenger...')
                # end navigation
                future = self.goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, future)
        elif self.state == States.WAITING_FOR_DESTINATION_INPUT:
            # waiting for user to publish point in rviz
            # TODO implement timeout
            if self.new_point_published is True:
                self.new_point_published = False
                self.publish_to_bearbrain_topic('beginning to drive to destination...')
                self.state = States.DRIVING_TO_DESTINATION
                self.go_to(self.published_point_x, self.published_point_y)
            else:
                self.publish_to_bearbrain_topic('waiting for destination input...')
        elif self.state == States.DRIVING_TO_DESTINATION:
            # check if we arrived at the destination
            if self.is_nav_complete() is True:
                self.publish_to_bearbrain_topic('arrived at destination!')
                self.state = States.WAITING_FOR_PASSENGER_TO_GET_OUT
                self.arrival_time = int(time.time())
            else:
                print('driving to destination...')
        elif self.state == States.WAITING_FOR_PASSENGER_TO_GET_OUT:
            # check if enough time has passed for the passenger to get out
            if int(time.time()) - self.arrival_time > 5:
                self.publish_to_bearbrain_topic('Dropped of passenger. Roaming again.')
                self.state = States.ROAMING
                self.go_to(10.0, 10.0)
            else:
                print("waiting for passenger to get out...")
        else:
            # just for debugging
            # we shouldn't get to this state
            print("unknown state")

    def __init__(self):
        super().__init__('minimal_subscriber')

        # initialize variables
        self.goal = None
        self.arrival_time = int(time.time())
        self.result_future = None
        self.status = None
        self.goal_handle = None

        self.x = 0
        self.y = 0

        self.state = States.IDLE

        self.new_point_published = True
        self.published_point_x = 5.0
        self.published_point_y = 10.0

        self.new_object_detected = False
        self.object_name = ""
        self.object_pos_x = 0
        self.object_pos_y = 0

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # subscribe to relevant topics
        self.published_point_subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.published_points_callback,
            10)
        self.subscription = self.create_subscription(
            ObjectPoses,
            '/object_poses',
            self.object_poses_callback,
            10)

        # create publisher
        self.bearbrain_publisher = self.create_publisher(String, '/bearbrain', 10)
        self.publish_to_bearbrain_topic('bearbrain started...')

        # warmup phase to give the simulation time to start
        self.publish_to_bearbrain_topic('warming up...')
        time.sleep(20)
        self.publish_to_bearbrain_topic('warmup done!')

        # jump to main logic
        self.main_loop()


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = Bearbrain()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
