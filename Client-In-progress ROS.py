from drone_interfaces.srv import Move
from std_srvs.srv import Empty

import rclpy
import time
from drone_interfaces.srv import Video
from cv_bridge import CvBridge
import cv2

def main(args=None):
    rclpy.init(args=args)
    bridge = CvBridge()

    node = rclpy.create_node('drone_service_client')
    # cli = node.create_client(Empty, 'takeoff')
    # req = Empty.Request()
    # while not cli.wait_for_service(timeout_sec=1.0):
    #     node.get_logger().info('service not available, waiting again...')

    # future = cli.call_async(req)
    # rclpy.spin_until_future_complete(node, future)

    # try:
    #     result = future.result()
    # except Exception as e:
    #     node.get_logger().info('Service call failed %r' % (e,))
    # else:
    #     node.get_logger().info("takeoff successful")

    # time.sleep(5)
    #########################################################################################

    # cli = node.create_client(Empty, 'battery')
    # req = Empty.Request()
    # while not cli.wait_for_service(timeout_sec=1.0):
    #     node.get_logger().info('service not available, waiting again...')

    # future = cli.call_async(req)
    # rclpy.spin_until_future_complete(node, future)

    # try:
    #     result = future.result()
    # except Exception as e:
    #     node.get_logger().info('Service call failed %r' % (e,))
    # else:
    #     node.get_logger().info("Battery status is shown")

    # time.sleep(5)
    #########################################################################################

    # cli = node.create_client(Move, 'move_forward')
    # req = Move.Request()
    # req.distance = 100
    # while not cli.wait_for_service(timeout_sec=1.0):
    #     node.get_logger().info('service not available, waiting again...')

    # future = cli.call_async(req)
    # rclpy.spin_until_future_complete(node, future)

    # try:
    #     result = future.result()
    # except Exception as e:
    #     node.get_logger().info('Service call failed %r' % (e,))
    # else:
    #     node.get_logger().info("move forward successful")

    # time.sleep(5)
    #########################################################################################

    # cli = node.create_client(Move, 'move_backward')
    # req = Move.Request()
    # req.distance = 120
    # while not cli.wait_for_service(timeout_sec=1.0):
    #     node.get_logger().info('service not available, waiting again...')

    # future = cli.call_async(req)
    # rclpy.spin_until_future_complete(node, future)

    # try:
    #     result = future.result()
    # except Exception as e:
    #     node.get_logger().info('Service call failed %r' % (e,))
    # else:
    #     node.get_logger().info("move backward successful")

    # time.sleep(5)
    #########################################################################################

    # cli = node.create_client(Move, 'move_left')
    # req = Move.Request()
    # req.distance = 100
    # while not cli.wait_for_service(timeout_sec=1.0):
    #     node.get_logger().info('service not available, waiting again...')

    # future = cli.call_async(req)
    # rclpy.spin_until_future_complete(node, future)

    # try:
    #     result = future.result()
    # except Exception as e:
    #     node.get_logger().info('Service call failed %r' % (e,))
    # else:
    #     node.get_logger().info("move left successful")

    # time.sleep(5)
    #########################################################################################

    # cli = node.create_client(Move, 'move_right')
    # req = Move.Request()
    # req.distance = 100
    # while not cli.wait_for_service(timeout_sec=1.0):
    #     node.get_logger().info('service not available, waiting again...')

    # future = cli.call_async(req)
    # rclpy.spin_until_future_complete(node, future)

    # try:
    #     result = future.result()
    # except Exception as e:
    #     node.get_logger().info('Service call failed %r' % (e,))
    # else:
    #     node.get_logger().info("move right successful")

    # time.sleep(5)
    #########################################################################################

    cli = node.create_client(Video, 'streamon')
    req = Video.Request()

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    try:
        result = future.result()
        cv_image = bridge.imgmsg_to_cv2(result, desired_encoding='passthrough')
    except Exception as e:
        node.get_logger().info('Service call failed %r' % (e,))
    else:
        node.get_logger().info("Video status is shown")
        cv2.imshow('Tello', cv_image)

    time.sleep(5)
    #########################################################################################

    # cli = node.create_client(Empty, 'land')
    # req = Empty.Request()
    # while not cli.wait_for_service(timeout_sec=1.0):
    #     node.get_logger().info('service not available, waiting again...')

    # future = cli.call_async(req)
    # rclpy.spin_until_future_complete(node, future)

    # try:
    #     result = future.result()
    # except Exception as e:
    #     node.get_logger().info('Service call failed %r' % (e,))
    # else:
    #     node.get_logger().info("land successful")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
