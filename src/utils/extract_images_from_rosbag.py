import argparse
import rospy
import rosbag
import cv2
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import os

class ImageFromBag:

    def __init__(self, bag_path, output_path):
        self.bridge = CvBridge()

        # Extract filename from the bag path
        bag_filename = os.path.splitext(os.path.basename(bag_path))[0]

        with rosbag.Bag(bag_path) as bag:
            for idx, (topic, msg, t) in enumerate(bag.read_messages(topics=['/uav0/color/image_raw'])):
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg)
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

                    if idx == 0:
                        # Get image dimensions for video writer
                        height, width, channels = cv_image.shape
                        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # You can also use 'XVID', 'MJPG', 'DIVX', etc.
                        out = cv2.VideoWriter(output_path + bag_filename + '.mp4', fourcc, 15.0, (width, height))

                    # Write the frame to the video
                    out.write(cv_image)
                    print(f"Added frame {idx} to video")

                except CvBridgeError as e:
                    print(e)

        out.release()
        print("Video saved successfully")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract images from a ROS bag file.')
    parser.add_argument('bag_path', type=str, help='Path to the ROS bag file')
    parser.add_argument('output_path', type=str, help='Path to the output directory')
    args = parser.parse_args()

    try:
        image_from_bag = ImageFromBag(args.bag_path, args.output_path)
    except rospy.ROSInterruptException:
        pass
