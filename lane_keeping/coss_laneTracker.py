import glob
import cv2
import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from tracker import LaneTracker

class LaneTrackerNode():
    def __init__(self):
        self.bridge = CvBridge()

        self.lane_tracker = None
        self.i = 1

        rospy.init_node('COSS_LaneTracker', anonymous=True)
        self.image_sub = rospy.Subscriber("/automobile/image_raw", Image, self.callback)
        
        #rospy.spin()
    
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data,"bgr8")

            if self.lane_tracker is None:
                self.lane_tracker = LaneTracker(cv_image)
                rospy.loginfo("LaneTracker initialized with the first frame.")

            processed_frame = self.lane_tracker.process(cv_image)
            
            cv2.imshow("Lane Tracking", processed_frame)

            # 키 입력 대기 (1ms)
            key = cv2.waitKey(1)
            if key == 27:  # ESC 키를 누르면 종료
                rospy.signal_shutdown("ESC key pressed. Shutting down...")
            
            # plt.figure(figsize=(10,3))
            # plt.subplot(1,1,1)
            # plt.title(f"Lane Track Image {self.i}")
            # plt.imshow(processed_frame)
            # plt.show()

            self.i += 1

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()
        
        #self.cv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)

        #lane_tracker = LaneTracker(self.cv_image)

        #processed_frame = lane_tracker.process(self.cv_image)

        

        self.i += 1

if __name__ == '__main__':
    try:
        lanetracker_nod = LaneTrackerNode()
        lanetracker_nod.run()
    except rospy.ROSInterruptException:
        pass
