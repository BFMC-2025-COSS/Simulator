#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class ParkingTriggerNode:
    def __init__(self):
        # 노드 초기화
        rospy.init_node('parking_trigger_node', anonymous=True)
        
        # 퍼블리셔 설정: /parking_trigger 토픽에 String 메시지 발행
        self.trigger_pub = rospy.Publisher('/parking_trigger', String, queue_size=1)
        
        # 미리 정의된 주차 목표 (x, y, yaw)
        self.parking_target = (10.06, 1.35, 0)  # 예: (5.0, 2.0), 90도

    def run(self):
        # 주차 명령 메시지 생성
        message = f"start_parking,{self.parking_target[0]},{self.parking_target[1]},{self.parking_target[2]}"
        
        # 메시지 발행
        self.trigger_pub.publish(message)
        rospy.loginfo(f"주차 트리거 발행: {message}")
        
        # 메시지 전송 보장을 위해 잠시 대기
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        node = ParkingTriggerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass