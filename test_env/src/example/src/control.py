#!/usr/bin/env python3

# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

import json
from pynput import keyboard
import math
from RcBrainThread import RcBrainThread
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

import rospy

class RemoteControlTransmitterProcess():
    # ===================================== INIT==========================================
    def __init__(self):
        """Run on the PC. It forwards the commans from the user via KeboardListenerThread to the RcBrainThread. 
        The RcBrainThread converts them into actual commands and sends them to the remote via a socket connection.
        
        """
        self.dirKeys   = ['w', 'a', 's', 'd']
        self.paramKeys = ['t','g','y','h','u','j','i','k', 'r', 'p']
        self.pidKeys = ['z','x','v','b','n','m']

        self.allKeys = self.dirKeys + self.paramKeys + self.pidKeys
        
        self.rcBrain   =  RcBrainThread()   
        
        rospy.init_node('EXAMPLEnode', anonymous=True)     
        self.scan_sub = rospy.Subscriber("/scan_depth", LaserScan, self.laserscan_callback, queue_size=1)
        self.publisher = rospy.Publisher('/automobile/command', String, queue_size=1)
        self.speed_pub = rospy.Publisher('/rear_wheel_speed', Float64, queue_size=10)
        self.steer_pub = rospy.Publisher('/steering_angle', Float64, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_data)
    # ===================================== RUN ==========================================
    def run(self):
        """Apply initializing methods and start the threads. 
        """
        with keyboard.Listener(on_press = self.keyPress, on_release = self.keyRelease) as listener: 
            listener.join()
	
    # ===================================== KEY PRESS ====================================
    def keyPress(self,key):
        """Processing the key pressing 
        
        Parameters
        ----------
        key : pynput.keyboard.Key
            The key pressed
        """                                     
        try:                                                
            if key.char in self.allKeys:
                keyMsg = 'p.' + str(key.char)

                self._send_command(keyMsg)
    
        except: pass
    def laserscan_callback(self, msg: LaserScan):
        """레이저스캔 데이터를 처리하여 주차 공간 점유 여부 확인"""

        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # 글로벌 좌표로 변환된 점들
        global_points = []
        for i, r in enumerate(ranges):
            if msg.range_min < r < msg.range_max:  # 유효 범위 내 데이터만 사용
                alpha = angle_min + i * angle_increment
                x_local = r * math.cos(alpha)
                y_local = r * math.sin(alpha)
                x_global = self.x + x_local * math.cos(self.yaw) - y_local * math.sin(self.yaw)
                y_global = self.y + x_local * math.sin(self.yaw) + y_local * math.cos(self.yaw)
                global_points.append((x_global, y_global))
                rospy.loginfo(global_points)
    # ===================================== KEY RELEASE ==================================
    def keyRelease(self, key):
        """Processing the key realeasing.
        
        Parameters
        ----------
        key : pynput.keyboard.Key
            The key realeased. 
        
        """ 
        if key == keyboard.Key.esc:                        #exit key      
            self.publisher.publish('{"action":"3","steerAngle":0.0}')   
            return False
        try:                                               
            if key.char in self.allKeys:
                keyMsg = 'r.'+str(key.char)

                self._send_command(keyMsg)
    
        except: pass                                                              
                 
    # ===================================== SEND COMMAND =================================
    def _send_command(self, key):
        """Transmite the command to the remotecontrol receiver. 
        
        Parameters
        ----------
        inP : Pipe
            Input pipe. 
        """
        command = self.rcBrain.getMessage(key)
        if command is not None:
	
            command = json.dumps(command)
            self.publisher.publish(command)  

    def publish_data(self, event=None):
        self.speed_pub.publish(Float64(self.rcBrain.speed))
        self.steer_pub.publish(Float64(-self.rcBrain.steerAngle))
if __name__ == '__main__':
    try:
        nod = RemoteControlTransmitterProcess()
        nod.run()
    except rospy.ROSInterruptException:
        pass