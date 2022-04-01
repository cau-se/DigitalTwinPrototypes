#!/usr/bin/env python3
#
# Copyright 2021-2022 Alexander Barbie
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
from std_msgs.msg import Int8
from picarx_msgs.msg import Test
from picarx_ackermann_drive.skill import AckermannDriveSkill, AckermannStartOptions
class AckermannDriveNode(AckermannDriveSkill):

    def __init__(self, name: str = 'AckermannSkill', uid: str = None) -> None:
        super(AckermannDriveNode, self).__init__(
            name, uid)

    def start(self) -> None:
        rospy.init_node("Ackermann driving Skill", anonymous=False)
        rospy.on_shutdown(self.stop)
        self.command_subscriber = rospy.Subscriber(
            rospy.get_param('~command_topic'), Test, self.drive)
        self.motor_left_publisher = rospy.Publisher(
            rospy.get_param('~motor_left_topic'), Int8, queue_size=5)
        self.motor_right_publisher = rospy.Publisher(
            rospy.get_param('~motor_right_topic'), Int8, queue_size=5)
        self.clutchgeer_publisher = rospy.Publisher(
            rospy.get_param('~steering_topic'), Int8, queue_size=5)

        rospy.spin()


if __name__ == "__main__":
    start_arguments = AckermannStartOptions(rospy.myargv()[1:])
    driving_skill = AckermannDriveNode(start_arguments.args.name, start_arguments.args.uid)
    driving_skill.start()
