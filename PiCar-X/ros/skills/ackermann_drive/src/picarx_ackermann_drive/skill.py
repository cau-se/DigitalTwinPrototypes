# Copyright 2022 Alexander Barbie
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

from typing import Any, Union
from numpy import angle
import rospy
from std_msgs.msg import Int8
from arches_core.skills import Skill, SkillType
from arches_core.digitalthread import DigitalThread
from picarx_msgs.msg import DriveStatus, MotorStatus, ClutchGearStatus
import argparse
import message_filters

class AckermannStartOptions(object):
    """Start argument parser to the digital shadow skill.
        You have to set the name and a unique id. If you leave out a unique id,
        an autoincrement value will be used as id.

        Note:
            If you use Docker, we recommand to set the ids.
    """

    def __init__(self, argv):
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "name", type=str, help="The name of the digital shadow skill")
        parser.add_argument(
            "uid", type=str, help="The unique identifier of the digital shadow skill. Leave blank if you want and automated generation.")

        self.args = parser.parse_args(argv)

class AckermannDriveSkill(Skill):

    def __init__(self, name: str, uid: str = None) -> None:
        super(AckermannDriveSkill, self).__init__(name, SkillType.CONTROL, uid)
        self.command_subscriber = None
        self.motor_left_publisher = None
        self.motor_right_publisher = None
        self.clutchgeer_publisher = None
        self.status_publisher = None

    @property
    def command_subscriber(self) -> Union[None, rospy.Subscriber]:
        return self.__command_subscriber

    @command_subscriber.setter
    @DigitalThread.control
    def command_subscriber(self, subscriber: rospy.Subscriber) -> Union[None, rospy.Subscriber]:
        if subscriber is None:
            self.__command_subscriber = None
        elif isinstance(subscriber, rospy.Subscriber):
            self.__command_subscriber = subscriber
        else:
            raise ValueError(
                "Subscriber has to be of type rospy.Subscriber or None, but {} was given.".format(type(subscriber)))

    @property
    def status_publisher(self) -> Union[None, rospy.Publisher]:
        return self.__status_publisher

    @status_publisher.setter
    @DigitalThread.data
    def status_publisher(self, publisher: rospy.Publisher) -> Union[None, rospy.Publisher]:
        if publisher is None:
            self.__status_publisher = None
        elif isinstance(publisher, rospy.Publisher):
            self.__status_publisher = publisher
        else:
            raise ValueError(
                "Publisher has to be of type rospy.Publisher or None, but {} was given.".format(type(publisher)))

    @property
    def motor_left_publisher(self) -> Union[None, rospy.Publisher]:
        return self.__motor_left_publisher

    @motor_left_publisher.setter
    def motor_left_publisher(self, publisher: Union[None, rospy.Publisher]) -> Union[None, rospy.Publisher]:
        if publisher is None:
            self.__motor_left_publisher = None
        elif isinstance(publisher, rospy.Publisher):
            self.__motor_left_publisher = publisher
        else:
            raise ValueError(
                "Subscriber has to be of type rospy.Publisher or None, but {} was given.".format(type(publisher)))

    @property
    def motor_right_publisher(self) -> Union[None, rospy.Publisher]:
        return self.__motor_right_publisher

    @motor_right_publisher.setter
    def motor_right_publisher(self, publisher: Union[None, rospy.Publisher]) -> Union[None, rospy.Publisher]:
        if publisher is None:
            self.__motor_right_publisher = None
        elif isinstance(publisher, rospy.Publisher):
            self.__motor_right_publisher = publisher
        else:
            raise ValueError(
                "Subscriber has to be of type rospy.Publisher or None, but {} was given.".format(type(publisher)))

    @property
    def clutchgeer_publisher(self) -> Union[None, rospy.Publisher]:
        return self.__clutchgeer_publisher

    @clutchgeer_publisher.setter
    def clutchgeer_publisher(self, publisher: Union[None, rospy.Publisher]) -> Union[None, rospy.Publisher]:
        if publisher is None:
            self.__clutchgeer_publisher = None
        elif isinstance(publisher, rospy.Publisher):
            self.__clutchgeer_publisher = publisher
        else:
            raise ValueError(
                "Subscriber has to be of type rospy.Publisher or None, but {} was given.".format(type(publisher)))

    def drive(self, ros_msg: Any) -> None:
        # There are problems with understeering in the Ackermann approximation in the simulation, thus we limitate
        # the steering angle to [-20, 20] degree, until the understeering is fixed. Max angle of the physical twin
        # is around |35| degree.
        if ros_msg.angle > 20:
            angle = 20
        elif ros_msg.angle < -20:
            angle = -20
        else:
            angle = ros_msg.angle

        self.motor_left_publisher.publish(Int8(ros_msg.speed))
        self.motor_right_publisher.publish(Int8(ros_msg.speed))
        self.clutchgeer_publisher.publish(Int8(angle))

    def send_status(self, motor_left_status: MotorStatus, motor_right_status:MotorStatus, clutchgear_status:ClutchGearStatus) -> None:
        timestamp = rospy.get_rostime()
        status_message = DriveStatus(timestamp=timestamp, motor_left = motor_left_status, motor_right = motor_right_status, clutchgear=clutchgear_status)
        self.status_publisher.publish(status_message)

    def init_status_filter(self, motor_left_filter, motor_right_filter, steering_filter):
        ts = message_filters.ApproximateTimeSynchronizer([motor_left_filter, motor_right_filter, steering_filter], queue_size=10,slop=0.5, allow_headerless=True)
        ts.registerCallback(self.send_status)
    
    def stop(self):
        rospy.loginfo("SHUTTING DOWN")
