import numpy as np
from ros_pb2 import ROSMessage
from google.protobuf.struct_pb2 import Struct


def get_ros_message(msg):
    ros_message = ROSMessage()
    ros_message.type = "std_msgs/String"
    ros_message = ROSMessage(content=msg)
    return ros_message