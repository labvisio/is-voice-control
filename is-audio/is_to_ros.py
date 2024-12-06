import numpy as np
from ros_pb2 import ROSMessage
from google.protobuf.struct_pb2 import Struct


def get_ros_message(msg):
    ros_message = ROSMessage()
    ros_message.type = "std_msgs/String"
    msg_dict = {
        'data': msg
    }
    string_msg = Struct()
    string_msg.update(msg_dict)
    ros_message = ROSMessage(content=string_msg)
    ros_message.type = "std_msgs/String"

    return ros_message