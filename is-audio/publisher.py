import time
from typing import Literal

from is_wire.core import Channel, Message
from is_to_ros import get_ros_message


class VoiceControl:
    def __init__(self, channel):
        self._channel = Channel(channel)

    def sent_to(self, msg):
        ros_message = get_ros_message(msg)
        message = Message(content=ros_message)
        print(message)
        print(self._channel)
        self._channel.publish(message, topic="ros.voice_msg")
