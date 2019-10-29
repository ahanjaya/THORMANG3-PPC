#!/usr/bin/env python

import os
import rospy
import rospkg
from gtts import gTTS
from std_msgs.msg import String

class TTS:
    def __init__(self):
        rospy.init_node("text_to_speech")
        rospy.loginfo("[TTS] Text to Speech Running")

        rospack           = rospkg.RosPack()
        self.sound_path   = rospack.get_path("thormang3_sensors") + "/scripts/tts.mp3"

        # Subscriber
        rospy.Subscriber('/robotis/sensor/text_to_speech', String, self.tts_callback)

    def tts_callback(self, msg):
        text = msg.data
        tts  = gTTS(text=text, lang='en')
        tts.save(self.sound_path)
        os.system("mpg321 {}".format(self.sound_path))
  
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    tts = TTS()
    tts.run()