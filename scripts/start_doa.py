#!/usr/bin/env python

from esiaf_doa.doa_wrapper import DOA
import pyesiaf
import rospy
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
from esiaf_ros.msg import RecordingTimeStamps, AugmentedAudio

# config
import yaml
import sys

def msg_from_string(msg, data):
    msg.deserialize(data)

nodename = 'esiaf_doa'

# initialize rosnode
rospy.init_node(nodename)
roscpp_init(nodename, [])

# read config
rospy.loginfo('Loading config...')
argv = sys.argv
if len(argv) < 2:
    rospy.logerr('Need path to configfile as first parameter!')
    exit('1')
path_to_config = argv[1]
data = yaml.safe_load(open(path_to_config))

rospy.loginfo('Creating direction of arrival instance...')

wrapper = DOA(data)

# add data publisher

rospy.loginfo('Creating esiaf handler...')
handler = pyesiaf.Esiaf_Handler('esiad_doa', pyesiaf.NodeDesignation.SSL, sys.argv)

rospy.loginfo('Setting up esiaf...')
esiaf_format = pyesiaf.EsiafAudioFormat()
esiaf_format.rate = pyesiaf.Rate.RATE_16000
esiaf_format.bitrate = pyesiaf.Bitrate.BIT_INT_16_SIGNED
esiaf_format.endian = pyesiaf.Endian.LittleEndian
esiaf_format.channels = 1

esiaf_audio_info = pyesiaf.EsiafAudioTopicInfo()
esiaf_audio_info.topic = data['esiaf_input_topic']
esiaf_audio_info.allowedFormat = esiaf_format

rospy.loginfo('adding input topic...')


def input_callback(audio, timeStamps):
    # deserialize inputs
    _recording_timestamps = RecordingTimeStamps()
    msg_from_string(_recording_timestamps, timeStamps)

    # call dao wrapper
    dao = wrapper.process_audio(audio)

    # assemble output

    # publish output



handler.add_input_topic(esiaf_audio_info, input_callback)
rospy.loginfo('input topic added')
handler.start_esiaf()

rospy.loginfo('DOA ready!')
rospy.spin()

handler.quit_esiaf()
