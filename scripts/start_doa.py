#!/usr/bin/env python

from esiaf_doa.doa_wrapper import DOA
import pyesiaf
import rospy
from esiaf_ros.msg import RecordingTimeStamps, AugmentedAudio, SSLDir, SSLInfo

# config
import yaml
import sys

# util
import StringIO


def msg_to_string(msg):
    buf = StringIO.StringIO()
    msg.serialize(buf)
    return buf.getvalue()


def msg_from_string(msg, data):
    msg.deserialize(data)


nodename = 'esiaf_doa'

# initialize rosnode
rospy.init_node(nodename)
pyesiaf.roscpp_init(nodename, [])

# read config
rospy.loginfo('Loading config...')
argv = sys.argv
if len(argv) < 2:
    rospy.logerr('Need path to configfile as first parameter!')
    exit('1')
path_to_config = argv[1]
data = yaml.safe_load(open(path_to_config))

rospy.loginfo('Adding SSLInfo publisher...')
ssl_publisher = rospy.Publisher(nodename + '/' + 'SSL', SSLInfo, queue_size=10)

rospy.loginfo('Creating direction of arrival instance...')

wrapper = DOA(data)

rospy.loginfo('Creating esiaf handler...')
handler = pyesiaf.Esiaf_Handler(nodename, pyesiaf.NodeDesignation.SSL, sys.argv)

rospy.loginfo('Setting up esiaf...')
esiaf_format = pyesiaf.EsiafAudioFormat()
esiaf_format.rate = pyesiaf.Rate.RATE_16000
esiaf_format.bitrate = pyesiaf.Bitrate.BIT_INT_16_SIGNED
esiaf_format.endian = pyesiaf.Endian.LittleEndian
esiaf_format.channels = len(data['mic_array'])

esiaf_audio_info = pyesiaf.EsiafAudioTopicInfo()
esiaf_audio_info.topic = data['esiaf_input_topic']
esiaf_audio_info.allowedFormat = esiaf_format


rospy.loginfo('adding output topic...')
esiaf_audio_out_info = pyesiaf.EsiafAudioTopicInfo()
esiaf_audio_out_info.topic = data['esiaf_output_topic']
esiaf_audio_out_info.allowedFormat = esiaf_format
handler.add_output_topic(esiaf_audio_out_info)

rospy.loginfo('adding input topic...')


def input_callback(audio, timeStamps):
    # deserialize inputs
    _recording_timestamps = RecordingTimeStamps()
    msg_from_string(_recording_timestamps, timeStamps)

    # call dao wrapper
    doa = wrapper.process_audio(audio.transpose())
    rospy.loginfo('DOA from ' + str(_recording_timestamps.finish.secs) + ': ' + str(doa))

    # assemble output
    doa_esiaf = SSLDir()
    doa_esiaf.angleHorizontal = doa
    doa_esiaf.angleVertical = 0.0
    doa_esiaf.sourceId = 'no tracking'

    doas = [msg_to_string(doa_esiaf)]
    handler.set_ssl_dirs(data['esiaf_output_topic'], doas)

    # publish output
    handler.publish(data['esiaf_output_topic'],
                    audio,
                    timeStamps)

    # assemble output
    output = SSLInfo()
    output.duration = _recording_timestamps
    output.directions = [doa_esiaf]

    ssl_publisher.publish(output)



handler.add_input_topic(esiaf_audio_info, input_callback)
rospy.loginfo('input topic added')
handler.start_esiaf()

rospy.loginfo('DOA ready!')
rospy.spin()

handler.quit_esiaf()