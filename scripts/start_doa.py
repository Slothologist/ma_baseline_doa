#!/usr/bin/env python

from esiaf_doa.doa_wrapper import DOA
import genpy
import rospy
import alsaaudio
from threading import Lock, Thread
import numpy as np
from collections import deque
from pepper_clf_msgs.srv import SynchronizedSSL, SynchronizedSSLResponse

# config
import yaml
import sys



nodename = 'ma_baseline_doa'

# initialize rosnode
rospy.init_node(nodename)

# read config
rospy.loginfo('Loading config...')
argv = sys.argv
if len(argv) < 2:
    rospy.logerr('Need path to configfile as first parameter!')
    exit('1')
path_to_config = argv[1]
data = yaml.safe_load(open(path_to_config))


queue = deque(100 * [(rospy.Time(), 0.0)])
mutex = Lock()


def handle_ssl_request(request):
    start = request.start
    end = request.end
    angles = []
    min_diff = genpy.Duration(sys.maxint)
    mutex.acquire()
    for value in queue:
        if start < value[0] < end:
            angles.append(value[1])
        else:
            before = start - value[0]
            after = end - value[0]
            if min(abs(before), abs(after)) < abs(min_diff):
                min_diff = before if abs(before) < abs(after) else after

    mutex.release()
    response = SynchronizedSSLResponse()
    rospy.loginfo('>>> Got request from %.2f to %.2f' % (start.to_sec(), end.to_sec()))
    if start > end:
        rospy.loginfo('>>> WARNING: start > end!!! this cannot work')
    if len(angles) > 0:
        response.angle = float(sum(angles)) / float(len(angles))
        response.valid = True
        rospy.loginfo('>>> Found %d matches' % len(angles))
    else:
        response.valid = False
        rospy.loginfo('>>> No match in queue. min. diff: %.2f' % min_diff.to_sec())
    return response


# prepare audio grabber
recorder = alsaaudio.PCM(alsaaudio.PCM_CAPTURE, alsaaudio.PCM_NONBLOCK)
CHANNELS = 4
INFORMAT = alsaaudio.PCM_FORMAT_S16_LE
RATE = 16000
FRAMESIZE = 512
recorder.setchannels(CHANNELS)
recorder.setrate(RATE)
recorder.setformat(INFORMAT)
recorder.setperiodsize(FRAMESIZE)

ssl_service = rospy.Service('pepper_ssl_service', SynchronizedSSL, handle_ssl_request)

rospy.loginfo('Creating direction of arrival instance...')
wrapper = DOA(data, plot=data['plot_spectra'])


def grab_thread():
    while not rospy.is_shutdown():
        length, data = recorder.read()
        usable_data = np.fromstring(data, dtype='int16')
        interleaved_data = np.array([[usable_data[4*x], usable_data[4*x+1], usable_data[4*x+2], usable_data[4*x+3]] for x in range(len(usable_data)//4)])
        deinterleaved_data = interleaved_data.transpose()
        doa = wrapper.process_audio(deinterleaved_data)

        mutex.acquire()
        queue.append((rospy.Time.now(), doa))
        queue.popleft()
        mutex.release()


t = Thread(target=grab_thread)
t.start()

rospy.loginfo('DOA ready!')
rospy.spin()
rospy.loginfo('Exiting...')