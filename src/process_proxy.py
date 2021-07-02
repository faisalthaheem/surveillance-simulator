#!/usr/bin/python3

__author__ =  'Faisal Thaheem <10938481+faisalthaheem@users.noreply.github.com>'
__version__=  '0.1'
__license__ = 'GPLV3'

#standard imports
import os
from os import popen
import sys, time, subprocess, argparse

#ros
import rospy

def main(args):

    NODE_NAME = os.environ.get('NODE_NAME')
    BIN_PATH = os.environ.get('BIN_PATH')
    ARGS = os.environ.get('ARGS')

    print("NODE_NAME [{}]".format(NODE_NAME))
    print("BIN_PATH [{}]".format(BIN_PATH))
    print("ARGS [{}]".format(ARGS))

    rospy.init_node(NODE_NAME, anonymous=True)

    process = subprocess.Popen([BIN_PATH, ARGS])

    try:
        rospy.spin()    
    except KeyboardInterrupt:
        pass

    process.kill()


if __name__ == '__main__':
    main(sys.argv)