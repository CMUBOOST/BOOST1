#!/usr/bin/env python

from conv_net.srv import *
import rospy

def handle_add_two_ints(req):
    #print "Returning [%s + %s = %s]"%(req.image_process_req)
    return ImageProcessResponse(req.image_process_req)

def add_two_ints_server():
    rospy.init_node('image_process')
    s = rospy.Service('image_process', ImageProcess, handle_add_two_ints)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
