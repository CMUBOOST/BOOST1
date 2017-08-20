#!/usr/bin/env python

from conv_net.srv import *
import rospy

def handle_image_process(req):
    #Insert Tanvir's code here
    #define your output image as resp.image_process_resp
    #replace the output of ImageProcessResponse with resp.image_process_resp
    return ImageProcessResponse(req.image_process_req)

def image_process_server():
    rospy.init_node('image_process')
    s = rospy.Service('image_process', ImageProcess, handle_image_process)
    print "Ready to process the image."
    rospy.spin()

if __name__ == "__main__":
    image_process_server()
