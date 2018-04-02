#!/usr/bin/python

import rospy
import std_msgs


import sys
import copy
import time
import numpy as np
import math as m
from OWL import *
from rospy.numpy_msg import numpy_msg
from numpy.linalg import inv
from geometry_msgs.msg import Point
from phasespace_publisher.msg import PhasespacePt
from phasespace_publisher.msg import PhasespacePtArray

def main():
  init_phasespace()
  main_loop()

def init_phasespace():

  # get PhaseSpace configuration
  marker_count = rospy.get_param('/phasespace_publisher/marker_count', '43')
  server_name = rospy.get_param('/phasespace_publisher/server_name', '192.168.1.6')
  init_flags = rospy.get_param('/phasespace_publisher/init_flags', "OWL_MODE1")

  # evaluate init flags to OWL global variable
  init_flags = eval(init_flags)

  if (owlInit(server_name, init_flags) < 0):
    print "init error: ", owlGetError()
    sys.exit(0)

  # Create tracker
  tracker = 0
  owlTrackeri(tracker, OWL_CREATE, OWL_POINT_TRACKER)
  for i in range(marker_count):
      owlMarkeri(MARKER(tracker, i), OWL_SET_LED, i)
  # activate tracker
  owlTracker(tracker, OWL_ENABLE)

  # set define frequency
  owlSetFloat(OWL_FREQUENCY, OWL_MAX_FREQUENCY)

  # start streaming
  owlSetInteger(OWL_STREAMING, OWL_ENABLE)
  owlSetInteger(OWL_FRAME_BUFFER_SIZE, 0)

def main_loop():
  print "PhaseSpace appears to be set up correctly!"
  print "> Printing data in 5 seconds..."
  print "> CTRL+C to break"
  time.sleep(5)

  pub = rospy.Publisher("/ps_markers/phasespace_points", PhasespacePtArray, queue_size = 10)
  rospy.init_node("phaseSpace")

  while not rospy.is_shutdown():
    # Get markers
    markers = owlGetMarkers()
    num_markers = markers.size()

    # Get rigids
    rigids = owlGetRigids()
    num_rigids = rigids.size()

    # Check for error
    err = owlGetError()
    if (err != OWL_NO_ERROR):
      owl_print_error("error", err)
    else:
      if (num_markers == 0 and num_rigids == 0):
        pass

      if (num_markers > 0):
        for i in range(num_markers):
          if (markers[i].cond > 0):
            marker_loc = "%d) %.2f %.2f %.2f" % (markers[i].id, markers[i].x, markers[i].y, markers[i].z)
            rospy.logdebug(marker_loc)

      if (num_rigids > 0):
        for i in range(num_rigids):
          if (rigids[i].cond > 0):
            rigid_loc =  "%d) %.2f %.2f %.2f %.2f %.2f %.2f %.2f" % (rigids[i].id, rigids[i].pose[0], rigids[i].pose[1], rigids[i].pose[2], rigids[i].pose[3], rigids[i].pose[4], rigids[i].pose[5], rigids[i].pose[6])
            rospy.logdebug(ridid_loc)

    final = PhasespacePtArray()
    if (num_markers > 0):
      for i in range(num_markers):
        if (markers[i].cond > 0):
          orig = np.matrix([[markers[i].x], [markers[i].y], [markers[i].z], [1]])
          new_pt = translate(orig, markers[i].id)
          final.points.append(new_pt)
      pub.publish(final)


  owlDone()



def translate(orig, marker_id):
  tx = 733
  ty = 1000
  tz = -1938-200

  cos90 = np.cos(m.pi/2)
  sin90 = np.sin(m.pi/2)
  cos270 = np.cos(3 * m.pi/2)
  sin270 = np.sin(3 * m.pi/2)

  rx = np.matrix([[1, 0, 0], [0, cos270, -sin270], [0, sin270, cos270]])
  ry = np.matrix([[cos270, 0, sin270], [0, 1, 0], [-sin270, 0, cos270]])
  rz = np.matrix([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

  rotation = rz * ry * rx

  transform = np.matrix([[1, 0, 0, tx], [0, 1, 0, ty], [0, 0, 1, tz], [0, 0, 0, 1]])
  trans_pt = inv(transform) * orig

  transform[0:3, 0:3] = rotation

  new_pt = inv(transform) * orig
  new_loc = "Transformed %d) %.2f %.2f %.2f" % (marker_id, new_pt[0], new_pt[1], new_pt[2])
  rospy.logdebug(new_loc)

  final_pt = Point(new_pt[0]/1000, new_pt[1]/1000, new_pt[2]/1000)
  final = PhasespacePt()
  final.pt = final_pt
  final.id = marker_id
  return final;


def owl_print_error(s, n):
  """Print OWL error."""
  if(n < 0): print "%s: %d" % (s, n)
  elif(n == OWL_NO_ERROR): print "%s: No Error" % s
  elif(n == OWL_INVALID_VALUE): print "%s: Invalid Value" % s
  elif(n == OWL_INVALID_ENUM): print "%s: Invalid Enum" % s
  elif(n == OWL_INVALID_OPERATION): print "%s: Invalid Operation" % s
  else: print "%s: 0x%x" % (s, n)

if __name__ == "__main__":
  main()
