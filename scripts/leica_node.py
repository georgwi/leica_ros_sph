#!/usr/bin/env python

# ===========================================================================
#
# Leica - ROS
# This is a bridge for spherical measurements with the Leica Totalstation.
# Spherical measurements allow to take advantage of the fast angular
# measurements. With this node angular measurements will be published with
# about 20Hz and a radial distance with about 7Hz
#
# ===========================================================================

import roslib; roslib.load_manifest('leica_ros_sph')
import rospy
import sys
import time

import GeoCom_mod
from leica_ros_sph.msg import AngleMeasStamped
from leica_ros_sph.msg import RadialMeasStamped
from geometry_msgs.msg import PointStamped
from optparse import OptionParser
from math import sin,cos

# Handling options
usage = "usage: rosrun leica_interface %prog [options]"
parser = OptionParser(usage=usage)
parser.set_defaults(coordinit=False, carthesian=False, big_prism=False, port="/dev/ttyUSB0", verbose=False)
parser.add_option("-i", "--init", action="store_true", dest="coordinit", help="initialize the x-axis")
parser.add_option("-c", "--carth", action="store_true", dest="carthesian", help="activate publishing of carthesian coordinates")
parser.add_option("-b", "--big", action="store_true", dest="big_prism", help="set the big prism as prism type [default: mini prism]")
parser.add_option("-p", "--port", action="store", type="string", dest="port", help="specify used port [default: %default]")
parser.add_option("-v", "--verbose", action="store_true", dest="verbose", help="print more information")
(options, args) = parser.parse_args()

# Set up the serial connection and the Leica TS:
if options.verbose:
  print "Initializing Leica TS"
if GeoCom_mod.COM_OpenConnection(options.port, 115200)[0]:
  sys.exit("Can not open Port... exiting")
prism_type = 7
if options.big_prism:
  prism_type = 3;
  if options.verbose:
    print "Using the big prism"
GeoCom_mod.BAP_SetPrismType(prism_type)
GeoCom_mod.AUT_LockIn()
if options.coordinit:
  GeoCom_mod.TMC_SetOrientation()
  if options.verbose:
    print "Initialized coordinate system"
GeoCom_mod.TMC_SetEdmMode(9)
GeoCom_mod.TMC_DoMeasure()
time.sleep(3)
if options.verbose:
  print "Leica TS is set up"

# Set up ROS:
# We publish to two different nodes for the angular measurement and the
# distance measurement have different rates, precisions and delays.
# Publishing in two nodes allows us to use all the information we have
# in the best way.
rospy.init_node('leica_node')
pub_angle = rospy.Publisher('leica/angles', AngleMeasStamped)
pub_radius = rospy.Publisher('leica/radius', RadialMeasStamped)
if options.carthesian:
  pub_carth = rospy.Publisher('leica/position',PointStamped)

# Set some initial parameters:
# Since we do not want to publish the same measurement twice we assume that
# with every new measurement the number we recieve chances. This assumtion
# is reasonable since the precision of a double is much higher, than the
# precision of the measurement.
theta_prev = 0
phi_prev = 0
radius_prev = 0
# We want to request angular measurements three times while we request only
# one distance measurements. Therefore we introduce a loop counter.
loop_count = 1

# Start the loop that is requesting measurements
if options.verbose:
  print "ROS-node is set up"
if options.carthesian:
  print "Publishing spherical and carthesian coordinates"
else:
  print "Publishing spherical coordinates"
while not rospy.is_shutdown():
  try:
    # Get the angles seperately from the distance measurement, since getting
    # them together causes the rate of theta to slow down
    [error, RC, coord] = GeoCom_mod.TMC_GetAngle()
    phi = -float(coord[0])
    theta = float(coord[1])

    angle_meas = AngleMeasStamped()
    angle_meas.header.stamp = rospy.Time.now()
    if theta != theta_prev and phi != phi_prev:
      angle_meas.phi = phi
      angle_meas.theta = theta
      pub_angle.publish(angle_meas)

    theta_prev = theta
    phi_prev = phi

    # Now request a complete measurement. Do this loop only every second time
    # since the angular measurements are done much faster and we do not want
    # to block them with the request for the complete measurement
    loop_count = loop_count + 1
    if loop_count%2:
      [error, RC, coord] = GeoCom_mod.TMC_GetSimpleMea(5, 1)
      phi = -float(coord[0])
      theta = float(coord[1])
      radius = float(coord[2])

      radius_meas = RadialMeasStamped()
      angle_meas = AngleMeasStamped()
      radius_meas.header.stamp = rospy.Time.now()
      angle_meas.header.stamp = radius_meas.header.stamp
      
      if radius != radius_prev:
        radius_meas.radius = radius
        radius_meas.header.seq = loop_count;
        pub_radius.publish(radius_meas)
        
        if options.carthesian:
          carth_meas = PointStamped()
          carth_meas.header.stamp = radius_meas.header.stamp
          carth_meas.point.x = sin(theta) * cos(phi) * radius
          carth_meas.point.y = sin(theta) * sin(phi) * radius
          carth_meas.point.z = cos(theta) * radius
          carth_meas.header.seq = loop_count;
          pub_carth.publish(carth_meas)
      
      if theta != theta_prev and phi != phi_prev:
        angle_meas.phi = phi
        angle_meas.theta = theta
        angle_meas.header.seq = loop_count;
        pub_angle.publish(angle_meas)

      theta_prev = theta
      phi_prev = phi
      radius_prev = radius

  # We do not want the program to crash if the above code causes an error.
  # Possible causes are:
  # - Mistake in line read from serial port due to high request rate
  # - Loss of prism resulting in unexpected return values
  # - ...
  except ValueError:
    print "Non numeric value recieved!"
  except:
    print "No measurement or drop."
    # Short break in case the problem was related to the serial connection.
    time.sleep(0.3)
    # Then restart the measurement
    GeoCom_mod.TMC_DoMeasure()
    if options.verbose:
      print "Restarted measurements"


# Closing serial connection, when execution is stopped
GeoCom_mod.COM_CloseConnection()
