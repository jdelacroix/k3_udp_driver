#
# Author: Jean-Pierre de la Croix
# Last Modified: 21 November 2011
#
# Originally:
# (c) 2006 - 2008 EPFL, Lausanne, Switzerland
# Thomas Lochmatter
#

# List of standard modules that this program needs
MODULES	:= khepera3 commandline i2cal odometry_track

# List of modules that you created on your own (and path to your modules)
#MY_MODULES	:=

# The file name of the executable
TARGET	:= k3_udp_driver

# If the K3_ROOT environment variable is set, you can comment out the following line
#K3_ROOT	:= /home/jdelacroix/K3/khepera3toolbox/Scripts/..

# Include the Makefile
include $(K3_ROOT)/Programs/Makefile.include
