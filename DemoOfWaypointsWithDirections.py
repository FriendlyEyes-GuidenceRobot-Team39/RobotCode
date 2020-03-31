from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command, Vehicle
import time
import math
from pymavlink import mavutil
from dronekit_sitl import SITL

#Creating a simulated rover at the home location of the Rover, if not SITL it will auto update
sitl = SITL()
sitl.download('rover', '2.50', verbose=True)
sitl_args = ['-I0', '--model', 'rover', '--home=30.606623,-96.325931,0,0']
sitl.launch(sitl_args, await_ready=False, restart=True)
connection_string = 'tcp:127.0.0.1:5760'

print("Sitl: %s" % sitl_args)

print("Connecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=True, baud=57600)
time.sleep(1)


while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def readmission(aFileName):
    """
    Load a mission from a file into a list. The mission definition is in the Waypoint file
    format (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    This function is used by upload_mission().
    """
    print("\nReading mission from file: %s" % aFileName)
    cmds = vehicle.commands
    missionlist=[]
    with open(aFileName) as f:
        for i, line in enumerate(f):
            if i==0:
                if not line.startswith('QGC WPL 110'):
                    raise Exception('File is not supported WP version')
            else:
                linearray=line.split('\t')
                ln_index=int(linearray[0])
                ln_currentwp=int(linearray[1])
                ln_frame=int(linearray[2])
                ln_command=int(linearray[3])
                ln_param1=float(linearray[4])
                ln_param2=float(linearray[5])
                ln_param3=float(linearray[6])
                ln_param4=float(linearray[7])
                ln_param5=float(linearray[8])
                ln_param6=float(linearray[9])
                ln_param7=float(linearray[10])
                ln_autocontinue=int(linearray[11].strip())
                cmd = Command( 0, 0, 0, ln_frame, ln_command, ln_currentwp, ln_autocontinue, ln_param1, ln_param2, ln_param3, ln_param4, ln_param5, ln_param6, ln_param7)
                missionlist.append(cmd)
    return missionlist


def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

def upload_mission(aFileName):
    """
    Upload a mission from a file. 
    """
    #Read mission from file
    missionlist = readmission(aFileName)
    
    print("\nUpload mission from a file: %s" % aFileName)
    #Clear existing mission from vehicle
    print(' Clear mission')
    cmds = vehicle.commands
    cmds.clear()
    #Add new mission to vehicle
    for command in missionlist:
        cmds.add(command)
    print(' Upload mission')
    vehicle.commands.upload()

def download_mission():
    """
    Downloads the current mission and returns it in a list.
    It is used in save_mission() to get the file information to save.
    """
    print(" Download mission from vehicle")
    missionlist=[]
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() #wait until download is complete
    for cmd in cmds:
        missionlist.append(cmd)
    return missionlist



		
def number_of_waypoints(missionlist):
	"""
	Find the number of waypoints to help end the mission
	"""
	
	NumberOfWayPoints = len(missionlist)
	print (' Number of Waypoints: %x' % NumberOfWayPoints)
	
def create_dummy_waypoint(NumberOfWayPoints):
	"""
	Will end the route once the rover has reached its final waypoint by creating a dummy one."
	"""
	cmds = vehicle.commands
	#extracmds = vehicle.commands(NumberOfWayPoints)
	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 30.60724360,-96.32654790,100.000000, 1))
	#cmds.add(Command( 0, 0, 0, 16, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 0))
	
    
def condition_yaw(heading, relative=True):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).
    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.
    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)
    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
	

def save_mission(aFileName):
    """
    Save a mission in the Waypoint file format 
    (http://qgroundcontrol.org/mavlink/waypoint_protocol#waypoint_file_format).
    """
    print("\nSave mission from Vehicle to file: %s" % aFileName)    
    #Download mission from vehicle
    missionlist = download_mission()
    #Add file-format information
    output='QGC WPL 110\n'
    #Add home location as 0th waypoint
    home = vehicle.home_location
    output+="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (0,1,0,16,0,0,0,0,home.lat,home.lon,home.alt,1)
    #Add commands
    for cmd in missionlist:
        commandline="%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (cmd.seq,cmd.current,cmd.frame,cmd.command,cmd.param1,cmd.param2,cmd.param3,cmd.param4,cmd.x,cmd.y,cmd.z,cmd.autocontinue)
        output+=commandline
    with open(aFileName, 'w') as file_:
        print(" Write mission to file")
        file_.write(output)
	
import_mission_filename = 'ToTimber.txt'

#Upload mission from file
upload_mission(import_mission_filename)

#if not vehicle.home_location:
#my_location = vehicle.location.global_frame
#vehicle.home_location = my_location

#sets home location for the RTK on the pixhawk to work
my_location_alt=vehicle.location.global_frame
my_location_alt.alt=0
vehicle.home_location=my_location_alt

print (' Home Location: %s' % vehicle.home_location)

print(' Location: %s' % vehicle.location.global_frame)

number_of_waypoints(vehicle.commands)

NumberOfLegs = len(vehicle.commands)

create_dummy_waypoint(NumberOfLegs)
print(' Added Dummy Waypoint')
#shows that we have added a waypoint
number_of_waypoints(vehicle.commands)

AdditionalLeg = len(vehicle.commands)

#disarm the vehicle
vehicle.armed = False

#set the default groundspeed to be used in movement commands
#vehicle.groundspeed = 1.0  

print(" Groundspeed: %s" % vehicle.groundspeed)    # settable

print("Basic pre-arm checks")
	# Don't let the user try to arm until autopilot is ready
while not vehicle.is_armable:
	print(" Waiting for vehicle to initialise...")
	time.sleep(1)

print("Starting mission")
# Reset mission set to first (0) waypoint
vehicle.commands.next=0
time.sleep(1)

#vehicle.parameters['WPNAV_SPEED'] = 100  
original_time = time.time()
stop_time1 = original_time + 30.0000
stop_time2 = original_time + 60.0000

time.sleep(1)
#print('Starting Time: %s , stop time 1: %s , stop time 2: %s' % (original_time, stop_time1, stop_time2))
# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")
time.sleep(1)

# Monitor mission. 
# Demonstrates getting and setting the command number 
# Uses distance_to_current_waypoint(), a convenience function for finding the 
#   distance to the next waypoint.
print(" Groundspeed: %s" % vehicle.groundspeed)    # settable
time.sleep(1)
'''
if time.time() == 30:
	print('Object Detected')
	vehicle.mode = VehicleMode("HOLD")
'''	

YawOld = vehicle.attitude.yaw * 180 / (math.pi*2)

while True:
    nextwaypoint=vehicle.commands.next
    print('Distance to waypoint (%s): %s, GPS Coords: %s' % (nextwaypoint, distance_to_current_waypoint(), vehicle.location.global_frame ))
    #print('time: %s' % (time.time()))
	#print(' Longitude: (%s) ', vehicle.location.global_relative_frame.lon)
	#print(' Latitude: (%s) ', vehicle.location.global_relative_frame.lat)
	
	#this will make the rover hold when the user is out of frame
    TrackingFunction()
	#if the function returns a True value the user is out of frame
    if UserTracking == True
        vehicle.mode = VehicleMode("HOLD")
	#system is set to hold to stop the rover
	sleep(1)
	while UserTracking:  #this while loop will keep rover in hold until the user is back in frame
		TrackingFunction()
		if UserTracking == False # once user is back in frame, the rover will go back into auto mode
		    vehicle.mode = VehicleMode("AUTO")
		    break
	break
	
	#this will update user on direction
    if round(time.time())%2 == 0 : #if the time is divisble by 5 it will give directions
        YawNew = vehicle.attitude.yaw * 180 / (math.pi*2)
        direction = YawNew - YawOld
        if direction <= -5 :
            print('Go Left')
        elif direction >= 5:
            print('Go Right')
        else :
            if vehicle.groundspeed < .5 :
                print('Stop')
            else:
                print('Go Straight')
        YawOld = YawNew

    time.sleep(1)
save_mission(attempt1)


#Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()
	
