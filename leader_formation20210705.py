# for vehicle 1
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import threading
import math
import time
import datetime
import serial
import struct
import logging
import sys

###########################################
CMD_LAT_LEAD = 39.3692619
CMD_LON_LEAD = 115.9152790
CMD_ALT_LEAD = 15 # relative altitude

CMD_LAT_FOLLOW = 39.3692659
CMD_LON_FOLLOW = 115.9154827
CMD_ALT_FOLLOW = 10 # relative alttitude
###########################################

# Arms vehicle and fly to aTargetAltitude, revised by tongbingda
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    logging.info("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        logging.info(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    logging.info("Arming motors")
    vehicle.armed = True    

    while not vehicle.armed:      
        logging.info(" Waiting for arming...")
        time.sleep(1)

    logging.info("Taking off!")
    # Copter should arm in GUIDED mode
    vehicle.mode = "GUIDED"
    time.sleep(1)
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        logging.info(" Altitude: " + str(vehicle.location.global_relative_frame.alt))      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            logging.info("Reached target altitude")
            break
        time.sleep(1)

# calculate_distance(cmd_lat, cmd_lng) in meters, revised by tongbingda
def calculate_distance(cmd_lat, cmd_lng, loc_lat, loc_lng):
    earth_radius = 6371.393
    # convert command position into randians
    cmd_lat_rad = math.radians(cmd_lat)
    cmd_lng_rad = math.radians(cmd_lng)
    # convert local position into radians
    loc_lat_rad = math.radians(loc_lat)
    loc_lng_rad = math.radians(loc_lng)
    # calculate difference value between command position and local position
    diff_lat = abs(cmd_lat_rad - loc_lat_rad)
    diff_lng = abs(cmd_lng_rad - loc_lng_rad)
    # use haversine formula to calculate h, h is the great circle distance in radians
    h = haver_sin(diff_lat) + math.cos(cmd_lat_rad) * math.cos(loc_lat_rad) * haver_sin(diff_lng)
    distance = 2 * earth_radius * math.asin(math.sqrt(h)) * 1e3
    return distance

# haver sine formula, see reference at https://en.wikipedia.org/wiki/Haversine_formula
def haver_sin(theta):
    out_sqrt = math.sin(theta / 2)
    out = out_sqrt * out_sqrt
    return out

# Calculate the azimuth angle in radians between two GPS coordinates
def calculate_bearing(cmd_lat, cmd_lng, loc_lat, loc_lng):
    earth_radius = 6371.393
    # convert command position into randians
    cmd_lat_rad = math.radians(cmd_lat)
    cmd_lng_rad = math.radians(cmd_lng)
    # convert local position into radians
    loc_lat_rad = math.radians(loc_lat)
    loc_lng_rad = math.radians(loc_lng)

    offset_lng = cmd_lng_rad - loc_lng_rad
    X = math.cos(cmd_lat_rad) * math.sin(offset_lng)
    Y = math.cos(loc_lat_rad) * math.sin(cmd_lat_rad) - math.sin(loc_lat_rad) * math.cos(cmd_lat_rad) * math.cos(offset_lng)

    beta = math.atan2(X, Y)
    return beta

# Calculate the GPS position of the commanded location by relative position, distance in meters, bearing in radians
def calculate_latlng(distance, bearing, loc_lat, loc_lng):
    earth_radius = 6371.393
    delta = (distance * 0.001) / earth_radius
    cmd_lat = math.asin(math.sin(loc_lat) * math.cos(delta) + math.cos(loc_lat) * math.sin(delta) * math.cos(bearing))
    cmd_lng = loc_lng + math.atan2(math.sin(bearing) * math.sin(delta) * math.cos(loc_lat),math.cos(delta) - math.sin(loc_lat) * math.sin(cmd_lat))
    return cmd_lat, cmd_lng 

# Move vehicle in direction based on specified velocity vectors
def send_global_velocity(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.

    This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).
    
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version 
    (sending the message multiple times does not cause problems).
    
    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    
    vehicle.send_mavlink(msg)

# The program's entry point is here:
# Save infomation in the log file
strtime = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
logging.basicConfig(level=logging.DEBUG,
                    filename='vehicle' + strtime + '.log',
                    filemode='w',
                    format='%(asctime)s - %(pathname)s[line:%(lineno)d] - %(levelname)s: %(message)s'
                    )

# Connect to the Vehicle
try:
    vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=115200)
except Exception as e:
    logging.error("Vehicle connection error.")
    logging.error(e)
    sys.exit(0)
else:
    logging.info("The vehicle is successfully connected.")

# Connect to RFD900x telem
try:
    telem = serial.Serial("/dev/ttyRFD900x", 115200)
    logging.info("Telem connected at a baud rate of 115200.")
except Exception as e:
    logging.error("Telem connection error.")
    logging.error(e)
    sys.exit(0)

print("Waiting for takeoff command.")

# One-click takeoff, vehicle must in loiter mode and telem must be connected, revised by tongbingda
while True:
    command = telem.read(1)
    if command == b'\xAA':
        logging.info("Takeoff command received.")
        break
    time.sleep(1)

# Take off and go to the command location.
try:
    arm_and_takeoff(CMD_ALT_LEAD)
except Exception as e:
    logging.error("Takeoff error.")
    logging.error(e)
    vehicle.mode = VehicleMode("LAND")
    sys.exit(0)

# Command the vehicle to fly to the target location. 
command_location = LocationGlobalRelative(CMD_LAT_LEAD, CMD_LON_LEAD, CMD_ALT_LEAD)
vehicle.simple_goto(command_location)
logging.info("Go to the command location.")

# Follow-up code can only be executed when arriving near the predetermined location.
while calculate_distance(CMD_LAT_LEAD, CMD_LON_LEAD, vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon) > 5:
    location_logging = "Latitude = " + str(vehicle.location.global_relative_frame.lat) + " Longitude =" + str(vehicle.location.global_relative_frame.lon)
    logging.info(location_logging)
    time.sleep(0.5)
logging.info("Vehicle has arrived the destination.")

# The latitude and longitude position coordinates of the vehicle should broadcast at this time.

# From here, the satellite should turn on the camera to identify the target.
while vehicle.mode.name != "LAND":
    vehicle.mode = "LAND" 


# Close vehicle object before exiting script
logging.info("Close vehicle object")
vehicle.close()
logging.info("Mission completed.")