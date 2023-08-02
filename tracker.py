from pymavlink import mavutil
import keyboard
import time
import os.path
from getch import getche, getch
import RPi.GPIO as GPIO
import lib.pitftDisplay as disp
import lib.geoTransform as geo

#GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

enable = 12
step = 13
dir = 16

GPIO.setup(enable, GPIO.OUT)
GPIO.setup(step, GPIO.OUT)
GPIO.setup(dir, GPIO.OUT)

GPIO.output(enable, False)

tracker_pos = geo.geo_coord(0.0, 0.0, 0.0)
precision = 10000000.0
alt_precision = 10000.0

def wait_connection(m):
    disp.print_center('Waiting for connection...')
    msg = m.recv_match(type='SYS_STATUS', blocking=True)
    disp.print_center('Connected')

def connect_serial():
    master = mavutil.mavlink_connection('tcp:0.0.0.0:5601') # Listen to a port connected to mavp2p
    wait_connection(master)
    return master

def wait_for_fix():
    disp.print_center('Waiting for GPS fix')
    fixed = False
    while not fixed:
        gps_data = connection.recv_match(type='GPS_RAW_INT', blocking=True)
        fix = gps_data.fix_type
        if fix >= 3:
            fixed = True
 
def get_gps_data():
    global tracker_pos
    lat_buf = 0
    lon_buf = 0
    alt_buf = 0
    for i in range(10):
        gps_data = connection.recv_match(type='GPS_RAW_INT', blocking=True)
        lat_buf += gps_data.lat
        lon_buf += gps_data.lon
        alt_buf += gps_data.alt
    tracker_pos.lat = float(lat_buf / (10 * precision))
    tracker_pos.lon = float(lon_buf / (10 * precision)) 
    tracker_pos.alt = float(alt_buf / (10 * alt_precision))

def save_tracker_pos(): 
    disp.print_center("Press any key to save tracker's location")
    a = getch()
    get_gps_data()
    LastPos = (str(tracker_pos.lat)+'\n'+str(tracker_pos.lon)+'\n'+str(tracker_pos.alt)+'\n')
    file = open("LastPos.txt", "w")
    file.write(LastPos)
    file.close()

def set_tracker_pos():
    global tracker_pos
    path = './LastPos.txt'
    is_LastPos = os.path.isfile(path)
    if is_LastPos == True:
        disp.print_center_yesno("Saved postion found. Do you want to use it?")
        selection = getch()
        if selection == 'a':
            file = open("LastPos.txt", "r")
            tracker_pos.lat = float(file.readline())
            tracker_pos.lon = float(file.readline())
            tracker_pos.alt = float(file.readline())
            file.close()
            disp.print_center("Position set")
            time.sleep(2)
        else:
            wait_for_fix()
            save_tracker_pos()
    else:
        wait_for_fix()
        save_tracker_pos()

def motor_step():
    GPIO.output(step, True)
    time.sleep(0.001)
    GPIO.output(step, False)

disp.clear_screen()
disp.startup()
disp.clear_screen()
connection = connect_serial()
disp.clear_screen()
set_tracker_pos()
disp.clear_screen()

buf_az = 0.0
buf_incl = 0.0

while True:
    try:
        gps_data = connection.recv_match(type='GPS_RAW_INT', blocking=True)
    except:
        disp.print_center("Connection lost")
        break
    drone_pos = geo.geo_coord(gps_data.lat / precision, gps_data.lon / precision, gps_data.alt / alt_precision)
    track_data = geo.DistAziElev(tracker_pos.lat, tracker_pos.lon, drone_pos.lat, drone_pos.lon)

    distance = track_data[0]
    azimuth = track_data[1]
    inclination = track_data[2]
    
    diff_az = azimuth - buf_az
    diff_incl = inclination - buf_incl
    if abs(diff_az) > 180:
        if diff_az < 0:
            diff_az += 360
        else:
            diff_az -= 360

    deg_in_steps = int(abs(diff_az)/(360/1600)) 

    if diff_az >= 0:
       GPIO.output(dir, False)
    else:
       GPIO.output(dir, True)

    for i in range(deg_in_steps):
       motor_step()
       time.sleep(0.001)

    if distance > 1000:
        #print(f'{str(round(diff_az, 2)):5} {str(round(diff_alt, 2)):5}')
        disp.print_center(f'Az.(deg): {str(round(azimuth, 2)):7} Dist.(km): {str(round(distance / 1000, 2)):7} Incl.(deg): {str(round(inclination, 2)):7}')
    else:
        #print(f'{str(round(diff_az, 2)):5} {str(round(diff_alt, 2)):5}')
        disp.print_center(f'Az.(deg): {str(round(azimuth, 2)):7} Dist.(m): {str(round(distance, 2)):7} Incl.(deg): {str(round(inclination, 2)):7}')
