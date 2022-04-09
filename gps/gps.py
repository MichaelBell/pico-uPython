# GPS display for Badger2040

import time

import pa1010
import badger2040

badger2040.system_speed(badger2040.SYSTEM_SLOW)

GPS_UDPATE_RATE = 5  # Request a position fix every 5 seconds

# Create the GPS, set update rate and turn off the flashing green LED
gps = pa1010.PA1010()
gps.set_update_rate(GPS_UDPATE_RATE)
gps.set_pps(False)

# It's nice to have a badger on the Badger
BADGER_IMAGE = bytearray((88 * 108) // 8)
open("badger_crop.bin", "rb").readinto(BADGER_IMAGE)

# Create badger display.  Fast update is fine as there isn't much change between updates
display = badger2040.Badger2040()
display.update_speed(badger2040.UPDATE_FAST)

    
def display_time_pos():
    display.pen(15)
    display.rectangle(0, 0, 192, 128)
    display.pen(2)
    display.rectangle(192, 0, 104, 128)
    display.pen(0)
    display.image(BADGER_IMAGE, w=88, h=108, x=200, y=10)
    
    if gps.valid:
        date = "{day:02d}/{month:02d}/{year:04d}".format(**gps.__dict__)
        time = "{hour:02d}:{minute:02d}:{second:02d}".format(**gps.__dict__)
        lat = "{}  {}° {:.3f}".format(gps.latNS, gps.lat[:2], float(gps.lat[2:]))
        lon = "{} {}° {:.3f}".format(gps.lonEW, gps.lon[:3], float(gps.lon[3:]))
        alt = "Alt {:.1f}m".format(gps.altitude)
        satellites = "Tracking {} sats".format(gps.satellites)
    
        display.font("bitmap8")
        display.text(date + " " + time, 5, 5, 2)
        display.text(lat, 20, 30, 2)
        display.text(lon, 20, 50, 2)
        display.text(alt, 20, 70, 2)
        display.text(satellites, 5, 95, 2)
        
    else:
        display.font("sans")
        display.text("GPS invalid", 5, 60, 1)

    display.update()

valid = False
next_display = 0

while True:
    gps.update()
    if gps.valid and not valid:
        valid = True
        next_display = gps.second
    
    if valid:
        if gps.second >= next_display and gps.second - next_display < 30:
            next_display = (gps.second + 10) % 60
            display_time_pos()
        else:
            time.sleep(GPS_UDPATE_RATE)
    
    else:
        if next_display == 0:
            display_time_pos()
        next_display = (next_display + 1) % 10
        time.sleep(GPS_UDPATE_RATE)
