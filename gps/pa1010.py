# Heavily inspired by the Pimoroni library: https://github.com/pimoroni/pa1010d-python

import machine
import time
import re

class GPSTimeoutError(Exception):
    pass

class PA1010:
    """Support for GPS PA1010 module"""
    
    def update(self):
        """Call update periodically to update data.
        If update returns True, the following class members are populated:
           lat:    Text latitude in ddmm.mmmm format (e.g. 5606.1725)
           latNS:  Either N for North or S for South
           lon:    Text longitude in dddmm.mmmm format (e.g. 01404.0622)
           lonEW:  Either E for East or W for West
           altitude: The altitude of the module in metres (float, quite approximate)
           year:   The year (integer, note has a Y21K bug)
           month:  The month (integer)
           day:    The day (integer)
           hour:   The hour (integer)
           minute: The minute (integer)
           second: The second (integer)
           satellites: The number of satellites being tracked (integer)
           speed:  Speed over ground in knots (float)
           heading: Direction of movement over ground in degrees from true North (float)"""
        try:
            # Keep reading data while any is available to ensure we update
            # to the latest state
            while True:
                self._decode_sentence(self.read_sentence())
        except GPSTimeoutError:
            pass
        
        return self.valid

    def set_pps(self, enable):
        """Enable or disable the pulse per second.
        Disabling it stops the green LED flashing"""
        if enable:
            self.send_command("PMTK285,2,100")
        else:
            self.send_command("PMTK285,0,100")
    
    def set_update_rate(self, seconds_per_update):
        """Set the rate at which the module attempts to update its position,
        in seconds between updates.  Can be fractional, but the range is clamped
        to 0.1 - 10s."""
        ms_per_update = int(seconds_per_update * 1000)
        if ms_per_update < 1000:  ms_per_update = 1000
        if ms_per_update > 10000: ms_per_update = 10000
        
        self.send_command("PMTK300,{},0,0,0,0".format(ms_per_update))

    def cold_boot(self):
        """Cold boot the module, so it forgets all data that helps it start quickly.
        It takes a few minutes to start up after this - useful for testing having no signal"""
        self.send_command("PMTK103")


    GGA_DECODE = re.compile(r"\$GNGGA,(\d\d)(\d\d)(\d\d)\.(\d\d\d),(\d+\.\d+),([NS]),(\d+\.\d+),([EW]),(\d),(\d+),[^,]+,([0-9.]+),")
    RMC_DECODE = re.compile(r"\$GNRMC,(\d\d)(\d\d)(\d\d)\.(\d\d\d),([AV]),(\d+\.\d+),([NS]),(\d+\.\d+),([EW]),([0-9.]+),([0-9.]+),(\d\d)(\d\d)(\d\d),")
    I2C_ADDR = 16
    
    def __init__(self):
        self.i2c = machine.I2C(scl=machine.Pin(5), sda=machine.Pin(4), id=0, freq=400000)
        self.valid = False
        self.satellites = 0
        self.altitude = 0
        
        # Only request GGA and RMC data
        self.send_command("PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
        
    def send_command(self, command, add_checksum=True):
        """Send a command string to the PA1010D.
        If add_checksum is True (the default) a NMEA checksum will automatically be computed and added.
        """
        if type(command) is not bytes:
            command = command.encode("ascii")

        buf = bytearray()
        buf += b'$'
        buf += command
        if add_checksum:
            checksum = 0
            # bytes() is a real thing in Python 3
            # so `for char in commaud` iterates through char ordinals
            for char in command:
                checksum ^= char
            buf += b'*'  # Delimits checksum value
            buf += "{checksum:02X}".format(checksum=checksum).encode("ascii")
        buf += b'\r\n'
        self.i2c.writeto(PA1010.I2C_ADDR, buf)

    def read_sentence(self, timeout=50):
        """Attempt to read an NMEA sentence from the PA1010D."""
        buf = b""
        start = time.ticks_ms()

        while time.ticks_diff(time.ticks_ms(), start) < timeout:
            char = self.i2c.readfrom_mem(PA1010.I2C_ADDR, 0, 1)

            if len(buf) == 0 and char != b'$':
                continue

            buf += char

            # Check for end of line
            # Should be a full \r\n since the GPS emits spurious newlines
            if buf[-2:] == b"\r\n":
                # Remove line ending and spurious newlines from the sentence
                return buf.decode("ascii").strip().replace("\n", "")

        raise GPSTimeoutError("Timeout waiting for readline")

    def _decode_sentence(self, buf):
        m = PA1010.GGA_DECODE.match(buf)
        if m is not None:
            self._set_data_from_gga(m)
        else:
            m = PA1010.RMC_DECODE.match(buf)
            if m is not None:
                self._set_data_from_rmc(m)

    def _set_data_from_rmc(self, m):
        self.hour, self.minute, self.second, self.milli = [int(x) for x in m.groups()[:4]]
        self.valid = m.group(5) == "A"
        self.lat, self.latNS, self.lon, self.lonEW = m.groups()[5:9]
        self.speed = float(m.group(10))
        self.heading = float(m.group(11))
        self.day, self.month, self.year = [int(x) for x in m.groups()[11:14]]
        self.year += 2000

    def _set_data_from_gga(self, m):
        #self.hour, self.minute, self.seconds, self.millis = [int(x) for x in m.groups()[:4]]
        #self.lat, self.latNS, self.lon, self.lonEW = m.groups()[4:8]
        self.satellites = int(m.group(10))
        self.altitude = float(m.group(11))
        