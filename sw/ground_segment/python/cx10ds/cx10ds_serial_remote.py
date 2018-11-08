from __future__ import absolute_import, division, print_function

import threading
import serial

CMD = {
        ':TAKEOFF': 'o',
        ':IDLE': 'i',
        ':LAND': 'l',
        ':DIST': 'dist',
        ':START': 'm',
        ':START_LAND': 'M',
        ':LIMIT': 'limit',
        ':START2': 'm2',
        ':START_LAND2': 'M2',
        ':LIMIT2': 'limit2',
        ':RESET': 'r'
    }

class RemoteError(Exception):
    """Base class for exceptions in this module."""
    pass

class Cx10dsSerialRemote(threading.Thread):
    def __init__(self, callback, verbose=False, device='/dev/ttyUSB0', baudrate=57600):
        threading.Thread.__init__(self)
        self.callback = callback
        self.verbose = verbose
        self.running = True
        try:
            self.ser = serial.Serial(device, baudrate, timeout=1.0)
        except serial.SerialException:
            print("Error: unable to open serial port '%s'" % device)
            raise RemoteError

    def stop(self):
        print("End thread and close serial link")
        self.running = False
        self.ser.close()

    def shutdown(self):
        self.stop()

    def __del__(self):
        try:
            self.ser.close()
        except:
            pass

    def send(self, msg):
        """ Send a message over a serial link"""
        self.ser.write(msg.encode())
        self.ser.flush()

    def send_pos(self, x, y, d, m):
        msg = ":{},{},{},{}\n".format(x,y,d,m)
        self.send(msg)

    def send_sliders(self, dist, limit1, limit2):
        msg = ":{},{},{}\n".format(dist,limit1,limit2)
        self.send(msg)

    def run(self):
        """Thread running function"""
        try:
            while self.running:
                # Parse incoming data
                msg = self.ser.readline()
                msg = msg.strip('\n')
                if len(msg) > 0:
                    data = msg.split(' ')
                    try:
                        cmd = CMD[data[0]]
                        val = data[1:]
                        self.callback(cmd, val)
                    except Exception as e:
                        print("invalid command ({})".format(str(e)))
                    if self.verbose:
                        print("New incoming message '{}'".format(msg))

        except StopIteration:
            pass


