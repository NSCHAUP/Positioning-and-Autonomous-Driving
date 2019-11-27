
#!/usr/bin/env python
"""

The Pozyx ready to localize tutorial (c) Pozyx Labs
Please read the tutorial that accompanies this sketch:
https://www.pozyx.io/Documentation/Tutorials/ready_to_localize/Python

This tutorial requires at least the contents of the Pozyx Ready to Localize kit. It demonstrates the positioning capabilities
of the Pozyx device both locally and remotely. Follow the steps to correctly set up your environment in the link, change the
parameters and upload this sketch. Watch the coordinates change as you move your device around!

"""
from time import sleep

from pypozyx import (PozyxConstants, Coordinates, POZYX_SUCCESS, PozyxRegisters, version,
                     DeviceCoordinates, PozyxSerial, get_first_pozyx_serial_port, SingleRegister)
from pythonosc.udp_client import SimpleUDPClient

from pypozyx.tools.version_check import perform_latest_version_check

x_position_front = 0;
y_position_front = 0;
z_position_front = 0;
x_position_back = 0;
y_position_back = 0;
z_position_back = 0;
        
class MultitagPositioning(object):
    """Continuously performs multitag positioning"""

    def __init__(self, pozyx, osc_udp_client, tag_ids, anchors, algorithm=PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY,
                 dimension=PozyxConstants.DIMENSION_3D, height=1000):
        self.pozyx = pozyx
        self.osc_udp_client = osc_udp_client

        self.tag_ids = tag_ids
        self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height

    def setup(self):
        """Sets up the Pozyx for positioning by calibrating its anchor list."""
        print("------------POZYX MULTITAG POSITIONING V{} -------------".format(version))
        print("")
        print(" - System will manually calibrate the tags")
        print("")
        print(" - System will then auto start positioning")
        print("")
        if None in self.tag_ids:
            for device_id in self.tag_ids:
                self.pozyx.printDeviceInfo(device_id)
        else:
            for device_id in [None] + self.tag_ids:
                self.pozyx.printDeviceInfo(device_id)
        print("")
        print("------------POZYX MULTITAG POSITIONING V{} -------------".format(version))
        print("")

        self.setAnchorsManual()

        self.printPublishAnchorConfiguration()

    def loop(self):
        """Performs positioning and prints the results."""
        for tag_id in self.tag_ids:
            position = Coordinates()
            status = self.pozyx.doPositioning(
                position, self.dimension, self.height, self.algorithm, remote_id=tag_id)
            if status == POZYX_SUCCESS:
                self.printPublishPosition(position, tag_id)
            else:
                self.printPublishErrorCode("positioning", tag_id)
    
    # WORK IN PROGRESS: First attempt at making a function to collect and store the x, y, and z coordinates globally
    """def giveUsDemCoordyBois(self, position, network_id):
        #hopefully gives us the coordinates to be able to access outside of the class
        x_position = position.x;
        y_position = position.y;
        z_position = position.z;
        
        print(x_position);
        print(y_position);
        print(z_position);"""

    def printPublishPosition(self, position, network_id):
        """Prints the Pozyx's position and possibly sends it as a OSC packet"""
        if network_id is None:
            network_id = 0
        s = "POS ID: {}, x(mm): {}, y(mm): {}, z(mm): {}".format("0x%0.4x" % network_id,
                                                                 position.x, position.y, position.z)
        
        print(s)
        if self.osc_udp_client is not None:
            self.osc_udp_client.send_message(
                "/position", [network_id, position.x, position.y, position.z])
        
        #NOTE: front tag is the tag connected to the Pi(0x673c/0x0000) and back tag is connected to external power source
        if network_id==0x0000: #if the tag that is having its coordinates measured is the one connected to the Pi
            global x_position_front #indicate to program that the global variable for the front tag's x-position is to be used
            global y_position_front #indicate to program that the global variable for the front tag's y-position is to be used
            global z_position_front #indicate to program that the global variable for the front tag's z-position is to be used
            x_position_front = position.x; #set the x-position of the front tag to the x-value output by the Pozyx
            y_position_front = position.y; #set the y-position of the front tag to the y-value output by the Pozyx
            z_position_front = position.z; #set the z-position of the front tag to the z-value output by the Pozyx
            print(x_position_front); #output the front tag's x-position
            print(y_position_front); #output the front tag's y-position
            print(z_position_front); #output the front tag's z-position
        else: #otherwise, if the tag that is having its coordinates measured is the one connected to the external power source
            global x_position_back #indicate to program that the global variable for the back tag's x-position is to be used
            global y_position_back #indicate to program that the global variable for the back tag's y-position is to be used
            global z_position_back #indicate to program that the global variable for the back tag's z-position is to be used
            x_position_back = position.x; #set the x-position of the back tag to the x-value output by the Pozyx
            y_position_back = position.y; #set the y-position of the back tag to the y-value output by the Pozyx
            z_position_back = position.z; #set the z-position of the back tag to the z-value output by the Pozyx
            print(x_position_back); #output the back tag's x-position
            print(y_position_back); #output the back tag's y-position
            print(z_position_back); #output the back tag's z-position
        
        #print("THE LOOP HAS BEEN EXITED"); FOR TESTING: print a statement that will allow us to see how exactly the loop
                                                        #running the main body is working and where it is at in its execution
        
        """NOTE: The following print statements were used previously to check if the global variables for x, y, and z
        were being set to the Pozyx's outputs:
        print(x_position);
        print(y_position);
        print(z_position);"""

    def setAnchorsManual(self):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""
        for tag_id in self.tag_ids:
            status = self.pozyx.clearDevices(tag_id)
            for anchor in self.anchors:
                status &= self.pozyx.addDevice(anchor, tag_id)
            if len(anchors) > 4:
                status &= self.pozyx.setSelectionOfAnchors(PozyxConstants.ANCHOR_SELECT_AUTO, len(anchors))
            # enable these if you want to save the configuration to the devices.

    def printPublishConfigurationResult(self, status, tag_id):
        """Prints the configuration explicit result, prints and publishes error if one occurs"""
        if tag_id is None:
            tag_id = 0
        if status == POZYX_SUCCESS:
            print("Configuration of tag %s: success" % tag_id)
        else:
            self.printPublishErrorCode("configuration", tag_id)

    def printPublishErrorCode(self, operation, network_id):
        """Prints the Pozyx's error and possibly sends it as a OSC packet"""
        error_code = SingleRegister()
        status = self.pozyx.getErrorCode(error_code, network_id)
        if network_id is None:
            network_id = 0
        if status == POZYX_SUCCESS:
            print("Error %s on ID %s, %s" %
                  (operation, "0x%0.4x" % network_id, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/error_%s" % operation, [network_id, error_code[0]])
        else:
            # should only happen when not being able to communicate with a remote Pozyx.
            self.pozyx.getErrorCode(error_code)
            print("Error % s, local error code %s" % (operation, str(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message("/error_%s" % operation, [0, error_code[0]])

    def printPublishAnchorConfiguration(self):
        for anchor in self.anchors:
            print("ANCHOR,0x%0.4x,%s" % (anchor.network_id, str(anchor.pos)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/anchor", [anchor.network_id, anchor.pos.x, anchor.pos.y, anchor.pos.z])
                sleep(0.025)


if __name__ == "__main__":
    # Check for the latest PyPozyx version. Skip if this takes too long or is not needed by setting to False.
    check_pypozyx_version = True
    if check_pypozyx_version:
        perform_latest_version_check()

    # shortcut to not have to find out the port yourself.
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    # enable to send position data through OSC
    use_processing = True

    # configure if you want to route OSC to outside your localhost. Networking knowledge is required.
    ip = "127.0.0.1"
    network_port = 8888


    # IDs of the tags to position, add None to position the local tag as well.
    tag_ids = [None,0x6728]

    # necessary data for calibration
    anchors = [DeviceCoordinates(0x6e09, 1, Coordinates(0, 0, 0)),
               DeviceCoordinates(0x674c, 1, Coordinates(1650, 0, 0)),
               DeviceCoordinates(0x6729, 1, Coordinates(0, 480, 0)),
               DeviceCoordinates(0x6765, 1, Coordinates(1650, 480, 0))]

    # positioning algorithm to use, other is PozyxConstants.POSITIONING_ALGORITHM_TRACKING
    algorithm = PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY
    # positioning dimension. Others are PozyxConstants.DIMENSION_2D, PozyxConstants.DIMENSION_2_5D
    dimension = PozyxConstants.DIMENSION_2D
    # height of device, required in 2.5D positioning
    height = 1000

    osc_udp_client = None
    if use_processing:
        osc_udp_client = SimpleUDPClient(ip, network_port)

    pozyx = PozyxSerial(serial_port)

    r = MultitagPositioning(pozyx, osc_udp_client, tag_ids, anchors,
                            algorithm, dimension, height)
    r.setup()
    def getUpdatedCoordinates():
        # while True:    
        r.loop()
        print("THE FRONT X POSITION IS:" + str(x_position_front)); #used to see if the x-positions of the front tag were correct
        print("THE FRONT Y POSITION IS:" + str(y_position_front)); #used to see if the y-positions of the front tag were correct
        print("THE FRONT Z POSITION IS:" + str(z_position_front)); #used to see if the z-positions of the front tag were correct
        print("THE BACK X POSITION IS:" + str(x_position_front)); #used to see if the x-positions of the back tag were correct
        print("THE BACK Y POSITION IS:" + str(y_position_front)); #used to see if the y-positions of the back tag were correct
        print("THE BACK Z POSITION IS:" + str(z_position_front)); #used to see if the z-positions of the back tag were correct
        y = int(y_position_front)
        x = int(x_position_front)
            #break; #NOTE: this was previously used to end the execution of the Pozyx measuring to see if the new variables were working
            
#Previously used to check if the global variables for the x, y, and z coordinates were functioning properly:
#print(x_position);
#print(y_position);
#print(z_position);
        import RPi.GPIO as GPIO #Pin setup for Entire Pi
        import time
        import curses #User Interface
        import serial
        #pin setup
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(13, GPIO.OUT)
        GPIO.setup(22, GPIO.OUT)
        GPIO.setup(15, GPIO.OUT)
        GPIO.setup(18, GPIO.OUT)



    #motor varibles
        FR= GPIO.PWM(13,50)#Front Right Motor #The value 50 is the Frequency 
        FL= GPIO.PWM(22,50)#Front Left Motor #The value 12 is the GPIO pin
        RR= GPIO.PWM(15,50)#Rear Right Motor
        RL= GPIO.PWM(18,50)#Rear Left Motor
        FR.start(100)
        FL.start(100)
        RR.start(100)
        RL.start(100)
    #curses setup
    #screen = curses.initscr()
    #curses.noecho()
    #curses.cbreak()
    #screen.keypad(True)
    #User Interface
        print('...Loading...')
        while True:
           getUpdatedCoordinates()
           print(y_position_front)
           if  int(y_position_front)>1400:
              FR.ChangeDutyCycle(100)
              FL.ChangeDutyCycle(100)
              RR.ChangeDutyCycle(100)
              RL.ChangeDutyCycle(100)
              print('Almost there')
              break
           else:
              #getUpdatedCoordinates()
              print(y_position_front)
              FR.ChangeDutyCycle(6.5)
              FL.ChangeDutyCycle(8)
              RR.ChangeDutyCycle(6.5)
              RL.ChangeDutyCycle(8)
        while True:
            print("turning 90 degrees left")
            FR.ChangeDutyCycle(5)
            FL.ChangeDutyCycle(5)
            RR.ChangeDutyCycle(5)
            RL.ChangeDutyCycle(5)
            time.sleep(.68)
            FR.ChangeDutyCycle(100)
            FL.ChangeDutyCycle(100)
            RR.ChangeDutyCycle(100)
            RL.ChangeDutyCycle(100)
            break
        while True:
            getUpdatedCoordinates()
            print(x_position_front)
            if (x_position_front)>(1400):
               FR.ChangeDutyCycle(100)
               FL.ChangeDutyCycle(100)
               RR.ChangeDutyCycle(100)
               RL.ChangeDutyCycle(100)
               print('Arrived')
               break
            else:
               #getUpdatedCoordinates()
               print(x_position_front)
               FR.ChangeDutyCycle(6.5)
               FL.ChangeDutyCycle(8)
               RR.ChangeDutyCycle(6.5)
               RL.ChangeDutyCycle(8)
               print("tada!")



            #cleanup
            GPIO.cleanup
            curses.nobreak()
            screen.keypad(0)
            curses.echo()
            curses.endwin()

        
