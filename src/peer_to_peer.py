import logging
from threading import Thread
import struct
import math

import cflib
from cflib.crazyflie import Crazyflie

#ROS2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

logging.basicConfig(level=logging.ERROR)

# Map object class to single/double
# From the model on AI-deck:
# BALL = 1
# CONE = 2
CLASS_SINGLE = 2
CLASS_DOUBLE = 1

# Map virtual tag IDs to single/double
TAGS_SINGLE = ["tag36h11:200","tag36h11:204","tag36h11:205"]
TAGS_DOUBLE = ["tag36h11:206"]

# Drone IDs to create publishers
DRONE_ID_MAX = 25

# Other config
LOCATION_THRESHOLD = 1.0

# Coordinates seem to be in NWU
CF_OFFSETS = {
    0: [0, 0],
    1: [1, 0],
    18: [0, 2]
}

class ProcessDetection(Node):
    def __init__(self, link_uri):
        """ Initialize and run with the specified link_uri """

        #Init ROS2 node
        super().__init__('P2P_GCS_Node')

        # self.undetectedTags = {"tag36h11:200","tag36h11:204","tag36h11:205"}
        # self.doublerescue = {"tag36h11:206":2}

        self._cf = Crazyflie()
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        self._cf.appchannel.packet_received.add_callback(self._app_packet_received)

        # /cf05/tf
        self.tfPublishers = {}
        for currentDroneID in range(1, DRONE_ID_MAX + 1):
            currentDroneString = "cf{:02d}".format(currentDroneID)
            currentTopic = "/" + currentDroneString + "/tf"
            self.tfPublishers[currentDroneID] = self.create_publisher(TFMessage, currentTopic, 10)

        print('Connecting to %s' % link_uri)
        self._cf.open_link(link_uri)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)

    def _app_packet_received(self, data):
        #  struct SendPacket
        # {
        #   uint8_t id;
        #   uint8_t rssi;
        #   uint8_t class;
        #   float confidence;
        #   float x;
        #   float y;
        #   float z;
        # } __attribute__((packed));

        id, rssi, objClass, confidence, x, y, z = struct.unpack("<BBBffff", data)

        print(f"[Recv] ID: {id}, RSSI: -{rssi}, Class: {objClass}, Confidence: {confidence:.3f}, Pos: [{x:.2f}, {y:.2f}, {z:.2f}]")

        if (id not in CF_OFFSETS):
            print(f"[Warning] CF ID: {id} not found in CF_OFFSETS!")
            return

        # Global pose of CF
        globalPos = [CF_OFFSETS[id][0] + x, CF_OFFSETS[id][1] + y]

        # Class to check
        checkClassLocations = classTagLocations[objClass]

        # Find matching tag based on global X/Y
        matchingTag = False

        for tagID, tagLocation in checkClassLocations.items():
            if tagLocation == None:
                # This virtual tag ID has not been used
                checkClassLocations[tagID] = globalPos
                matchingTag = tagID
                print(f"[CF{id}] {globalPos} tagID: {tagID}, tagLocation: {globalPos}, [Added]")
                break;
            else:
                distance = math.sqrt((tagLocation[0] - globalPos[0])**2 + (tagLocation[1] - globalPos[1])**2)
                matched = distance <= LOCATION_THRESHOLD

                print(f"[CF{id}] {globalPos} tagID: {tagID}, tagLocation: {tagLocation}, Distance: {distance:.2f}, Matched: {matched}")

                if matched:
                    matchingTag = tagID
                    break

        if not matchingTag:
            print(f"[Warning] No matching tag found for global pose {globalPos} in: ")
            print(checkClassLocations)
            return

        if not id in self.tfPublishers:
            print(f"[Warning] tfPublishers key not found: {id}")
            return

        tfMessage = TFMessage()
        tfStamped = TransformStamped()
        tfStamped.child_frame_id = matchingTag
        tfMessage.transforms.append(tfStamped)
        self.tfPublishers[id].publish(tfMessage)

if __name__ == '__main__':
    rclpy.init()

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    for i in available:
        print(i[0])

    # self.undetectedTags = {"tag36h11:200","tag36h11:204","tag36h11:205"}
    # self.doublerescue = {"tag36h11:206":2}
    # Struct tagLocations[class][tag][{x,y}]
    classTagLocations = {}

    #Populate with initial values
    classTagLocations[CLASS_SINGLE] = {}
    for currentTag in TAGS_SINGLE:
        classTagLocations[CLASS_SINGLE][currentTag] = None

    classTagLocations[CLASS_DOUBLE] = {}
    for currentTag in TAGS_DOUBLE:
        classTagLocations[CLASS_DOUBLE][currentTag] = None

    if len(available) > 0:
        processDetection = ProcessDetection(available[0][0])
        rclpy.spin(processDetection)
        processDetection.destroy_node()
    else:
        print('No Crazyflies found, cannot run example')

    rclpy.shutdown()
