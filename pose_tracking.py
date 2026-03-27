from mctrl_networking import MctrlNet, Coordinate, Quaternion
from pose_detector import PoseDetector

import sched
import signal, sys
import time

class Poser():

    def __init__(self, net_mod, cam_mod, rate, debug=False):
        
        self.net_mod = net_mod
        self.cam_mod = cam_mod
        self.scheduler = sched.scheduler(time.time, time.sleep)
        self.interval = 1 / rate

        self.init_time = None
        self.next_time = None

        self.debug = debug

    def pose_tx_loop(self):
        
        # get camera data
        pos, ori = self.cam_mod.get_pose()

        if self.debug:
            print(pos, ori)
        
        if (pos, ori) != (None, None):
            pos, ori = Coordinate.from_tuple(pos), Quaternion.from_tuple(ori)

            # send packet
            pckt = self.net_mod.get_odom_msg(pos, ori)
            self.net_mod.transfer_packet(pckt)

            if self.debug:
                print(pckt)

        self.next_time += self.interval

        self.scheduler.enterabs(self.next_time, 1, self.pose_tx_loop)

    def pose_tx_start(self):

        self.init_time = (time.time() // self.interval + 1) * interval
        self.next_time = self.init_time

        self.scheduler.enterabs(self.init_time, 1, self.pose_tx_loop)

        self.scheduler.run()

    def sig_int_handler(self, sig, frame):

        print(f"stopped process from {self.init_time}")
        print(f"sent {self.net_mod.seq} packets")

        self.net_mod.end_connection()
        sys.exit(0)


if __name__=="__main__":
    
    debug = True

    # networking config
    udp_ip = "127.0.0.1"
    tx_port = 5005
    rx_port = 0 # unused
    net_mod = MctrlNet(udp_ip, tx_port, rx_port)

    # camera config
    cam_mod = PoseDetector()

    # poser config
    rate = 30
    poser = Poser(net_mod, cam_mod, rate, debug=debug)

    signal.signal(signal.SIGINT, poser.sig_int_handler)

    poser.pose_tx_start()
