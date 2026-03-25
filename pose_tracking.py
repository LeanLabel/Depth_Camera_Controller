from mctrl_networking import MctrlNet
import sched
import time

class Poser():

    def __init__(self, net_mod, cam_mod, rate):
        
        self.net_mod = net_mod
        self.cam_mod = cam_mod
        self.scheduler = sched.scheduler(time.time, time.sleep)
        self.interval = 1 / rate

        self.init_time = None
        self.next_time = None

    def pose_tx_loop(self):
        
        # get camera data
        pos, ori = None, None # TODO

        # send packet
        pckt = self.net_mod.get_odom_msg(pos, ori)
        self.net_mod.transfer_packet(pckt)

        self.next_time += self.interval

        self.scheduler.enterabs(self.next_time, 1, self.pose_tx_loop)

    def pose_tx_start(self):

        self.init_time = (time.time() // self.interval + 1) * interval
        self.next_time = self.init_time

        self.scheduler.enterabs(self.init_time, 1, self.pose_tx_loop)

        self.scheduler.run()


if __name__=="__main__":
    
    # networking config
    udp_ip = "127.0.0.1"
    tx_port = 5005
    rx_port = 0 # unused
    net_mod = MctrlNet(udp_ip, tx_port, rx_port)

    # camera config

    # poser config
    rate = 30
    poser = Poser(net_mod, cam_mod, rate)
