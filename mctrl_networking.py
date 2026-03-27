import socket
import json

class Coordinate():
"""simple coordinate class"""

    def __init__(self, x, y, z):

        self.x = x
        self.y = y
        self.z = z

    def __iter__(self):

        yield self.x
        yield self.y
        yield self.z

    @staticmethod
    def from_tuple(cls, coord):

        return cls(coord[0], coord[1], coord[2])

class Quaternion():
"""simple quaternion class"""

    def __init__(self, w, x, y, z):

        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __iter__(self):

        yield w
        yield x
        yield y
        yield z

    @staticmethod
    def from_tuple(cls, quat):

        return cls(quat[0], quat[1], quat[2], quat[3])

class MctrlNet():

    def __init__(self, udp_ip, tx_port, rx_port):

        # remember network addresses
        self.udp_ip = udp_ip
        self.tx_port = tx_port
        self.rx_port = rx_port

        # connect udp socket
        self.sckt = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sckt.bind((self.udp_ip, self.rx_port))

        # last position and orientation
        self.pos, self.ori = self.get_pos_ori()

        # last packet id
        self.seq = 1

    def get_odom_msg(self, pos, ori) -> str:
        
        odom_dict = {
                "seq": self.seq,
                "px": pos.x,
                "py": pos.y,
                "pz": pos.z,
                "qw": ori.w,
                "qx": ori.x,
                "qy": ori.y,
                "qz": ori.z
                }

        odom_pack = json.dumps(odom_dict)

        return odom_pack

    def transfer_packet(self, data):

        self.sckt.sendto(bytes(data, "utf-8"), (self.udp_ip, self.tx_port))
        self.seq += 1

    def end_connection(self):

        self.sckt.close()
