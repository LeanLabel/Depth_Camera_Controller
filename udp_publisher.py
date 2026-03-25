import socket
import json

class UDPPublisher:
    """Send pose data over UDP as JSON."""
    
    def __init__(self, ip='127.0.0.1', port=5005):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send_pose(self, translation, rotation_quat):
        """
        Send a pose as a JSON dictionary over UDP.
        
        Parameters:
            translation : list of 3 floats [x, y, z] in meters
            rotation_quat : list of 4 floats [w, x, y, z]
        """
        data = {
            "translation": {
                "x": translation[0],
                "y": translation[1],
                "z": translation[2]
            },
            "rotation_quat": {
                "w": rotation_quat[0],
                "x": rotation_quat[1],
                "y": rotation_quat[2],
                "z": rotation_quat[3]
            }
        }
        message = json.dumps(data).encode('utf-8')
        self.sock.sendto(message, (self.ip, self.port))

    def close(self):
        self.sock.close()
