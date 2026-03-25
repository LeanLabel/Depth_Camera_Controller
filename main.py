import cv2
from pose_detector import PoseDetector
from udp_publisher import UDPPublisher


def main():
    detector = PoseDetector('config.json')
    publisher = UDPPublisher('127.0.0.1', 5005)

    try:
        while True:
            color_image, depth_frame = detector.get_frames()
            if color_image is None:
                continue

            success, t, q, annotated = detector.detect_pose(
                color_image, depth_frame, return_annotated=True
            )

            if success:
                publisher.send_pose(t, q)   # send only when pose is valid
                
            cv2.imshow('Pose Detection', annotated)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        detector.stop()
        # publisher.close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
