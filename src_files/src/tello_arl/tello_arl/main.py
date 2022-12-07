#!/usr/bin/env python3

import rclpy
from . import TelloARL


def main(args=None):
    rclpy.init(args=args)
    tello_arl = TelloARL()

    try:
        rclpy.spin(tello_arl)
    except KeyboardInterrupt:
        pass
    finally:
        tello_arl.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(args=None)
