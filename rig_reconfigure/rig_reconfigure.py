import rclpy

def main(args=None):
    rclpy.init(args=args)

    print("Hello World!")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
