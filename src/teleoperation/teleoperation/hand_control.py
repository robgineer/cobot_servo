import rclpy
import threading

from queue import Queue
from teleoperation.hand_tracker import HandTracker
from teleoperation.servo_controller_node import ServoController


def main():

    command_queue = Queue(maxsize=1)

    # start hand tracking in a separate thread
    tracker = HandTracker(command_queue)
    tracker_thread = threading.Thread(target=tracker.run, daemon=True)
    tracker_thread.start()

    rclpy.init()

    max_attempts = 100
    attempt = 0
    # this exception is required for MoveItPy
    # in some cases it needs some time to initialize.
    while attempt < max_attempts:
        attempt += 1
        try:
            servo_node = ServoController(command_queue)
            break
        except Exception as e:
            print(f"[MoveItPy] Attempt {attempt} failed: {e.__class__.__name__}: {e}")
            if attempt == max_attempts:
                print("[MoveItPy] All attempts failed. Giving up.")
                raise

    try:
        rclpy.spin(servo_node)
    except KeyboardInterrupt:
        pass

    servo_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
