import cv2
import numpy as np
import time
import mediapipe as mp
from queue import Queue

from py_utils.controller_types import (
    HandTrackerData,
    HandLabel,
    ControlType,
    JointJogLabel,
    HandTrackerInfo,
)


class HandTracker:
    def __init__(self, command_queue: Queue):
        self.command_queue = command_queue

        # MediaPipe setup
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            model_complexity=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.mp_draw = mp.solutions.drawing_utils
        self.cap = cv2.VideoCapture(0)

    def classify_hand_command(self, landmarks):
        """Identifies the hand gesture and returns a command."""

        # the finger tips
        tips = [
            self.mp_hands.HandLandmark.INDEX_FINGER_TIP,
            self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
            self.mp_hands.HandLandmark.RING_FINGER_TIP,
            self.mp_hands.HandLandmark.PINKY_TIP,
        ]
        # the middle knuckles
        pips = [
            self.mp_hands.HandLandmark.INDEX_FINGER_PIP,
            self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP,
            self.mp_hands.HandLandmark.RING_FINGER_PIP,
            self.mp_hands.HandLandmark.PINKY_PIP,
        ]

        extended_fingers = 0
        # init dict
        fingers = {}
        for item in tips:
            fingers[item] = False

        for index, tip in enumerate(tips):
            # if the location of the tip is smaller than
            # the location of the middle knuckle,
            # then the finger is extended (y=0 is top of screen)
            if landmarks[tip].y < landmarks[pips[index]].y:
                fingers[tip] = True
                extended_fingers += 1
        # check thumb separately
        thumb_extended = False
        if (
            abs(
                landmarks[self.mp_hands.HandLandmark.THUMB_TIP].x
                - landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP].x
            )
            > 0.1
        ):
            thumb_extended = True
        # for each extended number, we assign a category
        # if thumb_extended:
        #    return HandTrackerInfo(ControlType.GRIPPER)
        if extended_fingers == 0:
            return HandTrackerInfo(ControlType.RESET)
        elif extended_fingers == 1:
            if fingers[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]:
                return HandTrackerInfo(ControlType.JOINT_JOG, JointJogLabel.FIRST)
        elif extended_fingers == 2:
            if (
                fingers[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
                and fingers[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
            ):
                return HandTrackerInfo(ControlType.JOINT_JOG, JointJogLabel.SECOND)
        elif extended_fingers == 4:
            return HandTrackerInfo(ControlType.TWIST)
        else:
            return HandTrackerInfo(ControlType.INVALID)

    def run(self):
        """The main function."""

        last_fps = 0

        while True:
            success, img = self.cap.read()
            if not success:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Reset to first frame
                continue

            img_raw = cv2.flip(img, 1)

            img = cv2.cvtColor(img_raw, cv2.COLOR_BGR2RGB)

            img_rgb = np.ascontiguousarray(img)  # <--- this line fixes it

            result = self.hands.process(img_rgb)

            # display FPS
            now = time.time()
            dt = now - last_fps
            last_fps = now
            if dt > 0:
                fps_current = 1.0 / dt
            draw_panel(img, [f"FPS: {fps_current:.1f}", f"DT: {dt:.1f}"], x=10, y=20)

            hand_tracker_data_left_right = [HandTrackerData(), HandTrackerData()]

            if result.multi_hand_landmarks:
                if len(result.multi_handedness) > 2:
                    # there are more than two hands in the image => abort control
                    draw_panel(
                        img, ["Allowing only one operator at a time."], x=100, y=200
                    )
                    continue

                for hand, handedness in zip(
                    result.multi_hand_landmarks, result.multi_handedness
                ):
                    hand_tracker_data = HandTrackerData()

                    label = handedness.classification[0].label
                    # focus on index finger position
                    idx = hand.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]

                    control_command_info = self.classify_hand_command(hand.landmark)
                    if control_command_info == None:
                        continue
                    hand_tracker_data.set_control_info(control_command_info)
                    hand_tracker_data.set_x_y_values(idx.x, idx.y)
                    hand_tracker_data.set_hand_label(label)

                    y = 0
                    if label == "Left":
                        hand_tracker_data_left_right[HandLabel.LEFT] = hand_tracker_data
                        y = 120
                    elif label == "Right":
                        hand_tracker_data_left_right[HandLabel.RIGHT] = (
                            hand_tracker_data
                        )
                        y = 200
                    else:
                        draw_panel(
                            img, ["ERROR! No left / right hand detected."], x=100, y=200
                        )

                    # draw hand landmarks
                    self.mp_draw.draw_landmarks(
                        img, hand, self.mp_hands.HAND_CONNECTIONS
                    )

                    draw_panel(
                        img,
                        [
                            f"Label: {label}",
                            f"Command: {control_command_info.control_type.name}",
                        ],
                        x=20,
                        y=y,
                    )

            # publish to queue (latest command overwrites older ones)
            if not self.command_queue.empty():
                self.command_queue.get_nowait()
            self.command_queue.put_nowait(hand_tracker_data_left_right)

            # display processed image on screen
            cv2.imshow("Hand Tracker", img)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        self.cap.release()
        cv2.destroyAllWindows()


def draw_rounded_box(img, x, y, w, h, radius=10, color=(40, 40, 40), alpha=0.6):
    """ """
    overlay = img.copy()
    # draw rounded rectangle
    cv2.rectangle(overlay, (x + radius, y), (x + w - radius, y + h), color, -1)
    cv2.rectangle(overlay, (x, y + radius), (x + w, y + h - radius), color, -1)
    cv2.circle(overlay, (x + radius, y + radius), radius, color, -1)
    cv2.circle(overlay, (x + w - radius, y + radius), radius, color, -1)
    cv2.circle(overlay, (x + radius, y + h - radius), radius, color, -1)
    cv2.circle(overlay, (x + w - radius, y + h - radius), radius, color, -1)

    # blend
    cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0, img)


def draw_panel(img, lines, x, y, w=240, line_h=28):
    """Display helper."""
    # background box height
    h = line_h * len(lines) + 20
    draw_rounded_box(img, x, y, w, h)

    # draw text
    for i, text in enumerate(lines):
        cv2.putText(
            img,
            text,
            (x + 15, y + 30 + i * line_h),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )
