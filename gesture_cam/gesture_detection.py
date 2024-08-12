import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pickle
from std_msgs.msg import String

import cv2
import mediapipe as mp
import numpy as np


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        
        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        self.get_logger().info('image_publisher has been started...')
        # Create the timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.gesture_publish_ = self.create_publisher(String, 'gesture_recog', 10)
        
        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.cap = cv2.VideoCapture(0)
        
        self.br = CvBridge() #bridge for ros and openCV
        
        self.model_dict = pickle.load(open(r'/home/manan/ros2_ws/src/model.p', 'rb'))
        self.model = self.model_dict['model']
        
        self.mp_hands = mp.solutions.hands
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        self.hands = self.mp_hands.Hands(static_image_mode=True, min_detection_confidence=0.3)

        self.labels_dict = {0: 'go', 1: 'reverse', 2: 'right', 3:'left', 4:'stop'}
        
    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.1 seconds.
        """
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        gesture_recog = String()
        
        data_aux = []
        x_ = []
        y_ = []
        
        ret, frame = self.cap.read()
        H, W, _ = frame.shape
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        results = self.hands.process(frame_rgb)
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    frame,  # image to draw
                    hand_landmarks,  # model output
                    self.mp_hands.HAND_CONNECTIONS,  # hand connections
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style())

            for hand_landmarks in results.multi_hand_landmarks:
                for i in range(len(hand_landmarks.landmark)):
                    x = hand_landmarks.landmark[i].x
                    y = hand_landmarks.landmark[i].y

                    x_.append(x)
                    y_.append(y)

                for i in range(len(hand_landmarks.landmark)):
                    x = hand_landmarks.landmark[i].x
                    y = hand_landmarks.landmark[i].y
                    data_aux.append(x - min(x_))
                    data_aux.append(y - min(y_))

            x1 = int(min(x_) * W) - 10
            y1 = int(min(y_) * H) - 10

            x2 = int(max(x_) * W) - 10
            y2 = int(max(y_) * H) - 10

            prediction = self.model.predict([np.asarray(data_aux)])

            predicted_character = self.labels_dict[int(prediction[0])]
            
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 0), 4)
            cv2.putText(frame, predicted_character, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 0, 0), 3,
                    cv2.LINE_AA)
        
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
            gesture_recog.data = predicted_character
            # Display the message on the console
            self.get_logger().info('Publishing video frame...')
            self.get_logger().info(predicted_character)
            
            self.gesture_publish_.publish(gesture_recog)

def main(args=None):
  
  rclpy.init(args=args)
  image_publisher = ImagePublisher()
  rclpy.spin(image_publisher)
  image_publisher.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()