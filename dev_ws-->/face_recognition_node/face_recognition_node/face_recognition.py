import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import face_recognition

class FaceRecognitionNode(Node):
    def __init__(self):
        super().__init__('face_recognition_node')
        self.publisher_ = self.create_publisher(Image, 'face_recognition/image', 10)
        self.bridge = CvBridge()

        # Open a connection to the camera
        self.video_capture = cv2.VideoCapture(1)

        # Define frame skip and resize factor
        self.frame_skip = 1
        self.resize_factor = 0.5

        # Create a timer to periodically execute the detection
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Capture frame-by-frame
        ret, frame = self.video_capture.read()

        # Skip frames according to frame_skip
        if self.frame_skip > 1:
            for _ in range(self.frame_skip - 1):
                self.video_capture.grab()  # Skip frames without decoding

        # Resize frame to reduce processing time
        small_frame = cv2.resize(frame, (0, 0), fx=self.resize_factor, fy=self.resize_factor)

        # Find all face locations in the current frame
        face_locations = face_recognition.face_locations(small_frame, model="hog")

        # Draw rectangles around the detected faces
        for (top, right, bottom, left) in face_locations:
            # Scale face locations back to original size
            top *= int(1 / self.resize_factor)
            right *= int(1 / self.resize_factor)
            bottom *= int(1 / self.resize_factor)
            left *= int(1 / self.resize_factor)
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

        # Convert OpenCV image to ROS 2 Image message
        image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(image_message)

        # Display the resulting image
        cv2.imshow('Video', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognitionNode()
    rclpy.spin(node)

    # Clean up
    node.video_capture.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

