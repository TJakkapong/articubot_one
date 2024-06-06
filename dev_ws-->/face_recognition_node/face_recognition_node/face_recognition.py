import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import face_recognition
import pyglet

class FaceRecognitionNode(Node):
    def __init__(self):
        super().__init__('face_recognition_node')
        self.publisher_ = self.create_publisher(Image, 'face_recognition/image', 10)
        self.bridge = CvBridge()

        # Get camera index from parameter, default to 0
        self.camera_index = self.declare_parameter('camera_index', 4).value

        # Open a connection to the camera
        self.video_capture = cv2.VideoCapture(self.camera_index)
        if not self.video_capture.isOpened():
            self.get_logger().error('Failed to open camera')
            return

        # Define frame skip
        self.frame_skip = 1

        # Load image and video files
        self.image_file = "/home/kittawat/dev_ws/myvideos/no_face_detected.jpg"  # Replace with your image file
        self.video_file = "/home/kittawat/dev_ws/myvideos/face_detected_video.mp4"  # Replace with your video file

        # Check if image file exists and can be loaded
        self.image = cv2.imread(self.image_file)
        if self.image is None:
            self.get_logger().error(f'Failed to load image file: {self.image_file}')
            return

        # Check if video file exists
        if not cv2.VideoCapture(self.video_file).isOpened():
            self.get_logger().error(f'Video file not found: {self.video_file}')
            return

        # Flag to track if someone is detected
        self.person_detected = False

        # Create a timer to periodically execute the detection
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Capture frame-by-frame
        ret, frame = self.video_capture.read()
        if not ret:
            self.get_logger().warning('Failed to read frame from camera')
            return

        # Convert grayscale image to BGR format if necessary
        if len(frame.shape) == 2:  # Check if the image is grayscale
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        # Skip frames according to frame_skip
        if self.frame_skip > 1:
            for _ in range(self.frame_skip - 1):
                self.video_capture.grab()  # Skip frames without decoding

        # Find all face locations in the current frame
        face_locations = face_recognition.face_locations(frame, model="hog")

        # If faces are detected, set the flag
        if face_locations:
            self.person_detected = True

        # Display video if someone is detected
        if self.person_detected:
            # Initialize pyglet window
            self.display = pyglet.canvas.get_display()
            self.screen = self.display.get_default_screen()
            self.window = pyglet.window.Window(width=self.screen.width, height=self.screen.height, fullscreen=True)

            # Load and play video with sound
            self.player = pyglet.media.Player()
            self.source = pyglet.media.StreamingSource()
            self.media = pyglet.media.load(self.video_file)
            self.player.queue(self.media)
            self.player.play()

            @self.window.event
            def on_draw():
                self.window.clear()
                self.player.get_texture().blit(0, 0)

            @self.window.event
            def on_key_press(symbol, modifiers):
                if symbol == pyglet.window.key.Q:
                    self.player.pause()
                    self.window.close()

            pyglet.app.run()

        else:
            # Display image fullscreen
            cv2.namedWindow('Image', cv2.WND_PROP_FULLSCREEN)
            cv2.setWindowProperty('Image', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            image_resized = cv2.resize(self.image, (1280, 800))
            cv2.imshow('Image', image_resized)
            cv2.waitKey(0)

        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = FaceRecognitionNode()
    rclpy.spin(node)

    # Clean up
    node.video_capture.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

