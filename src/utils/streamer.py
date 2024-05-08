import sys
import time
import traceback
import cv2
import imagezmq
from threading import Thread


class Streamer:
    def __init__(self, jpeg_quality=95, adress='tcp://127.0.0.1:5555', sending_fps = 30, sender_name="src1"):
        # 0 to 100, higher is better quality, 95 is cv2 default
        self.jpeg_quality = jpeg_quality
        self.sender = imagezmq.ImageSender(connect_to=adress)
        self.last_frame = None
        self.sending_fps = sending_fps
        self.sender_name = sender_name
        self.sending_thread = Thread(target=self.sending_loop)
        self.sending_thread.start()
        print(f"Streamer started with parameters: jpeg_quality={jpeg_quality}, adress={adress}, sending_fps={sending_fps}, sender_name={sender_name}")

    def add_frame_to_send(self, frame):
        self.last_frame = frame

    def sending_loop(self):
        frame_delay = 1 / self.sending_fps 
        while True:
            start_time = time.time()  
            if self.last_frame is not None:
                try:
                    self.send_frame(self.last_frame)
                except (KeyboardInterrupt, SystemExit):
                    break  # Ctrl-C was pressed to end program
                except Exception as ex:
                    print('Traceback error:', ex)
                    traceback.print_exc()
            else: 
                time.sleep(0.1)

            end_time = time.time() 
            elapsed_time = end_time - start_time  

            if elapsed_time < frame_delay:
                time.sleep(frame_delay - elapsed_time)  # Delay the remaining time to achieve the desired FPS

    def send_frame(self, frame):
        ret_code, jpg_buffer = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        reply_from_mac = self.sender.send_jpg(self.sender_name, jpg_buffer)

    def stop(self):
        sys.exit()

if __name__ == "__main__":
    # Example: Streaming video from webcam 
    cap = cv2.VideoCapture(0)
    streamer = Streamer()
    while cap.isOpened():
        success, frame = cap.read()
        if success:
            streamer.add_frame_to_send(frame)
          