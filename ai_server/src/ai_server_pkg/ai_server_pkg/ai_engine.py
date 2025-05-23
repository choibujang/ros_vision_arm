import cv2
import numpy as np
from ultralytics import YOLO
import time
from collections import defaultdict
import copy
from queue import Full

class AIEngine:
    def __init__(self, frame_queue, detected_objects, lock):
        self.model = YOLO("../resource/best.pt")
        self.frame_queue = frame_queue
        self.lock = lock
        self.detected_objects = detected_objects

    def run_detection(self):
        while True:
            frame = self.frame_queue.get()
            device_id = frame['device_id']

            image_bytes = frame['data']
            np_arr = np.frombuffer(image_bytes, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if img is None:
                continue

            try:
                results = self.model.predict(img, verbose=False)
            except Exception as e:
                print(f"[ERROR] YOLO failed: {e}")
                continue
            
            detections = results[0].boxes

            buffer = defaultdict(lambda: {'count' : 0, 'pixels' : []})

            if len(detections) > 0:
                for det in detections:
                    x1, y1, x2, y2 = map(int, det.xyxy[0])
                    x_center = (x1 + x2) // 2
                    y_center = (y1 + y2) // 2
                    class_id = int(det.cls[0]) 
                    class_name = self.model.names[class_id]

                    buffer[class_name]['count'] += 1
                    buffer[class_name]['pixels'].append((x_center, y_center))

                    cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.circle(img, (x_center, y_center), 3, (0, 0, 255), -1)
                    cv2.putText(img, class_name, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            with self.lock:
                self.detected_objects[device_id] = dict(buffer)

            cv2.imshow("AI Server Detection", img)
            if cv2.waitKey(1) & 0xFF == 27:
                break

        cv2.destroyAllWindows()