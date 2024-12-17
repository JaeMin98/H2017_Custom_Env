#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading

class CameraViewer:
    def __init__(self):
        # OpenCL 사용 비활성화
        cv2.ocl.setUseOpenCL(False)

        # ROS 노드 초기화
        rospy.init_node('camera_viewer', anonymous=True)
        self.bridge = CvBridge()

        # YOLOv3 모델 파일 경로 지정 (사용 환경에 맞게 변경)
        self.weights_path = "./yolov3_models/yolov3.weights"  # 실제 경로로 변경
        self.config_path = "./yolov3_models/yolov3.cfg"       # 실제 경로로 변경
        self.names_path = "./yolov3_models/coco.names"        # 실제 경로로 변경

        # YOLOv3 네트워크 로드
        self.net = cv2.dnn.readNetFromDarknet(self.config_path, self.weights_path)
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        self.lock = threading.Lock()  # 추론 시 동시성 문제 방지용 Lock

        # 클래스 이름 로드
        with open(self.names_path, 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]

        # YOLOv3 출력 레이어 추출
        layer_names = self.net.getLayerNames()
        self.output_layers = [layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

        # departure_place_camera 이미지 토픽 구독
        rospy.Subscriber("/departure_place_camera/departure_place_camera/image_raw", Image, self.departure_callback)
        # destination_place_camera 이미지 토픽 구독
        rospy.Subscriber("/destination_place_camera/destination_place_camera/image_raw", Image, self.destination_callback)

        rospy.loginfo("Subscribed to both camera topics and loaded YOLOv3 model.")

    def detect_objects(self, frame):
        # 이미지 전처리
        with self.lock:
            blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
            self.net.setInput(blob)
            outs = self.net.forward(self.output_layers)

        class_ids = []
        confidences = []
        boxes = []

        h, w = frame.shape[:2]

        for out in outs:
            for detection in out:
                # detection = [center_x, center_y, width, height, obj_score, class_score...]
                # Inf/NaN 체크
                if not (np.isfinite(detection[0]) and np.isfinite(detection[1]) and 
                        np.isfinite(detection[2]) and np.isfinite(detection[3])):
                    continue

                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                if np.isfinite(confidence) and confidence > 0.5:
                    center_x = detection[0] * w
                    center_y = detection[1] * h
                    dw = detection[2] * w
                    dh = detection[3] * h

                    # 비정상적인 바운딩 박스 걸러내기
                    if not (np.isfinite(center_x) and np.isfinite(center_y) and 
                            np.isfinite(dw) and np.isfinite(dh)):
                        continue
                    if dw <= 0 or dh <= 0:
                        continue

                    x = int(center_x - dw / 2)
                    y = int(center_y - dh / 2)
                    boxes.append([x, y, int(dw), int(dh)])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # 비정상 값 여부 추가 체크
        filtered_boxes = []
        filtered_confidences = []
        filtered_class_ids = []
        for b, c, cid in zip(boxes, confidences, class_ids):
            # int 변환 시 오버플로우가 없도록 제한
            if all(abs(val) < 1e9 for val in b):
                filtered_boxes.append(b)
                filtered_confidences.append(c)
                filtered_class_ids.append(cid)

        if len(filtered_boxes) == 0:
            return [], [], [], []

        # NMS 수행
        indices = cv2.dnn.NMSBoxes(filtered_boxes, filtered_confidences, 0.5, 0.4)

        return indices, filtered_boxes, filtered_class_ids, filtered_confidences

    def draw_boxes(self, frame, indices, boxes, class_ids, confidences):
        for i in indices:
            i = i[0]
            x, y, w, h = boxes[i]
            label = str(self.classes[class_ids[i]])
            confidence = confidences[i]

            # dining_table 클래스는 바운딩박스를 표시하지 않음
            if label == "diningtable" or label == "dining_table" or label == "dining table" :
                continue

            # orange, sports ball 클래스를 apple로 표기
            if label == "orange" or label == "sports ball" or label == "sportsball" or label == "sports_ball":
                label = "apple"

            color = (0, 255, 0)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            text = "{}: {:.2f}".format(label, confidence)
            cv2.putText(frame, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return frame

    def process_frame(self, cv_image, window_name):
        # 이미지 크기를 640x480으로 조정
        resized_image = cv2.resize(cv_image, (640, 480))
        # YOLO 객체 검출
        indices, boxes, class_ids, confidences = self.detect_objects(resized_image)
        # 바운딩 박스 표시
        output_frame = self.draw_boxes(resized_image, indices, boxes, class_ids, confidences)
        cv2.imshow(window_name, output_frame)
        cv2.waitKey(1)

    def departure_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.process_frame(cv_image, "Departure Place Camera")
        except Exception as e:
            rospy.logerr("Error in departure_callback: {}".format(e))

    def destination_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.process_frame(cv_image, "Destination Place Camera")
        except Exception as e:
            rospy.logerr("Error in destination_callback: {}".format(e))

if __name__ == '__main__':
    try:
        viewer = CameraViewer()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Camera viewer node interrupted.")
    finally:
        cv2.destroyAllWindows()
