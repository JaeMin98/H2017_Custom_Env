#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import threading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Object_detection_module:
    def __init__(self):
        cv2.ocl.setUseOpenCL(False)
        rospy.init_node('Object_detection_module_node', anonymous=True)
        self._bridge_interface = CvBridge()
        self._path_weights = "./yolov3_models/yolov3.weights"
        self._path_config = "./yolov3_models/yolov3.cfg"
        self._path_classes = "./yolov3_models/coco.names"
        self._net = cv2.dnn.readNetFromDarknet(self._path_config, self._path_weights)
        self._net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self._net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        self._forward_pass_lock = threading.Lock()
        with open(self._path_classes, 'r') as f:
            self._object_classes = [line.strip() for line in f.readlines()]
        layer_names = self._net.getLayerNames()
        self._terminal_layers = [layer_names[i[0] - 1] for i in self._net.getUnconnectedOutLayers()]
        rospy.Subscriber("/departure_place_camera/departure_place_camera/image_raw", Image, self._ingest_departure_frame)
        rospy.Subscriber("/destination_place_camera/destination_place_camera/image_raw", Image, self._ingest_destination_frame)
        rospy.loginfo("Object_detection_module: Initialization complete.")

    def _execute_inference_pipeline(self, input_frame: np.ndarray):
        frame_height, frame_width = input_frame.shape[:2]
        with self._forward_pass_lock:
            input_blob = cv2.dnn.blobFromImage(input_frame, 1/255.0, (416, 416), swapRB=True, crop=False)
            self._net.setInput(input_blob)
            inference_output = self._net.forward(self._terminal_layers)
        assembled_boxes = []
        assembled_confidences = []
        assembled_class_ids = []
        for output_arr in inference_output:
            for vectorized_det in output_arr:
                if not (np.isfinite(vectorized_det[0]) and np.isfinite(vectorized_det[1]) and
                        np.isfinite(vectorized_det[2]) and np.isfinite(vectorized_det[3])):
                    continue
                classification_scores = vectorized_det[5:]
                chosen_class_id = np.argmax(classification_scores)
                candidate_confidence = classification_scores[chosen_class_id]
                if np.isfinite(candidate_confidence) and candidate_confidence > 0.5:
                    center_x = vectorized_det[0] * frame_width
                    center_y = vectorized_det[1] * frame_height
                    detected_width = vectorized_det[2] * frame_width
                    detected_height = vectorized_det[3] * frame_height
                    if not (np.isfinite(center_x) and np.isfinite(center_y) and
                            np.isfinite(detected_width) and np.isfinite(detected_height)):
                        continue
                    if detected_width <= 0 or detected_height <= 0:
                        continue
                    x_coord = int(center_x - detected_width / 2)
                    y_coord = int(center_y - detected_height / 2)
                    assembled_boxes.append([x_coord, y_coord, int(detected_width), int(detected_height)])
                    assembled_confidences.append(float(candidate_confidence))
                    assembled_class_ids.append(chosen_class_id)
        filtered_boxes, filtered_confs, filtered_ids = [], [], []
        for box_vals, conf_val, cid_val in zip(assembled_boxes, assembled_confidences, assembled_class_ids):
            if all(abs(val) < 1e9 for val in box_vals):
                filtered_boxes.append(box_vals)
                filtered_confs.append(conf_val)
                filtered_ids.append(cid_val)
        if len(filtered_boxes) == 0:
            return [], [], [], []
        final_indices = cv2.dnn.NMSBoxes(filtered_boxes, filtered_confs, 0.5, 0.4)
        return final_indices, filtered_boxes, filtered_ids, filtered_confs

    def _annotate_detections(self, base_frame: np.ndarray, selected_indices, stable_boxes, class_id_set, confidence_values):
        for idx in selected_indices:
            idx = idx[0]
            x_val, y_val, w_val, h_val = stable_boxes[idx]
            detected_label = str(self._object_classes[class_id_set[idx]])
            detected_conf = confidence_values[idx]
            if detected_label in ["diningtable", "dining_table", "dining table"]:
                continue
            if detected_label in ["orange", "sports ball", "sportsball", "sports_ball"]:
                detected_label = "apple"
            annotation_color = (0, 255, 0)
            cv2.rectangle(base_frame, (x_val, y_val), (x_val + w_val, y_val + h_val), annotation_color, 2)
            label_text = "{}: {:.2f}".format(detected_label, detected_conf)
            cv2.putText(base_frame, label_text, (x_val, y_val - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, annotation_color, 2)
        return base_frame

    def _process_and_display_frame(self, cv_image: np.ndarray, window_identifier: str):
        unified_frame = cv2.resize(cv_image, (640, 480))
        selected_indices, stable_boxes, stable_ids, stable_confs = self._execute_inference_pipeline(unified_frame)
        annotated_frame = self._annotate_detections(unified_frame, selected_indices, stable_boxes, stable_ids, stable_confs)
        cv2.imshow(window_identifier, annotated_frame)
        cv2.waitKey(1)

    def _ingest_departure_frame(self, msg_data):
        try:
            cv_img = self._bridge_interface.imgmsg_to_cv2(msg_data, "bgr8")
            self._process_and_display_frame(cv_img, "Departure Place Camera")
        except Exception as err:
            rospy.logerr("Departure Frame Ingestion Error: {}".format(err))

    def _ingest_destination_frame(self, msg_data):
        try:
            cv_img = self._bridge_interface.imgmsg_to_cv2(msg_data, "bgr8")
            self._process_and_display_frame(cv_img, "Destination Place Camera")
        except Exception as err:
            rospy.logerr("Destination Frame Ingestion Error: {}".format(err))

if __name__ == '__main__':
    try:
        cortex = Object_detection_module()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Object_detection_module: Execution halted.")
    finally:
        cv2.destroyAllWindows()
