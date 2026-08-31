""" YOLOv11 based Tic-Tac-Toe game detector.

Autor: Rainer Meyer, r.meyer494@gmail.com
"""
from dataclasses import dataclass
import cv2
import numpy as np
from typing import Tuple, Optional, Sequence, List, Any
from onnxruntime import NodeArg
import onnxruntime as ort


@dataclass
class DetectorOutput:
    indices: Sequence[int]
    boxes: List[Any]
    centers: List[Any]
    confidences: List[Any]
    class_ids: List[Any]


class TTTDetector:

    def __init__(self, model_loc: str):

        # self.providers = [
        #     ('CUDAExecutionProvider', {
        #         'device_id': 0,  # Use the first GPU
        #         'arena_extend_strategy': 'kNextPowerOfTwo',
        #         'gpu_mem_limit': 2 * 1024 * 1024 * 1024,  # Optional: Limit to 2GB
        #     }),
        #     'CPUExecutionProvider'
        # ]
        # options = ort.SessionOptions()
        # options.log_severity_level = 0  # 0 = Verbose, 1 = Info, 2 = Warning, 3 = Error

        # self.session = ort.InferenceSession(model_loc, sess_options=options, providers=self.providers)
        # print(f"Active providers: {self.session.get_providers()}")
        self.session = ort.InferenceSession(model_loc)

        self.input_name: Sequence[NodeArg] = self.session.get_inputs()[0].name

        self.ttt_classes: List[str] = ["O", "Board", "X"]

        self.frame_size: Tuple[int, int] = (640, 640)
        self._conf_threshold: float = 0.25  # Detections under this threshold are not used.
        self._nms_threshold: float = 0.45  # Threshold for non-maximum suppression.

    def detect(self, frame: np.ndarray) -> DetectorOutput:

        # Postprocess frame.
        orig_h, orig_w = frame.shape[:2]
        img_resized = cv2.resize(frame, dsize=self.frame_size)
        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
        input_data = np.transpose(img_rgb, (2, 0, 1)).astype(np.float32) / 255.0
        input_data = np.expand_dims(input_data, axis=0)

        # Run inference. Returns a list of a single np.array of shape=(1, 4+N_CLASS, 8400).
        outputs = self.session.run(None, {self.input_name: input_data})
        output = outputs[0][0].T  # Shape: (8400, 4+N_CLASS).

        # Scale factors.
        x_scale: float = orig_w / float(self.frame_size[0])
        y_scale: float = orig_h / float(self.frame_size[1])

        # Extract bounding boxes, centers, class ID's and confidences.
        boxes, centers, confidences, class_ids = [], [], [], []
        for row in output:
            xc, yc, w, h = row[:4]
            scores = row[4:]
            class_id = np.argmax(scores)  # Index of maximum probability class.
            confidence = scores[class_id]  # Probability of estimation.

            if confidence >= self._conf_threshold:
                # Scale center coordinates back to original image dimensions.
                center_x, center_y = int(xc * x_scale), int(yc * y_scale)
                # Bounding box corner location.
                box_x, box_y = int((xc - w / 2) * x_scale), int((yc - h / 2) * y_scale)
                # Bounding box width and height.
                box_w, box_h = int(w * x_scale), int(h * y_scale)

                boxes.append([box_x, box_y, box_w, box_h])
                centers.append((center_x, center_y))
                confidences.append(float(confidence))
                class_ids.append(int(class_id))

        # Non-maximum suppression to remove duplicate detections.
        indices: Sequence[int] = cv2.dnn.NMSBoxes(boxes, confidences, self._conf_threshold, self._nms_threshold)

        detection: DetectorOutput = DetectorOutput(
            indices=indices,
            boxes=boxes,
            centers=centers,
            confidences=confidences,
            class_ids=class_ids)

        return detection
