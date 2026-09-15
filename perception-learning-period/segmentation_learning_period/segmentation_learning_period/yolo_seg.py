"""YOLO segmentation model wrapper for inference and visualization using Ultralytics."""

import numpy as np
import torch
from ultralytics import YOLO
from ultralytics.engine.results import Results


class YoloSegmentation:
    """Wrapper class for YOLO segmentation model inference and visualization."""

    def __init__(
        self,
        model_path: str,
        device: str,
        confidence_threshold: float,
        iou: float,
        imgsz: int,
        compile: bool,
        verbose: bool,
    ) -> None:
        self.model_path = model_path
        self.device = device
        self.confidence_threshold = confidence_threshold
        self.iou = iou
        self.imgsz = imgsz
        self.compile = compile
        self.verbose = verbose
        self._model: YOLO = YOLO(model_path, task="segment")

    def predict(self, cv_image: np.ndarray) -> list[Results]:
        """Run prediction on an input image and return the Results object(s)."""
        return self._model.predict(
            source=cv_image,
            imgsz=self.imgsz,
            conf=self.confidence_threshold,
            iou=self.iou,
            device=torch.device(self.device),
            compile=self.compile,
            verbose=self.verbose,
        )

    def visualize(self, result: Results) -> np.ndarray:
        """Generate a visualization image from the Results object."""
        return result.plot()
