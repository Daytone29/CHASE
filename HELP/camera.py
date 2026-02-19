import cv2
import numpy as np
from picamera2 import Picamera2
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

class TrackerState(Enum):
    IDLE = 0
    TRACKING = 1
    LOST = 2

@dataclass
class TrackResult:
    bbox: Optional[Tuple[int, int, int, int]]
    confidence: float
    velocity: Tuple[float, float]

class MILTracker:
    """Lightweight MIL-based object tracker with velocity estimation."""
    
    def __init__(self, history_size: int = 10, max_lost_frames: int = 30):
        self.tracker = None
        self.bbox = None
        self.state = TrackerState.IDLE
        self.position_history = deque(maxlen=history_size)
        self.lost_frames = 0
        self.max_lost_frames = max_lost_frames
        self.velocity = (0.0, 0.0)
        
    def init(self, frame: np.ndarray, bbox: Tuple[int, int, int, int]) -> None:
        """Initialize tracker with first frame and bounding box."""
        self.tracker = cv2.TrackerMIL_create()
        self.tracker.init(frame, bbox)
        self.bbox = bbox
        self.state = TrackerState.TRACKING
        self.lost_frames = 0
        self.position_history.clear()
        self.position_history.append(self._bbox_center(bbox))
        self.velocity = (0.0, 0.0)
        
    def update(self, frame: np.ndarray) -> TrackResult:
        """Update tracker with new frame."""
        if self.state == TrackerState.IDLE:
            return TrackResult(None, 0.0, (0.0, 0.0))
        
        success, bbox = self.tracker.update(frame)
        
        if success and self._is_valid_bbox(bbox, frame.shape):
            bbox = tuple(map(int, bbox))
            self.bbox = bbox
            self.position_history.append(self._bbox_center(bbox))
            self.velocity = self._calculate_velocity()
            self.lost_frames = 0
            self.state = TrackerState.TRACKING
            return TrackResult(bbox, 1.0, self.velocity)
        
        # Handle lost tracking
        self.lost_frames += 1
        if self.lost_frames < self.max_lost_frames and self.bbox:
            self.state = TrackerState.LOST
            predicted_bbox = self._predict_bbox()
            return TrackResult(predicted_bbox, 0.3, self.velocity)
        
        # Reset after too many lost frames
        self.reset()
        return TrackResult(None, 0.0, (0.0, 0.0))
    
    def reset(self) -> None:
        """Reset tracker to idle state."""
        self.tracker = None
        self.bbox = None
        self.state = TrackerState.IDLE
        self.position_history.clear()
        self.lost_frames = 0
        self.velocity = (0.0, 0.0)
    
    @staticmethod
    def _bbox_center(bbox: Tuple[int, int, int, int]) -> Tuple[int, int]:
        """Calculate bounding box center."""
        return (bbox[0] + bbox[2] // 2, bbox[1] + bbox[3] // 2)
    
    def _is_valid_bbox(self, bbox: Tuple, frame_shape: Tuple) -> bool:
        """Check if bounding box is within frame boundaries."""
        h, w = frame_shape[:2]
        cx, cy = self._bbox_center(tuple(map(int, bbox)))
        return 0 <= cx < w and 0 <= cy < h
    
    def _calculate_velocity(self) -> Tuple[float, float]:
        """Calculate velocity from position history."""
        if len(self.position_history) < 2:
            return (0.0, 0.0)
        p1, p2 = self.position_history[-2], self.position_history[-1]
        return (float(p2[0] - p1[0]), float(p2[1] - p1[1]))
    
    def _predict_bbox(self) -> Tuple[int, int, int, int]:
        """Predict next bounding box position using velocity."""
        cx, cy = self._bbox_center(self.bbox)
        predicted_cx = int(cx + self.velocity[0] * 2)
        predicted_cy = int(cy + self.velocity[1] * 2)
        w, h = self.bbox[2], self.bbox[3]
        return (predicted_cx - w // 2, predicted_cy - h // 2, w, h)

class TrackingPipeline:
    """Interactive tracking pipeline with GUI."""
    
    def __init__(self, resolution: Tuple[int, int] = (640, 480)):
        self.resolution = resolution
        self.camera = self._init_camera()
        self.tracker = MILTracker()
        self.crosshair = (resolution[0] // 2, resolution[1] // 2)
        
        # Selection state
        self.selecting = False
        self.selection_start = None
        self.selection_rect = None
        self.pending_init = None
        
    def _init_camera(self) -> Picamera2:
        """Initialize Picamera2 with configuration."""
        camera = Picamera2()
        config = camera.create_preview_configuration(
            raw={"size": (1640, 1232)},
            main={"format": "RGB888", "size": self.resolution}
        )
        camera.configure(config)
        return camera
    
    def _mouse_callback(self, event: int, x: int, y: int, flags: int, param) -> None:
        """Handle mouse events for target selection."""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.selecting = True
            self.selection_start = (x, y)
            self.selection_rect = None
            
        elif event == cv2.EVENT_MOUSEMOVE and self.selecting:
            self.selection_rect = (
                min(self.selection_start[0], x),
                min(self.selection_start[1], y),
                abs(x - self.selection_start[0]),
                abs(y - self.selection_start[1])
            )
            
        elif event == cv2.EVENT_LBUTTONUP and self.selecting:
            self.selecting = False
            if self.selection_rect and min(self.selection_rect[2:]) > 10:
                self.pending_init = self.selection_rect
    
    def _draw_overlay(self, frame: np.ndarray, result: TrackResult) -> np.ndarray:
        """Draw tracking overlay on frame."""
        h, w = frame.shape[:2]
        
        # Draw crosshair
        cv2.drawMarker(frame, self.crosshair, (0, 255, 0), cv2.MARKER_CROSS, 40, 1)
        
        # Draw tracking info
        if result.bbox:
            x, y, bw, bh = result.bbox
            color = (0, 255, 0) if self.tracker.state == TrackerState.TRACKING else (0, 255, 255)
            
            cv2.rectangle(frame, (x, y), (x + bw, y + bh), color, 2)
            
            target_cx, target_cy = x + bw // 2, y + bh // 2
            cv2.circle(frame, (target_cx, target_cy), 4, color, -1)
            cv2.line(frame, self.crosshair, (target_cx, target_cy), color, 1)
            
            offset_x = target_cx - self.crosshair[0]
            offset_y = target_cy - self.crosshair[1]
            
            cv2.putText(frame, f"OFF: {offset_x:+d}, {offset_y:+d}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, f"VEL: {result.velocity[0]:+.1f}, {result.velocity[1]:+.1f}", (10, 50),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Draw selection rectangle
        if self.selection_rect:
            x, y, sw, sh = self.selection_rect
            cv2.rectangle(frame, (x, y), (x + sw, y + sh), (255, 0, 0), 2)
        
        # Draw state
        cv2.putText(frame, self.tracker.state.name, (w - 100, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return frame
    
    def get_tracking_offset(self, result: TrackResult) -> Optional[Tuple[int, int]]:
        """Get offset between target center and crosshair."""
        if not result.bbox:
            return None
        target_cx = result.bbox[0] + result.bbox[2] // 2
        target_cy = result.bbox[1] + result.bbox[3] // 2
        return (target_cx - self.crosshair[0], target_cy - self.crosshair[1])
    
    def run(self) -> None:
        """Run main tracking loop."""
        self.camera.start()
        cv2.namedWindow("Tracker")
        cv2.setMouseCallback("Tracker", self._mouse_callback)
        
        try:
            while True:
                frame = self.camera.capture_array()
                
                if self.pending_init:
                    self.tracker.init(frame, self.pending_init)
                    self.pending_init = None
                
                result = self.tracker.update(frame)
                display = self._draw_overlay(frame.copy(), result)
                cv2.imshow("Tracker", display)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    self.tracker.reset()
                elif key == ord('c'):
                    self.crosshair = (self.resolution[0] // 2, self.resolution[1] // 2)
        finally:
            self.camera.stop()
            self.camera.close()
            cv2.destroyAllWindows()

class HeadlessPipeline:
    """Headless tracking pipeline for embedded systems."""
    
    def __init__(self, resolution: Tuple[int, int] = (640, 480)):
        self.resolution = resolution
        self.camera = self._init_camera()
        self.tracker = MILTracker()
        self.crosshair = (resolution[0] // 2, resolution[1] // 2)
        self.running = False
        
    def _init_camera(self) -> Picamera2:
        """Initialize Picamera2 with configuration."""
        camera = Picamera2()
        config = camera.create_preview_configuration(
            raw={"size": (1640, 1232)},
            main={"format": "RGB888", "size": self.resolution}
        )
        camera.configure(config)
        return camera
    
    def start(self) -> None:
        """Start camera and pipeline."""
        self.camera.start()
        self.running = True
        
    def stop(self) -> None:
        """Stop camera and pipeline."""
        self.running = False
        self.camera.stop()
        self.camera.close()
        
    def init_target(self, bbox: Tuple[int, int, int, int]) -> None:
        """Initialize tracking target."""
        frame = self.camera.capture_array()
        self.tracker.init(frame, bbox)
        
    def process_frame(self) -> Tuple[Optional[TrackResult], Optional[Tuple[int, int]]]:
        """Process single frame and return tracking result and offset."""
        if not self.running:
            return None, None
            
        frame = self.camera.capture_array()
        result = self.tracker.update(frame)
        
        offset = None
        if result.bbox:
            target_cx = result.bbox[0] + result.bbox[2] // 2
            target_cy = result.bbox[1] + result.bbox[3] // 2
            offset = (target_cx - self.crosshair[0], target_cy - self.crosshair[1])
            
        return result, offset

if __name__ == "__main__":
    pipeline = TrackingPipeline(resolution=(640, 480))
    pipeline.run()