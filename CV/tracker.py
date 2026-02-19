import cv2
import numpy as np
from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple


class TrackerState(Enum):
    """Стани трекера."""
    IDLE = 0
    TRACKING = 1
    LOST = 2


@dataclass
class TrackResult:
    """Результат трекінгу."""
    bbox: Optional[Tuple[int, int, int, int]]
    confidence: float
    velocity: Tuple[float, float]


class MILTracker:
    """Легковаговий MIL-based трекер об'єктів з оцінкою швидкості."""
    
    def __init__(self, history_size: int = 10, max_lost_frames: int = 30):
        """
        Args:
            history_size: Розмір історії позицій для розрахунку швидкості
            max_lost_frames: Максимальна кількість кадрів втрати об'єкта
        """
        self.tracker = None
        self.bbox = None
        self.state = TrackerState.IDLE
        self.position_history = deque(maxlen=history_size)
        self.lost_frames = 0
        self.max_lost_frames = max_lost_frames
        self.velocity = (0.0, 0.0)
        
    def init(self, frame: np.ndarray, bbox: Tuple[int, int, int, int]) -> None:
        """
        Ініціалізація трекера з першим кадром і bounding box.
        
        Args:
            frame: Кадр для ініціалізації
            bbox: Bounding box (x, y, width, height)
        """
        self.tracker = cv2.TrackerMIL_create()
        self.tracker.init(frame, bbox)
        self.bbox = bbox
        self.state = TrackerState.TRACKING
        self.lost_frames = 0
        self.position_history.clear()
        self.position_history.append(self._bbox_center(bbox))
        self.velocity = (0.0, 0.0)
        
    def update(self, frame: np.ndarray) -> TrackResult:
        """
        Оновлення трекера з новим кадром.
        
        Args:
            frame: Новий кадр для обробки
            
        Returns:
            TrackResult з bbox, confidence та velocity
        """
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
        
        # Обробка втрати трекінгу
        self.lost_frames += 1
        if self.lost_frames < self.max_lost_frames and self.bbox:
            self.state = TrackerState.LOST
            predicted_bbox = self._predict_bbox()
            return TrackResult(predicted_bbox, 0.3, self.velocity)
        
        # Скидання після багатьох втрачених кадрів
        self.reset()
        return TrackResult(None, 0.0, (0.0, 0.0))
    
    def reset(self) -> None:
        """Скидання трекера в стан очікування."""
        self.tracker = None
        self.bbox = None
        self.state = TrackerState.IDLE
        self.position_history.clear()
        self.lost_frames = 0
        self.velocity = (0.0, 0.0)
    
    @staticmethod
    def _bbox_center(bbox: Tuple[int, int, int, int]) -> Tuple[int, int]:
        """Розрахунок центру bounding box."""
        return (bbox[0] + bbox[2] // 2, bbox[1] + bbox[3] // 2)
    
    def _is_valid_bbox(self, bbox: Tuple, frame_shape: Tuple) -> bool:
        """Перевірка чи bbox знаходиться в межах кадру."""
        h, w = frame_shape[:2]
        cx, cy = self._bbox_center(tuple(map(int, bbox)))
        return 0 <= cx < w and 0 <= cy < h
    
    def _calculate_velocity(self) -> Tuple[float, float]:
        """Розрахунок швидкості з історії позицій."""
        if len(self.position_history) < 2:
            return (0.0, 0.0)
        p1, p2 = self.position_history[-2], self.position_history[-1]
        return (float(p2[0] - p1[0]), float(p2[1] - p1[1]))
    
    def _predict_bbox(self) -> Tuple[int, int, int, int]:
        """Прогнозування наступної позиції bbox за допомогою швидкості."""
        cx, cy = self._bbox_center(self.bbox)
        predicted_cx = int(cx + self.velocity[0] * 2)
        predicted_cy = int(cy + self.velocity[1] * 2)
        w, h = self.bbox[2], self.bbox[3]
        return (predicted_cx - w // 2, predicted_cy - h // 2, w, h)