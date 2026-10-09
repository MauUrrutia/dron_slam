import cv2
from deskew import determine_skew
from zxingcpp import read_barcodes
from ultralytics import YOLO

MAX_SIDE = 640


def rotate_image(image, angle):
    h, w = image.shape[:2]
    M = cv2.getRotationMatrix2D((w // 2, h // 2), angle, 1.0)
    return cv2.warpAffine(image, M, (w, h), flags=cv2.INTER_CUBIC,
                          borderMode=cv2.BORDER_REPLICATE)


def _decode(gray):
    res = read_barcodes(gray)
    if res:
        return res[0].text.strip()
    return None


def read_code(roi):
    try:
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # Reducir ROIs grandes
        h, w = gray.shape[:2]
        scale = MAX_SIDE / max(h, w)
        if scale < 1.0:
            gray = cv2.resize(gray, None, fx=scale, fy=scale,
                              interpolation=cv2.INTER_AREA)

        # 1) Intento rápido, sin rotar
        text = _decode(gray)
        if text:
            return text

        # 2) Intento con deskew, solo si falló
        angle = determine_skew(gray)
        if angle is not None and abs(angle) > 1.0:
            M = cv2.getRotationMatrix2D(
                (gray.shape[1] // 2, gray.shape[0] // 2), angle, 1.0)
            rot = cv2.warpAffine(gray, M, (gray.shape[1], gray.shape[0]),
                                 flags=cv2.INTER_LINEAR,
                                 borderMode=cv2.BORDER_REPLICATE)
            return _decode(cv2.GaussianBlur(rot, (3, 3), 0))
    except Exception:
        pass
    return None


class Detector:
    def __init__(self, model_path):
        self.model = YOLO(model_path)
        self.frame_count = 0
        self.debug_image = None

    def detect(self, frame, conf=0.5, iou=0.45, roi_extension=50, draw=True):
        self.frame_count += 1
        h, w = frame.shape[:2]
        datos = {"frame_id": self.frame_count, "objetos": []}
        debug = frame.copy() if draw else None

        results = self.model.predict(frame, conf=conf, iou=iou,
                                     verbose=False, device=0)

        for obj in results[0].boxes:
            x1, y1, x2, y2 = map(int, obj.xyxy[0])
            label = self.model.names[int(obj.cls[0])]
            item = {"clase": label,
                    "confianza": round(float(obj.conf[0]), 2),
                    "bbox": [x1, y1, x2, y2],
                    "texto_extraido": None}

            if label in ("Barras", "QR", "Etiqueta"):
                roi = frame[max(0, y1 - roi_extension):min(h, y2 + roi_extension),
                            max(0, x1 - roi_extension):min(w, x2 + roi_extension)]
                if roi.size > 0:
                    item["texto_extraido"] = read_code(roi)

            if draw:
                cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(debug, f"{label} {item['texto_extraido'] or ''}",
                            (x1, max(15, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0), 2)
            datos["objetos"].append(item)

        self.debug_image = debug
        return datos

    def get_debug_image(self):
        return self.debug_image
