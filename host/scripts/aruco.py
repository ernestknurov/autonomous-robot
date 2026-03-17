import cv2

# Выбери словарь маркеров: DICT_4X4_50, DICT_5X5_100, DICT_6X6_250, DICT_ARUCO_ORIGINAL и т.д.
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Параметры детектора (можно тюнить)
params = cv2.aruco.DetectorParameters()

# Новый API (OpenCV 4.7+): Detector
detector = cv2.aruco.ArucoDetector(aruco_dict, params)

cap = cv2.VideoCapture(0)

while True:
    ok, frame = cap.read()
    if not ok:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is not None:
        # Рисуем рамки и id
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Пример: вывести id в консоль
        print("Corners:", corners)
        print("Rejected:", rejected)
        print("Detected ids:", ids)

    cv2.imshow("aruco", frame)
    if cv2.waitKey(1) & 0xFF == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()
