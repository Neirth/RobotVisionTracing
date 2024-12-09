# FILE: line_detection.py

import cv2

def process_image(image):
    # Convertir la imagen a escala de grises
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Aplicar un umbral para detectar la línea negra
    _, binary = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
    # Encontrar contornos
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def find_largest_contour(contours):
    if contours:
        # Suponiendo que el contorno más grande es la línea
        largest_contour = max(contours, key=cv2.contourArea)
        # Calcular el centro del contorno
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
        else:
            cx, cy = 0, 0
        return cx, cy
    return None, None