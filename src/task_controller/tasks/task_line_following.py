from ..common_modules.object_detection.line_detection import process_image, find_largest_contour

class TaskLineFollowing:
    def __init__(self, motion_control, camera_processing):
        self.motion_control = motion_control
        self.camera_processing = camera_processing

    def follow_line(self):
        # Obtener la imagen de la cámara
        image = self.camera_processing.get_image()
        # Procesar la imagen para encontrar la línea
        contours = process_image(image)
        
        # Encontrar el contorno más grande y su centro
        cx, cy = find_largest_contour(contours)

        if cx is not None and cy is not None:
            # Calcular el error de posición
            error = cx - image.shape[1] // 2

            # Ajustar el movimiento del robot
            self.motion_control.adjust_direction(error)
        else:
            # Si no se encuentra la línea, detener el robot
            self.motion_control.stop()
