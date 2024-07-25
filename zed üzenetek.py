import pyzed.sl as sl
import numpy as np
import cv2
import time

def is_camera_covered(image, depth, brightness_threshold=50, depth_std_threshold=0.1):
    """
    Ellenőrzi, hogy a kamera látómezeje el van-e takarva.

    :param image: Az RGB kép (sl.Mat)
    :param depth: A mélységi kép (sl.Mat)
    :param brightness_threshold: A fényerő küszöbértéke
    :param depth_std_threshold: A mélységi adatok szórásának küszöbértéke
    :return: True, ha a kamera el van takarva, különben False
    """
    # Konvertáljuk a képeket numpy tömbökké
    image_np = image.get_data()
    depth_np = depth.get_data()

    # Ellenőrizzük az RGB kép átlagos fényerejét
    gray_image = cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
    mean_brightness = np.mean(gray_image)
    if mean_brightness < brightness_threshold or mean_brightness > 255 - brightness_threshold:
        return True

    # Ellenőrizzük a mélységi adatokat
    if np.std(depth_np) < depth_std_threshold:
        return True

    return False

def check_camera_connection(zed):
    """
    Ellenőrzi, hogy a kamera csatlakoztatva van-e, és képes-e adatokat lekérni.

    :param zed: ZED Mini kamera objektum
    :return: True, ha a kamera csatlakoztatva van és képes adatokat lekérni, különben False
    """
    if not zed.is_opened():
        print("Nincs jel / lecsatlakozva!")
        return False
    return True

def main():
    # Create a Camera object
    zed = sl.Camera()

    # Set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_fps = 30  # Frame rate
    init_params.coordinate_units = sl.UNIT.METER  # Use meters as coordinate unit
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP  # Use a right-handed coordinate system

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open ZED Mini camera: {err}")
        exit(1)

    # Enable positional tracking with default parameters
    tracking_params = sl.PositionalTrackingParameters()
    zed.enable_positional_tracking(tracking_params)

    # Capture loop
    runtime_params = sl.RuntimeParameters()
    image = sl.Mat()
    depth = sl.Mat()

    try:
        while True:
            # Check if the camera is still connected
            if not check_camera_connection(zed):
                break

            # Try to grab a new frame
            err = zed.grab(runtime_params)
            if err != sl.ERROR_CODE.SUCCESS:
                print(f"Nincs jel / lecsatlakozva! Hiba: {err}")
                if err == sl.ERROR_CODE.CAMERA_NOT_DETECTED:
                    print("ZED Mini kamera lecsatlakozott!")
                time.sleep(1)
                continue

            # Retrieve image and depth
            zed.retrieve_image(image, sl.VIEW.LEFT)
            zed.retrieve_measure(depth, sl.MEASURE.DEPTH)

            # Check if the camera is covered
            if is_camera_covered(image, depth):
                print("Kamera kép eltakarva")

            # Delay to reduce print frequency
            time.sleep(1)

    except KeyboardInterrupt:
        print("Stopping the capture loop...")

    # Disable positional tracking and close the camera
    zed.disable_positional_tracking()
    zed.close()

if __name__ == "__main__":
    main()