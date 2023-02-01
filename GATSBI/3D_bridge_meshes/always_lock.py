from filelock import FileLock
import time


lock = FileLock('../deep_lab_v3_material_detection/predicted_masks/test.jpeg.lock')
with lock:
    print("Locked")
    time.sleep(10)
