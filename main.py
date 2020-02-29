import fedor
import time
import lidar

fedor.scene_reset()
time.sleep(10)

fedor.find_pos()
print('Distance:', lidar.get_distance())

fedor.goto_wall(0, 0.90, True)
print('Distance:', lidar.get_distance())

fedor.press_button(0.6)
print('Distance:', lidar.get_distance())

fedor.press_button(0.6)
print('Distance:', lidar.get_distance())

fedor.press_button(0.6)
print('Distance:', lidar.get_distance())

fedor.go_to()


