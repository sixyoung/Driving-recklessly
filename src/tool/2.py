import carla
import time
import pygame
import threading

# Box 尺寸
BOX_LENGTH = 2.0
BOX_WIDTH = 4.0
BOX_HEIGHT = 4.0
ARROW_LENGTH = 3.0

# CARLA客户端连接
client = carla.Client("localhost", 2000)
client.set_timeout(5.0)
world = client.get_world()
debug = world.debug

# 构造 road_lane_to_entry_stop_line_ 映射
transform_data = [
    ((118, -1), (-2.04, 16.00, 0.00, 0.0, 0.0, 90.0)),
    ((6, -1), (-2.04, 16.00, 0.00, 0.0, 0.0, 90.0)),
    ((50, -1), (15.88, 2.02, 0.00, 0.0, 0.0, 0.0)),
    ((8, -1), (15.88, 2.02, 0.00, 0.0, 0.0, 0.0)),
    ((118, -2), (-6.04, 16.00, 0.00, 0.0, 0.0, 90.0)),
    ((6, -2), (-6.04, 16.00, 0.00, 0.0, 0.0, 90.0)),
    ((73, -1), (-16.21, -5.97, 0.00, 0.0, 0.0, 180.0)),
    ((7, 2), (-16.21, -5.97, 0.00, 0.0, 0.0, 180.0)),

    ((117, 1), (1.96, -16.00, 0.00, 0.0, 0.0, 270.0)),
    ((5, 1), (1.96, -16.00, 0.00, 0.0, 0.0, 270.0)),
    ((178, 1), (-16.03, -1.97, 0.00, 0.0, 0.0, 180.0)),
    ((7, 1), (-16.03, -1.97, 0.00, 0.0, 0.0, 180.0)),
    ((117, 2), (5.96, -16.00, 0.00, 0.0, 0.0, 270.0)),
    ((5, 2), (5.96, -16.00, 0.00, 0.0, 0.0, 270.0)),
    ((158, 1), (16.19, 6.02, 0.00, 0.0, 0.0, 0.0)),
    ((8, -2), (16.19, 6.02, 0.00, 0.0, 0.0, 0.0)),

    ((139, 1), (-16.00, -1.97, 0.00, 0.0, 0.0, 180.0)),
    ((7, 1), (-16.00, -1.97, 0.00, 0.0, 0.0, 180.0)),
    ((140, 1), (-2.04, 15.95, 0.00, 0.0, 0.0, 90.0)),
    ((6, -1), (-2.04, 15.95, 0.00, 0.0, 0.0, 90.0)),
    ((139, 2), (-16.00, -5.97, 0.00, 0.0, 0.0, 180.0)),
    ((7, 2), (-16.00, -5.97, 0.00, 0.0, 0.0, 180.0)),
    ((64, 1), (5.96, -16.14, 0.00, 0.0, 0.0, 270.0)),
    ((5, 2), (5.96, -16.14, 0.00, 0.0, 0.0, 270.0)),

    ((138, -1), (16.00, 2.02, 0.00, 0.0, 0.0, 0.0)),
    ((8, -1), (16.00, 2.02, 0.00, 0.0, 0.0, 0.0)),
    ((89, -1), (1.96, -15.97, 0.00, 0.0, 0.0, 270.0)),
    ((5, 1), (1.96, -15.97, 0.00, 0.0, 0.0, 270.0)),
    ((138, -2), (16.00, 6.02, 0.00, 0.0, 0.0, 0.0)),
    ((8, -2), (16.00, 6.02, 0.00, 0.0, 0.0, 0.0)),
    ((165, -1), (-6.04, 16.27, 0.00, 0.0, 0.0, 90.0)),
    ((6, -2), (-6.04, 16.27, 0.00, 0.0, 0.0, 90.0)),
]


# 初始化 pygame 获取键盘输入
pygame.init()
win = pygame.display.set_mode((200, 200))

# 控制变量
index = 0
box_id = 0
arrow_id = 1
marker_ids = [None, None]

def draw_box_and_arrow(transform, duration=0.1):
    global marker_ids

    # 删除旧标记（无法直接删除，但我们设置life_time=0.1来“覆盖”）
    for marker_id in marker_ids:
        if marker_id is not None:
            pass  # 无需手动删除，CARLA debug marker 通过 life_time 控制

    location = carla.Location(x=transform.location.x, y=transform.location.y, z=transform.location.z + BOX_HEIGHT / 2.0)
    rotation = transform.rotation

    # Box
    marker_ids[0] = debug.draw_box(
        box=carla.BoundingBox(location, carla.Vector3D(BOX_LENGTH / 2, BOX_WIDTH / 2, BOX_HEIGHT / 2)),
        rotation=rotation,
        thickness=0.2,
        color=carla.Color(255, 0, 0),
        life_time=duration
    )

    # 箭头
    forward_vector = rotation.get_forward_vector() * ARROW_LENGTH
    start = transform.location + carla.Location(z=1.0)
    end = start + carla.Location(x=forward_vector.x, y=forward_vector.y, z=forward_vector.z)

    # ✅ 用位置参数调用 draw_arrow
    marker_ids[1] = debug.draw_arrow(
        start, end, 0.1, 0.3, carla.Color(0, 255, 0), duration, False
    )


# 主循环
def main_loop():
    global index
    clock = pygame.time.Clock()
    while index < len(transform_data):
        # 当前坐标与姿态
        (lane_id, section_id), (x, y, z, roll, pitch, yaw) = transform_data[index]
        transform = carla.Transform(
            location=carla.Location(x=x, y=y, z=z),
            rotation=carla.Rotation(roll=roll, pitch=pitch, yaw=yaw)
        )
        draw_box_and_arrow(transform)

        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
                print(f"[Box {index+1}/{len(transform_data)}] ID=({lane_id}, {section_id}), Pos=({x:.2f}, {y:.2f}, {z:.2f}), Yaw={yaw:.1f}°")
                index += 1
                break
        clock.tick(10)

    pygame.quit()

if __name__ == "__main__":
    main_loop()
