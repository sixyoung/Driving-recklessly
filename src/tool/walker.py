import carla, math

client = carla.Client('localhost', 2000)
client.set_timeout(5.0)
world = client.get_world()
bp_lib = world.get_blueprint_library()

# 1) 选一个路障蓝图（也可以试试 trafficcone01/02）
bp = bp_lib.find('static.prop.streetbarrier')

# 2) 选位置：按道路投影并贴地
loc = carla.Location(x=30.0, y=10.0, z=0.0)
wp  = world.get_map().get_waypoint(loc, project_to_road=True)
base = wp.transform  # 自带贴地 z 与朝向 yaw
# 让路障横在车道上：在道路朝向的基础上加 90 度
yaw  = base.rotation.yaw + 90.0
tf   = carla.Transform(base.location, carla.Rotation(pitch=0, yaw=yaw, roll=0))

# 3) 生成
actor = world.try_spawn_actor(bp, tf)
print('spawned:', actor.id if actor else 'failed')
