# ==============================
# Carla 依赖路径配置
# ==============================

# 获取工作空间路径（例如 ~/demo_03）
# get_filename_component(WORKSPACE_PATH ${CMAKE_SOURCE_DIR} DIRECTORY)

# Carla 安装目录
set(libCarla_DIR ${WORKSPACE_PATH}/libcarla)
set(libCarla_INCLUDE_DIR ${libCarla_DIR}/include)
set(libCarla_LIB_DIR     ${libCarla_DIR}/lib)

# Carla 静态 / 动态库
set(Carla_LIBRARIES
    ${libCarla_LIB_DIR}/libboost_filesystem.a 
    ${libCarla_LIB_DIR}/libboost_program_options.a 
    ${libCarla_LIB_DIR}/libboost_system.a 
    ${libCarla_LIB_DIR}/libcarla_client.a 
    ${libCarla_LIB_DIR}/librpc.a 
    ${libCarla_LIB_DIR}/libDebugUtils.a 
    ${libCarla_LIB_DIR}/libDetour.a 
    ${libCarla_LIB_DIR}/libDetourCrowd.a 
    ${libCarla_LIB_DIR}/libDetourTileCache.a 
    ${libCarla_LIB_DIR}/libRecast.a
    ${libCarla_LIB_DIR}/libboost_filesystem.so
    ${libCarla_LIB_DIR}/libboost_program_options.so
    ${libCarla_LIB_DIR}/libboost_system.so
)

# Carla 头文件目录
set(Carla_INCLUDE_DIRS
    ${libCarla_INCLUDE_DIR}
    ${libCarla_INCLUDE_DIR}/carla
    ${libCarla_INCLUDE_DIR}/system
    ${libCarla_INCLUDE_DIR}/moodycamel
    ${libCarla_INCLUDE_DIR}/odrSpiral
    ${libCarla_INCLUDE_DIR}/pugixml
)
