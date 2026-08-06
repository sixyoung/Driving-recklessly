#ifndef CARLA_VEHICLE_H
#define CARLA_VEHICLE_H

#include <carla/client/Actor.h>
#include <carla/client/Vehicle.h>
#include <carla/client/LightState.h>
#include <carla/client/World.h>
#include <carla/client/Client.h>

namespace common {

    // 启动左转向灯
    void TurnOnLeft(carla::SharedPtr<carla::client::Vehicle> vehicle);

    // 关闭左转向灯
    void TurnOffLeft(carla::SharedPtr<carla::client::Vehicle> vehicle);

    // 启动右转向灯
    void TurnOnRight(carla::SharedPtr<carla::client::Vehicle> vehicle);

    // 关闭右转向灯
    void TurnOffRight(carla::SharedPtr<carla::client::Vehicle> vehicle);

    // 启动刹车灯
    void TurnOnBrake(carla::SharedPtr<carla::client::Vehicle> vehicle);

    // 关闭刹车灯
    void TurnOffBrake(carla::SharedPtr<carla::client::Vehicle> vehicle);

    // 启动远光灯
    void TurnOnHighBeam(carla::SharedPtr<carla::client::Vehicle> vehicle);

    // 关闭远光灯
    void TurnOffHighBeam(carla::SharedPtr<carla::client::Vehicle> vehicle);

    // 启动近光灯
    void TurnOnLowBeam(carla::SharedPtr<carla::client::Vehicle> vehicle);

    // 关闭近光灯
    void TurnOffLowBeam(carla::SharedPtr<carla::client::Vehicle> vehicle);

    // 设置灯光状态
    void SetVehicleLights(carla::SharedPtr<carla::client::Vehicle> vehicle, carla::client::Vehicle::LightState light_state);

    // 获取当前车辆的灯光状态
    carla::client::Vehicle::LightState GetCurrentLightState(carla::SharedPtr<carla::client::Vehicle> vehicle);

    // 检查车辆是否开启某种灯光
    bool IsLightOn(carla::SharedPtr<carla::client::Vehicle> vehicle, carla::client::Vehicle::LightState light_state);
}

#endif  // CARLA_VEHICLE_H
