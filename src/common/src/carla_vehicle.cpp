#include "common/carla_vehicle.h"

namespace common {

    // 设置灯光状态
    void SetVehicleLights(carla::SharedPtr<carla::client::Vehicle> vehicle, carla::client::Vehicle::LightState light_state) {
        if (vehicle) {
            vehicle->SetLightState(light_state);
        }
    }

    // 获取当前车辆的灯光状态
    carla::client::Vehicle::LightState GetCurrentLightState(carla::SharedPtr<carla::client::Vehicle> vehicle) {
        if (vehicle) {
            try{
                return vehicle->GetLightState();
            }catch(const std::runtime_error& e){
                std::cerr << "车辆 " << vehicle->GetId() << " 车灯状态获取失败！" << e.what() << std::endl;
                return carla::client::Vehicle::LightState::None;
            }   

        }
        return carla::client::Vehicle::LightState::None;
    }

    // 启动左转向灯
    void TurnOnLeft(carla::SharedPtr<carla::client::Vehicle> vehicle) {
        if (vehicle) {
            carla::client::Vehicle::LightState left_blinker = carla::client::Vehicle::LightState::LeftBlinker;
            // 通过转换为整数类型进行按位操作
            int current_state = static_cast<int>(GetCurrentLightState(vehicle));
            int new_state = current_state | static_cast<int>(left_blinker);
            SetVehicleLights(vehicle, static_cast<carla::client::Vehicle::LightState>(new_state));
        }
    }

    // 关闭左转向灯
    void TurnOffLeft(carla::SharedPtr<carla::client::Vehicle> vehicle) {
        if (vehicle) {
            carla::client::Vehicle::LightState left_blinker = carla::client::Vehicle::LightState::LeftBlinker;
            // 通过转换为整数类型进行按位操作
            int current_state = static_cast<int>(GetCurrentLightState(vehicle));
            int new_state = current_state & ~static_cast<int>(left_blinker);
            SetVehicleLights(vehicle, static_cast<carla::client::Vehicle::LightState>(new_state));
        }
    }

    // 启动右转向灯
    void TurnOnRight(carla::SharedPtr<carla::client::Vehicle> vehicle) {
        if (vehicle) {
            carla::client::Vehicle::LightState right_blinker = carla::client::Vehicle::LightState::RightBlinker;
            // 通过转换为整数类型进行按位操作
            int current_state = static_cast<int>(GetCurrentLightState(vehicle));
            int new_state = current_state | static_cast<int>(right_blinker);
            SetVehicleLights(vehicle, static_cast<carla::client::Vehicle::LightState>(new_state));
        }
    }

    // 关闭右转向灯
    void TurnOffRight(carla::SharedPtr<carla::client::Vehicle> vehicle) {
        if (vehicle) {
            carla::client::Vehicle::LightState right_blinker = carla::client::Vehicle::LightState::RightBlinker;
            // 通过转换为整数类型进行按位操作
            int current_state = static_cast<int>(GetCurrentLightState(vehicle));
            int new_state = current_state & ~static_cast<int>(right_blinker);
            SetVehicleLights(vehicle, static_cast<carla::client::Vehicle::LightState>(new_state));
        }
    }

    // 启动刹车灯
    void TurnOnBrake(carla::SharedPtr<carla::client::Vehicle> vehicle) {
        if (vehicle) {
            carla::client::Vehicle::LightState brake_light = carla::client::Vehicle::LightState::Brake;
            // 通过转换为整数类型进行按位操作
            int current_state = static_cast<int>(GetCurrentLightState(vehicle));
            int new_state = current_state | static_cast<int>(brake_light);
            SetVehicleLights(vehicle, static_cast<carla::client::Vehicle::LightState>(new_state));
        }
    }

    // 关闭刹车灯
    void TurnOffBrake(carla::SharedPtr<carla::client::Vehicle> vehicle) {
        if (vehicle) {
            carla::client::Vehicle::LightState brake_light = carla::client::Vehicle::LightState::Brake;
            // 通过转换为整数类型进行按位操作
            int current_state = static_cast<int>(GetCurrentLightState(vehicle));
            int new_state = current_state & ~static_cast<int>(brake_light);
            SetVehicleLights(vehicle, static_cast<carla::client::Vehicle::LightState>(new_state));
        }
    }

    // 启动远光灯
    void TurnOnHighBeam(carla::SharedPtr<carla::client::Vehicle> vehicle) {
        if (vehicle) {
            carla::client::Vehicle::LightState high_beam = carla::client::Vehicle::LightState::HighBeam;
            // 通过转换为整数类型进行按位操作
            int current_state = static_cast<int>(GetCurrentLightState(vehicle));
            int new_state = current_state | static_cast<int>(high_beam);
            SetVehicleLights(vehicle, static_cast<carla::client::Vehicle::LightState>(new_state));
        }
    }

    // 关闭远光灯
    void TurnOffHighBeam(carla::SharedPtr<carla::client::Vehicle> vehicle) {
        if (vehicle) {
            carla::client::Vehicle::LightState high_beam = carla::client::Vehicle::LightState::HighBeam;
            // 通过转换为整数类型进行按位操作
            int current_state = static_cast<int>(GetCurrentLightState(vehicle));
            int new_state = current_state & ~static_cast<int>(high_beam);
            SetVehicleLights(vehicle, static_cast<carla::client::Vehicle::LightState>(new_state));
        }
    }

    // 启动近光灯
    void TurnOnLowBeam(carla::SharedPtr<carla::client::Vehicle> vehicle) {
        if (vehicle) {
            carla::client::Vehicle::LightState low_beam = carla::client::Vehicle::LightState::LowBeam;
            // 通过转换为整数类型进行按位操作
            int current_state = static_cast<int>(GetCurrentLightState(vehicle));
            int new_state = current_state | static_cast<int>(low_beam);
            SetVehicleLights(vehicle, static_cast<carla::client::Vehicle::LightState>(new_state));
        }
    }

    // 关闭近光灯
    void TurnOffLowBeam(carla::SharedPtr<carla::client::Vehicle> vehicle) {
        if (vehicle) {
            carla::client::Vehicle::LightState low_beam = carla::client::Vehicle::LightState::LowBeam;
            // 通过转换为整数类型进行按位操作
            int current_state = static_cast<int>(GetCurrentLightState(vehicle));
            int new_state = current_state & ~static_cast<int>(low_beam);
            SetVehicleLights(vehicle, static_cast<carla::client::Vehicle::LightState>(new_state));
        }
    }

    // 检查某种灯光是否开启
    bool IsLightOn(carla::SharedPtr<carla::client::Vehicle> vehicle, carla::client::Vehicle::LightState light_state) {
        if (vehicle) {
            return (static_cast<int>(GetCurrentLightState(vehicle)) & static_cast<int>(light_state)) != 0;
        }
        return false;
    }
}
