#ifndef ODRIVE_HPP
#define ODRIVE_HPP

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"
#include <functional>
#include "odrive_interfaces/msg/encoder.hpp"
#include "odrive_interfaces/msg/target.hpp"
#include "odrive_interfaces/msg/bus_voltage.hpp"
#include "odrive_interfaces/msg/torque.hpp"
#include "odrive_interfaces/msg/iq.hpp"
#include "odrive_interfaces/msg/temperature.hpp"
#include "odrive_interfaces/srv/clear_errors.hpp"
#include "odrive_interfaces/srv/reboot.hpp"
#include "odrive_interfaces/srv/set_pos_gain.hpp"
#include "odrive_interfaces/srv/set_vel_gains.hpp"
#include <math.h>
#include "byte_swap.hpp"
#include <chrono>
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"


  enum CMD_IDS{
    Get_Version = 0x000,
    Heartbeat = 0x001,
    Estop = 0x002,
    Get_Error = 0x003,
    RxSdo = 0x004,
    TxSdo = 0x005,
    Set_Axis_Node_ID = 0x006,
    Set_Axis_State = 0x007,
    Get_Encoder_Estimates = 0x009,
    Set_Controller_Mode = 0x00b,
    Set_Input_Pos = 0x00c,
    Set_Input_Vel = 0x00d,
    Set_Input_Torque = 0x00e,
    Set_Limits = 0x00f,
    Set_Traj_Vel_Limit = 0x011,
    Set_Traj_Accel_Limits = 0x012,
    Set_Traj_Inertia = 0x013,
    Get_Iq = 0x014,
    Get_Temperature = 0x015,
    Reboot = 0x016,
    Get_Bus_Voltage_Current = 0x017,
    Clear_Errors = 0x018,
    Set_Absolute_Position = 0x019,
    Set_Pos_Gain = 0x01a,
    Set_Vel_Gains = 0x01b,
    Get_Torques = 0x01c,
    Get_Controller_Error = 0x01d,
    Enter_DFU_Mode = 0x01f
  };


  enum AXIS_STATE {
      UNDEFINED = 0x00,
      IDLE = 0x01,
      STARTUP_SEQUENCE = 0x02,
      FULL_CALIBRATION_SEQUENCE = 0x03,
      MOTOR_CALIBRATION = 0x04,
      ENCODER_INDEX_SEARCH = 0x06,
      ENCODER_OFFSET_CALIBRATION = 0x07,
      CLOSED_LOOP_CONTROL = 0x08,
      LOCKIN_SPIN = 0x09,
      ENCODER_DIR_FIND = 0x0A,
      HOMING = 0x0B,
      ENCODER_HALL_POLARITY_CALIBRATION = 0x0C,
      ENCODER_HALL_PHASE_CALIBRATION = 0x0D
  };


  enum INPUT_MODE{
    INACTIVE = 0x00,
    PASSTHROUGH = 0x1,
    VEL_RAMP = 0x2,
    POS_FILTER = 0x3,
    MIX_CHANNELS=0x4,
    TRAP_TRAJ=0x5,
    TORQUE_RAMP = 0x6,
    MIRROR = 0x7,
    TUNNING = 0x8,
  };

enum CONTROL_MODE{
  VOLTAGE_CONTROL = 0x0, //Not Supported
  TORQUE_CONTROL = 0x1,
  VELOCITY_CONTROL = 0x2,
  POSITION_CONTROL = 0x3,
};


struct version_info{
  uint8_t protocol_version_;
  uint8_t hardware_version_major_;
  uint8_t hardware_version_minor_;
  uint8_t hardware_version_variant_;
  uint8_t firmware_version_major_;
  uint8_t firmware_version_minor_;
  uint8_t firmware_version_revision_;
  uint8_t firmware_version_unreleased_;
};


struct controller_limits{
  float velocity_limit;
  float current_limit;
  float traj_vel_limit;
  float traj_accel_limit;
  float traj_deaccel_limit;
  float traj_inertia;
};


namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class odrive_node: public lc::LifecycleNode{
    
    uint8_t node_id_;
    
    lc::LifecyclePublisher<odrive_interfaces::msg::Encoder>::SharedPtr pub_enc_;
    lc::LifecyclePublisher<odrive_interfaces::msg::BusVoltage>::SharedPtr pub_bus_;
    lc::LifecyclePublisher<odrive_interfaces::msg::Torque>:: SharedPtr pub_torque_;
    lc::LifecyclePublisher<odrive_interfaces::msg::Iq>::SharedPtr pub_iq_;
    lc::LifecyclePublisher<can_msgs::msg::Frame>::SharedPtr pub_can_;
    lc::LifecyclePublisher<odrive_interfaces::msg::Temperature>::SharedPtr pub_temp_;

    rclcpp::Subscription<odrive_interfaces::msg::Target>::SharedPtr sub_target_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;

    void target_callback(const odrive_interfaces::msg::Target::SharedPtr msg);

    void can_callback(const can_msgs::msg::Frame::SharedPtr msg);

    void pub_cmd(float pos, float vel, float torque);

    odrive_interfaces::msg::Encoder enc_msg_;
    odrive_interfaces::msg::BusVoltage bus_msg_;
    odrive_interfaces::msg::Torque torque_msg_;
    odrive_interfaces::msg::Iq iq_msg_;
    odrive_interfaces::msg::Temperature temp_msg_;
    std::string frame_id_;
    version_info version_info_;
    controller_limits limits_;

    bool heart_beat_received_;


    rclcpp::Service<odrive_interfaces::srv::Reboot>::SharedPtr reboot_srv_;
    rclcpp::Service<odrive_interfaces::srv::ClearErrors>::SharedPtr clear_errors_srv_;
    rclcpp::Service<odrive_interfaces::srv::SetPosGain>::SharedPtr set_pos_gain_srv_;
    rclcpp::Service<odrive_interfaces::srv::SetVelGains>::SharedPtr set_vel_gains_srv_;

    CONTROL_MODE ctrl_mode_;
    INPUT_MODE input_mode_;
    rclcpp::TimerBase::SharedPtr timer_;
    float gear_ratio_;


  public:
   
    odrive_node(const rclcpp::NodeOptions & options);

    /// \brief Callback from transition to "configuring" state.
    /// \param[in] state The current state that the node is in.
    LNI::CallbackReturn on_configure(const lc::State & state) override;

    /// \brief Callback from transition to "activating" state.
    /// \param[in] state The current state that the node is in.
    LNI::CallbackReturn on_activate(const lc::State & state) override;

    /// \brief Callback from transition to "deactivating" state.
    /// \param[in] state The current state that the node is in.
    LNI::CallbackReturn on_deactivate(const lc::State & state) override;

    /// \brief Callback from transition to "unconfigured" state.
    /// \param[in] state The current state that the node is in.
    LNI::CallbackReturn on_cleanup(const lc::State & state) override;

    /// \brief Callback from transition to "shutdown" state.
    /// \param[in] state The current state that the node is in.
    LNI::CallbackReturn on_shutdown(const lc::State & state) override;


    /// \brief Callback from transition to "error" state.
    /// \param[in] state The current state that the node is in.
    LNI::CallbackReturn on_error(const lc::State & state) override;

    void init();

    void initialize_services();

    void get_parameters();

    void initialize_publishers();

    void initialize_subscription();

    void get_version_callback(const can_msgs::msg::Frame::SharedPtr msg);

    void get_version();

    void heartbeat_callback(const can_msgs::msg::Frame::SharedPtr msg);

    void get_heartbeat();

    void publish_estop();

    void get_error_callback(const can_msgs::msg::Frame::SharedPtr msg);

    void get_error();

    void set_axis_node_id(uint32_t& node_id);

    void set_axis_state(AXIS_STATE& axis_state);

    void get_encoder_estimates_callback(const can_msgs::msg::Frame::SharedPtr msg);
    
    void set_limits_callback(const can_msgs::msg::Frame::SharedPtr msg);
    
    void get_encoder_estimates();

    bool set_controller_mode(CONTROL_MODE &ctrl_mode, INPUT_MODE &input_mode);

    void set_input_pos(float &pos, float& vel_max, float& torque_max);

    void set_input_vel(float & vel, float &torque_max);

    void set_input_torque(float & torque);

    void set_limits(float & vel_limit, float& curr_limit);

    void set_traj_vel_limit(float &traj_vel_limit);

    void set_traj_accel_limit(float &traj_accel_limit, float & traj_deaccel_limit);

    void set_traj_inertia(float & traj_inetria);

    void get_iq_callback(const can_msgs::msg::Frame::SharedPtr msg);

    void get_temp_callback(const can_msgs::msg::Frame::SharedPtr msg);

    bool reboot();

    void get_bus_voltage_callback(const can_msgs::msg::Frame::SharedPtr msg);

    void get_bus_voltage();

    void clear_errors();

    void set_abs_pos(float & abs_pos);

    void set_pos_gain(float & pos_gain);

    void set_vel_gain(float & vel_gain, float& vel_integrator_gain);

    void get_torques_callback(const can_msgs::msg::Frame::SharedPtr msg);

    void get_controller_error_callback(const can_msgs::msg::Frame::SharedPtr msg);

    void sanity_checker();

    void initialize_controller();

    void get_torques();
};


#endif