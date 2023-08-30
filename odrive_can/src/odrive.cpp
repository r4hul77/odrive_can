#include "odrive.hpp"



odrive_node::odrive_node(const rclcpp::NodeOptions & options):lc::LifecycleNode("odrive_node",options), sensor_msg_flag_(0b000){
}

CallbackReturn odrive_node::on_configure(const lc::State & state){
  RCLCPP_INFO(this->get_logger(),"Configuring");
  get_parameters();
  initialize_publishers();
  initialize_services();
  return CallbackReturn::SUCCESS;
}

CallbackReturn odrive_node::on_activate(const lc::State & state){
  this->create_bond();
  pub_can_->on_activate();
  pub_bus_->on_activate();
  pub_temp_->on_activate();
  pub_odrv_->on_activate();
  sanity_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),std::bind(&odrive_node::sanity_checker,this));
  sensor_sanity_timer_= this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&odrive_node::sensor_sanity_check, this));
  initialize_subscription();
  get_version();
  initialize_controller();
  return CallbackReturn::SUCCESS;
}

void odrive_node::sensor_sanity_check(){
  static int skipped_msgs = 0;
  while(!sensor_msg_flag_ & 0b111||skipped_msgs>10){
    sensor_sanity_timer_->reset();
    if(!sensor_msg_flag_ & 0b100){
      get_encoder_estimates();
      RCLCPP_WARN(this->get_logger(), "Didn't Receive Encoder Messages this cycle requesting it");
    }
    if(!sensor_msg_flag_ & 0b100){
      get_iq();
      RCLCPP_WARN(this->get_logger(), "Didn't Receive Iq Messages this cycle requesting it");
    }
    if(!sensor_msg_flag_ & 0b001){
      get_torques();
      RCLCPP_WARN(this->get_logger(), "Didn't Receive Torques Messages this cycle requesting it");
    }
    break;
    rclcpp::sleep_for(std::chrono::microseconds(1000));
  }
  sensor_msg_flag_ = 0b000;
  odrive_interfaces::msg::OdriveSensor odrv_msg_;
  odrv_msg_.torque = torque_msg_;
  odrv_msg_.encoder = enc_msg_;
  odrv_msg_.iq = iq_msg_;
  pub_odrv_->publish(std::move(odrv_msg_));
}


CallbackReturn odrive_node::on_deactivate(const lc::State & state){
  pub_cmd(0, 0, 0);
  set_axis_state(AXIS_STATE::IDLE);
  //reset all timers
  sanity_timer_.reset();
  sensor_sanity_timer_.reset();
  //deactivate all publishers
  pub_bus_->on_deactivate();
  pub_can_->on_deactivate();
  pub_temp_->on_deactivate();
  pub_odrv_->on_deactivate();
  //reset all subscriptions
  sub_can_.reset();
  sub_target_.reset();
  //destory bond
  this->destroy_bond();
  RCLCPP_INFO(this->get_logger(),"Deactivating");
  return CallbackReturn::SUCCESS;
}

CallbackReturn odrive_node::on_cleanup(const lc::State & state){
  RCLCPP_INFO(this->get_logger(),"Cleaning Up");
  //reset all publishers
  pub_bus_.reset();
  pub_can_.reset();
  pub_temp_.reset();
  pub_odrv_.reset();

  //reset all services
  reboot_srv_.reset();
  clear_errors_srv_.reset();
  set_pos_gain_srv_.reset();
  set_vel_gains_srv_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn odrive_node::on_error(const lc::State & state){
  RCLCPP_ERROR(this->get_logger(),"Error");
  return CallbackReturn::SUCCESS;
}

CallbackReturn odrive_node::on_shutdown(const lc::State & state){
  RCLCPP_INFO(this->get_logger(),"Shutting Down");
  //reset all publishers
  
  return CallbackReturn::SUCCESS;
}

void odrive_node::initialize_services(){
  // Lets use lambda functions to create the service callbacks
  // the services are as follows reboot_srv_, clear_errors_srv_, set_pos_gain_srv_, set_vel_gains_srv_
  auto reboot_callback = [this](const std::shared_ptr<odrive_interfaces::srv::Reboot::Request> request, std::shared_ptr<odrive_interfaces::srv::Reboot::Response> response) -> void {
    this->reboot();
    response->rebooted = true;
  };
  auto clear_errors_callback = [this](const std::shared_ptr<odrive_interfaces::srv::ClearErrors::Request> req, std::shared_ptr<odrive_interfaces::srv::ClearErrors::Response> res) -> void {
    this->clear_errors();
    res->cleared = true;
  };
  auto set_pos_gain_callback = [this](const std::shared_ptr<odrive_interfaces::srv::SetPosGain::Request> request, std::shared_ptr<odrive_interfaces::srv::SetPosGain::Response> response) -> void {
    this->set_pos_gain(request->pos_gain);
    response->pos_gain_set = true;
  };
  auto set_vel_gains_callback = [this](const std::shared_ptr<odrive_interfaces::srv::SetVelGains::Request> request, std::shared_ptr<odrive_interfaces::srv::SetVelGains::Response> response) -> void {
    this->set_vel_gain(request->vel_gain, request->vel_integrator_gain);
    response->vel_gains_set = true;
  };
  auto set_axis_state_callback = [this](std::shared_ptr<odrive_interfaces::srv::SetAxisState::Request> request, std::shared_ptr<odrive_interfaces::srv::SetAxisState::Response> response) -> void {
    if(!(request->axis_state < 0x0D && request->axis_state > 0x00))
      {
        response->axis_state_set = false;
        RCLCPP_ERROR(this->get_logger(),"Invalid Axis State");
        return;
      }
      this->set_axis_state((AXIS_STATE)request->axis_state);
    response->axis_state_set = true;
  };
  // Create the services and pass the callbacks
  reboot_srv_ = this->create_service<odrive_interfaces::srv::Reboot>(frame_id_ + "/reboot",reboot_callback);
  clear_errors_srv_ = this->create_service<odrive_interfaces::srv::ClearErrors>(frame_id_ + "/clear_errors",clear_errors_callback);
  set_pos_gain_srv_ = this->create_service<odrive_interfaces::srv::SetPosGain>(frame_id_ + "/set_pos_gain",set_pos_gain_callback);
  set_vel_gains_srv_ = this->create_service<odrive_interfaces::srv::SetVelGains>(frame_id_ + "/set_vel_gains",set_vel_gains_callback);
  set_axis_state_srv_ = this->create_service<odrive_interfaces::srv::SetAxisState>(frame_id_ + "/set_axis_state",set_axis_state_callback);
}

void odrive_node::initialize_controller(){
  set_controller_mode(ctrl_mode_,input_mode_);
  set_limits(limits_.velocity_limit,limits_.current_limit);
  set_traj_vel_limit(limits_.traj_vel_limit);
  set_traj_accel_limit(limits_.traj_accel_limit,limits_.traj_deaccel_limit);
  set_traj_inertia(limits_.traj_inertia);
  set_axis_state(AXIS_STATE::CLOSED_LOOP_CONTROL);
}


void odrive_node::sanity_checker(){
  if(!heart_beat_received_){
    this->get_heartbeat();
    if(! heart_beat_received_){
      RCLCPP_ERROR(this->get_logger(),"Heartbeat not received is the Odrive Powered On?");
    }
  }
  heart_beat_received_ = false;
}

void odrive_node::get_parameters(){
  this->declare_parameter("node_id",0);
  this->declare_parameter("control_mode",0x2);
  this->declare_parameter("input_mode",0x2);
  this->declare_parameter("velocity_limit", 25.);
  this->declare_parameter("current_limit", 10.);
  this->declare_parameter("traj_vel_limit", 25.);
  this->declare_parameter("traj_accel_limit", 25.);
  this->declare_parameter("traj_deaccel_limit", 25.);
  this->declare_parameter("traj_inertia", 25.);
  this->declare_parameter("gear_ratio", 20.);

  uint8_t placeholder(0);

  this->get_parameter("node_id",node_id_);
  this->get_parameter("control_mode",placeholder);
  ctrl_mode_ = static_cast<CONTROL_MODE>(placeholder);
  this->get_parameter("input_mode", placeholder);
  input_mode_ = static_cast<INPUT_MODE>(placeholder);
  this->get_parameter("velocity_limit",limits_.velocity_limit);
  limits_.velocity_limit /= 2*M_PI/gear_ratio_;
  this->get_parameter("current_limit",limits_.current_limit);
  this->get_parameter("traj_vel_limit",limits_.traj_vel_limit);
  this->get_parameter("traj_accel_limit",limits_.traj_accel_limit);
  this->get_parameter("traj_deaccel_limit",limits_.traj_deaccel_limit);
  this->get_parameter("traj_inertia",limits_.traj_inertia);
  this->get_parameter("gear_ratio",gear_ratio_);

  this->frame_id_ = "odrive" + std::to_string(node_id_);

  enc_msg_.header.frame_id = frame_id_;
  bus_msg_.header.frame_id = frame_id_;
  torque_msg_.header.frame_id = frame_id_;
  iq_msg_.header.frame_id = frame_id_;
  temp_msg_.header.frame_id = frame_id_;
}

void odrive_node::initialize_publishers(){
  pub_bus_ = this->create_publisher<odrive_interfaces::msg::BusVoltage>(frame_id_ + "/bus_voltage",10);
  pub_can_ = this->create_publisher<can_msgs::msg::Frame>("/to_can_bus",10);
  pub_temp_ = this->create_publisher<odrive_interfaces::msg::Temperature>(frame_id_ + "/temperature",10);
  pub_odrv_ = this->create_publisher<odrive_interfaces::msg::OdriveSensor>(frame_id_ + "/sensor", 25);
}

void odrive_node::initialize_subscription(){
  // Create call back groups for sub_can and sub_target
  auto sub_can_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto sub_target_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // Create sub_can and sub_target
  rclcpp::SubscriptionOptions sub_can_options;
  sub_can_options.callback_group = sub_can_callback_group;
  rclcpp::SubscriptionOptions sub_target_options;
  sub_target_options.callback_group = sub_target_callback_group;
  sub_can_ = this->create_subscription<can_msgs::msg::Frame>("/from_can_bus",50,std::bind(&odrive_node::can_callback,this,std::placeholders::_1),sub_can_options);
  sub_target_ = this->create_subscription<odrive_interfaces::msg::Target>(frame_id_ + "/target",50,std::bind(&odrive_node::target_callback,this,std::placeholders::_1),sub_target_options);
}

void odrive_node::target_callback(const odrive_interfaces::msg::Target::SharedPtr msg){
RCLCPP_DEBUG(this->get_logger(), "pos %lf, vel %lf, torque %lf", msg->pos_des.data, msg->vel_des.data, msg->torque_des.data);
  pub_cmd(msg->pos_des.data*gear_ratio_/(2*M_PI), msg->vel_des.data*gear_ratio_/(2*M_PI), msg->torque_des.data/gear_ratio_);
}

void odrive_node::pub_cmd(const float& pos,const float& vel,const float& torque){
  RCLCPP_DEBUG(this->get_logger(), "pos %f, vel %f, torque %f", pos, vel, torque);
  switch(ctrl_mode_){
    case CONTROL_MODE::POSITION_CONTROL:
      this->set_input_pos(pos,vel, torque);
      break;
    case CONTROL_MODE::VELOCITY_CONTROL:
      this->set_input_vel(vel,torque);
      break;
    case CONTROL_MODE::TORQUE_CONTROL:
      this->set_input_torque(torque);
      break;
    default:
      RCLCPP_ERROR(this->get_logger(),"Invalid Controller Mode Attained Restart the Node");
  }
}

void odrive_node::get_version_callback(const can_msgs::msg::Frame::SharedPtr msg){
  version_info_.protocol_version_ = msg->data[0];
  version_info_.hardware_version_major_ = msg->data[1];
  version_info_.hardware_version_minor_ = msg->data[2];
  version_info_.hardware_version_variant_ = msg->data[3];
  version_info_.firmware_version_major_ = msg->data[4];
  version_info_.firmware_version_minor_ = msg->data[5];
  version_info_.firmware_version_revision_ = msg->data[6];
  version_info_.firmware_version_unreleased_ = msg->data[7];
  RCLCPP_INFO(this->get_logger(),"Protocol Version: %d",version_info_.protocol_version_);
  RCLCPP_INFO(this->get_logger(),"Hardware Version: %d.%d.%d",version_info_.hardware_version_major_,version_info_.hardware_version_minor_,version_info_.hardware_version_variant_);
  RCLCPP_INFO(this->get_logger(),"Firmware Version: %d.%d.%d",version_info_.firmware_version_major_,version_info_.firmware_version_minor_,version_info_.firmware_version_revision_);
}

void odrive_node::get_version(){
  can_msgs::msg::Frame can_frame_;
  can_frame_.id = node_id_ << 5 | CMD_IDS::Get_Version;
  can_frame_.is_extended = false;
  can_frame_.is_rtr = false;
  can_frame_.is_error = false;
  can_frame_.dlc = 0;
  pub_can_->publish(std::move(can_frame_));

}

void odrive_node::create_bond(){
  RCLCPP_INFO(get_logger(), "Creating bond (%s) to lifecycle manager.", this->get_name());

  bond_ = std::make_unique<bond::Bond>(
    std::string("bond"),
    this->get_name(),
    shared_from_this());

  bond_->setHeartbeatPeriod(0.10);
  bond_->setHeartbeatTimeout(4.0);
  bond_->start();

}

void odrive_node::destroy_bond(){
  RCLCPP_INFO(get_logger(), "Destroying bond (%s) to lifecycle manager.", this->get_name());

  if (bond_) {
    bond_.reset();
  }  
}

void odrive_node::heartbeat_callback(const can_msgs::msg::Frame::SharedPtr msg){
  heart_beat_received_ = true;
  uint32_t axis_error_ = read_le<uint32_t>(msg->data.begin());
  uint8_t axis_state = msg->data[4];
  uint8_t procedure_result = msg->data[5];
  bool traj_done_flag = read_le<bool>(msg->data.begin() + 6);
  
  if(axis_error_ != 0){
    RCLCPP_ERROR(this->get_logger(),"Axis Error: %d",axis_error_);
    this->get_error();
    // publish Diag Message
  }
}

void odrive_node::get_heartbeat(){
  can_msgs::msg::Frame can_frame_;

  can_frame_.id = node_id_ << 5 | CMD_IDS::Heartbeat;
  can_frame_.is_extended = false;
  can_frame_.is_rtr = true;
  can_frame_.is_error = false;
  can_frame_.dlc = 0;
  pub_can_->publish(std::move(can_frame_));
}

void odrive_node::publish_estop(){

  can_msgs::msg::Frame can_frame_;
  can_frame_.id = node_id_ << 5 | CMD_IDS::Estop;
  can_frame_.is_extended = false;
  can_frame_.is_rtr = false;
  can_frame_.is_error = false;
  can_frame_.dlc = 0;
  pub_can_->publish(std::move(can_frame_));
}

void odrive_node::get_error_callback(const can_msgs::msg::Frame::SharedPtr msg){
  uint32_t axis_error_ = read_le<uint32_t>(msg->data.begin());
  uint32_t disarm_reason = read_le<uint32_t>(msg->data.begin() + 4);
  
  if(axis_error_ != 0){
    RCLCPP_ERROR(this->get_logger(),"Axis Error: %d",axis_error_);
    RCLCPP_ERROR(this->get_logger(),"Disarm Reason: %d",disarm_reason);
    // publish Diag Message
  }
}

void odrive_node::get_error(){
  can_msgs::msg::Frame can_frame_;

  can_frame_.id = node_id_ << 5 | CMD_IDS::Get_Error;
  can_frame_.is_extended = false;
  can_frame_.is_rtr = true;
  can_frame_.is_error = false;
  can_frame_.dlc = 0;
  pub_can_->publish(std::move(can_frame_));
}

void odrive_node::set_axis_node_id(const uint32_t& node_id){
  can_msgs::msg::Frame can_frame_;
  can_frame_.id = node_id_ << 5 | CMD_IDS::Set_Axis_Node_ID;
  can_frame_.is_extended = false;
  can_frame_.is_rtr = false;
  can_frame_.is_error = false;
  can_frame_.dlc = 4;
  write_le<uint32_t>(node_id, can_frame_.data.begin());
  this->node_id_ = node_id;
  pub_can_->publish(std::move(can_frame_));
}

void odrive_node::set_axis_state(const AXIS_STATE& axis_state){
  can_msgs::msg::Frame can_frame_;

  can_frame_.id = node_id_ << 5 | CMD_IDS::Set_Axis_State;
  can_frame_.is_extended = false;
  can_frame_.is_rtr = false;
  can_frame_.is_error = false;
  can_frame_.dlc = 4;
  write_le<uint32_t>(axis_state, can_frame_.data.begin());
  pub_can_->publish(std::move(can_frame_));
}


void odrive_node::get_encoder_estimates_callback(const can_msgs::msg::Frame::SharedPtr msg){
  enc_msg_.pos.data = read_le<float>(msg->data.begin());
  enc_msg_.pos.data *=  2*M_PI/gear_ratio_;
  enc_msg_.vel.data = read_le<float>(msg->data.begin() + 4);
  enc_msg_.vel.data *= 2*M_PI/gear_ratio_;
  enc_msg_.header.stamp = msg->header.stamp;
  // enc_msg_.pos.data = (float) (msg->data[0] | msg->data[1] << 8 | msg->data[2] << 16 | msg->data[3] << 24) / 100.0;
  // enc_msg_.vel.data = (float) (msg->data[4] | msg->data[5] << 8 | msg->data[6] << 16 | msg->data[7] << 24) / 100.0;
  //pub_enc_->publish(enc_msg_);
  RCLCPP_DEBUG(this->get_logger(), "Encoders Estimates Received");
  sensor_msg_flag_ |= 0b100;
}


void odrive_node::get_encoder_estimates(){
  can_msgs::msg::Frame can_frame_;
  can_frame_.id = node_id_ << 5 | CMD_IDS::Get_Encoder_Estimates;
  can_frame_.is_extended = false;
  can_frame_.is_rtr = true;
  can_frame_.is_error = false;
  can_frame_.dlc = 0;
  pub_can_->publish(std::move(can_frame_));

}


void odrive_node::get_bus_voltage_callback(const can_msgs::msg::Frame::SharedPtr msg){
  bus_msg_.bus_volt.data = read_le<float>(msg->data.begin());
  bus_msg_.bus_current.data = read_le<float>(msg->data.begin() + 4);
  bus_msg_.header.stamp = msg->header.stamp;
  pub_bus_->publish(bus_msg_);
}



bool odrive_node::set_controller_mode(CONTROL_MODE &ctrl_mode, INPUT_MODE &input_mode){
  if((ctrl_mode > 0x3 || ctrl_mode <0x1) && input_mode > 0x7){
    RCLCPP_ERROR(this->get_logger(),"Invalid Controller Mode or Input Mode");
    return false;
  }

  switch(input_mode){
    case INPUT_MODE::INACTIVE:
      break;
    case INPUT_MODE::PASSTHROUGH:
      break;
    case INPUT_MODE::VEL_RAMP:
      switch(ctrl_mode){
        case CONTROL_MODE::VELOCITY_CONTROL:
          break;  
        default:
          RCLCPP_ERROR(this->get_logger(),"Invalid Controller Mode or Input Mode");
          return false;
      }
      break;
    case INPUT_MODE::POS_FILTER:
      if(ctrl_mode != CONTROL_MODE::POSITION_CONTROL){
        RCLCPP_ERROR(this->get_logger(),"Invalid Controller Mode or Input Mode");
        return false;
      }
      break;
    case INPUT_MODE::MIX_CHANNELS:
      RCLCPP_ERROR(this->get_logger(),"MIX_CHANNELS not implemented");
      return false;
    case INPUT_MODE::TRAP_TRAJ:
      if(ctrl_mode != CONTROL_MODE::POSITION_CONTROL){
        RCLCPP_ERROR(this->get_logger(),"Invalid Controller Mode or Input Mode");
        return false;
      }
      break;
    case INPUT_MODE::TORQUE_RAMP:
      if(ctrl_mode != CONTROL_MODE::TORQUE_CONTROL){
        RCLCPP_ERROR(this->get_logger(),"Invalid Controller Mode or Input Mode");
        return false;
      }
      break;
    case INPUT_MODE::MIRROR:
      RCLCPP_ERROR(this->get_logger(),"MIRROR not implemented");
      return false;
    case INPUT_MODE::TUNNING:
      break;
    default:
      RCLCPP_ERROR(this->get_logger(),"Invalid Controller Mode or Input Mode");
      return false;
  }
  can_msgs::msg::Frame can_frame_;
  ctrl_mode_ = ctrl_mode;
  input_mode_ = input_mode;
  can_frame_.id = node_id_ << 5 | CMD_IDS::Set_Controller_Mode;
  can_frame_.is_extended = false;
  can_frame_.is_rtr = false;
  can_frame_.is_error = false;
  can_frame_.dlc = 8;
  write_le<uint32_t>(ctrl_mode_, can_frame_.data.begin());
  write_le<uint32_t>(input_mode_, can_frame_.data.begin() + 4);
  pub_can_->publish(std::move(can_frame_));
  return true;
}

void odrive_node::get_bus_voltage(){
  can_msgs::msg::Frame can_frame_;
  can_frame_.id = node_id_ << 5 | CMD_IDS::Get_Bus_Voltage_Current;
  can_frame_.is_extended = false;
  can_frame_.is_rtr = true;
  can_frame_.is_error = false;
  can_frame_.dlc = 0;
  pub_can_->publish(std::move(can_frame_));

}

void odrive_node::set_input_pos(const float &pos, const float& vel_max, const float& torque_max){
  can_msgs::msg::Frame can_frame_;
  can_frame_.id = node_id_ << 5 | CMD_IDS::Set_Input_Pos;
  can_frame_.is_extended = false;
  can_frame_.is_rtr = false;
  can_frame_.is_error = false;
  can_frame_.dlc = 8;
  write_le<float>(pos, can_frame_.data.begin());
  write_le<int16_t>((int16_t)vel_max*1000, can_frame_.data.begin() + 4);
  write_le<int16_t>((int16_t)torque_max*1000, can_frame_.data.begin() + 6);
  pub_can_->publish(std::move(can_frame_));

}

void odrive_node::set_input_vel(const float & vel, const float &torque_max){
  can_msgs::msg::Frame can_frame_;
  can_frame_.id = node_id_ << 5 | CMD_IDS::Set_Input_Vel;
  can_frame_.is_extended = false;
  can_frame_.is_rtr = false;
  can_frame_.is_error = false;
  can_frame_.dlc = 8;
  write_le<float>(vel, can_frame_.data.begin());
  write_le<float>(torque_max, can_frame_.data.begin() + 4);
  pub_can_->publish(std::move(can_frame_));

}

void odrive_node::set_input_torque(const float & torque){
  can_msgs::msg::Frame can_frame_;
  can_frame_.id = node_id_ << 5 | CMD_IDS::Set_Input_Torque;
  can_frame_.is_extended = false;
  can_frame_.is_rtr = false;
  can_frame_.is_error = false;
  can_frame_.dlc = 4;
  write_le<float>(torque, can_frame_.data.begin());
  pub_can_->publish(std::move(can_frame_));

}

void odrive_node::set_limits(float & vel_limit, float & curr_limit){
  can_msgs::msg::Frame can_frame_;
  can_frame_.id = node_id_ << 5 | CMD_IDS::Set_Limits;
  can_frame_.is_extended = false;
  can_frame_.is_rtr = false;
  can_frame_.is_error = false;
  can_frame_.dlc = 8;
  write_le<float>(vel_limit, can_frame_.data.begin());
  write_le<float>(curr_limit, can_frame_.data.begin() + 4);
  pub_can_->publish(std::move(can_frame_));

}

void odrive_node::set_limits_callback(const can_msgs::msg::Frame::SharedPtr msg){
  float vel_limit_set = read_le<float>(msg->data.begin());
  float curr_limit_set = read_le<float>(msg->data.begin()+4);
  RCLCPP_DEBUG(this->get_logger(), "vel limit: %f, curr limit %f", vel_limit_set, curr_limit_set);
}

void odrive_node::set_traj_vel_limit(float &traj_vel_limit){
  can_msgs::msg::Frame can_frame_;
  can_frame_.id = node_id_ << 5 | CMD_IDS::Set_Traj_Vel_Limit;
  can_frame_.is_extended = false;
  can_frame_.is_rtr = false;
  can_frame_.is_error = false;
  can_frame_.dlc = 4;
  write_le<float>(traj_vel_limit, can_frame_.data.begin());
  pub_can_->publish(std::move(can_frame_));

}

void odrive_node::set_traj_accel_limit(float &traj_accel_limit, float & traj_deaccel_limit){
  can_msgs::msg::Frame can_frame_;
  can_frame_.id = node_id_ << 5 | CMD_IDS::Set_Traj_Accel_Limits;
  can_frame_.is_extended = false;
  can_frame_.is_rtr = false;
  can_frame_.is_error = false;
  can_frame_.dlc = 8;
  write_le<float>(traj_accel_limit, can_frame_.data.begin());
  write_le<float>(traj_deaccel_limit, can_frame_.data.begin() + 4);
  pub_can_->publish(std::move(can_frame_));

}


void odrive_node::set_traj_inertia(float & traj_inetria){
  can_msgs::msg::Frame can_frame_;
  can_frame_.id = node_id_ << 5 | CMD_IDS::Set_Traj_Inertia;
  can_frame_.is_extended = false;
  can_frame_.is_rtr = false;
  can_frame_.is_error = false;
  can_frame_.dlc = 4;
  write_le<float>(traj_inetria, can_frame_.data.begin());
  pub_can_->publish(std::move(can_frame_));
}

void odrive_node::get_iq_callback(const can_msgs::msg::Frame::SharedPtr msg){
  iq_msg_.iq_des.data = read_le<float>(msg->data.begin());
  iq_msg_.iq_est.data = read_le<float>(msg->data.begin() + 4);
  iq_msg_.header.stamp = msg->header.stamp;
  RCLCPP_DEBUG(this->get_logger(), "Iq MSG recived");
  //pub_iq_->publish(iq_msg_);
  sensor_msg_flag_ |= 0b010;
}


void odrive_node::get_temp_callback(const can_msgs::msg::Frame::SharedPtr msg){
  temp_msg_.fet_temp.data = read_le<float>(msg->data.begin());
  temp_msg_.motor_temp.data = read_le<float>(msg->data.begin() + 4);
  temp_msg_.header.stamp = msg->header.stamp;
  pub_temp_->publish(temp_msg_);
}

bool odrive_node::reboot(){
  can_msgs::msg::Frame can_frame_;
  can_frame_.id = node_id_ << 5 | CMD_IDS::Reboot;
  can_frame_.is_extended = false;
  can_frame_.is_rtr = false;
  can_frame_.is_error = false;
  can_frame_.dlc = 0;
  pub_can_->publish(std::move(can_frame_));

  return true;
}

void odrive_node::clear_errors(){
  can_msgs::msg::Frame can_frame_;
  can_frame_.id = node_id_ << 5 | CMD_IDS::Clear_Errors;
  can_frame_.is_extended = false;
  can_frame_.is_rtr = false;
  can_frame_.is_error = false;
  pub_can_->publish(std::move(can_frame_));

}


void odrive_node::set_abs_pos(float & abs_pos){
  can_msgs::msg::Frame can_frame_;
  can_frame_.id = node_id_ << 5 | CMD_IDS::Set_Absolute_Position;
  can_frame_.is_extended = false;
  can_frame_.is_rtr = false;
  can_frame_.is_error = false;
  can_frame_.dlc = 4;
  write_le<float>(abs_pos, can_frame_.data.begin());
  pub_can_->publish(std::move(can_frame_));

}

void odrive_node::set_pos_gain(float & pos_gain){
  can_msgs::msg::Frame can_frame_;
  can_frame_.id = node_id_ << 5 | CMD_IDS::Set_Pos_Gain;
  can_frame_.is_extended = false;
  can_frame_.is_rtr = false;
  can_frame_.is_error = false;
  can_frame_.dlc = 4;
  write_le<float>(pos_gain, can_frame_.data.begin());
  pub_can_->publish(std::move(can_frame_));

}

void odrive_node::set_vel_gain(float &vel_gain, float& vel_integrator_gain){
  can_msgs::msg::Frame can_frame_;
  can_frame_.id = node_id_ << 5 | CMD_IDS::Set_Vel_Gains;
  can_frame_.is_extended = false;
  can_frame_.is_rtr = false;
  can_frame_.is_error = false;
  can_frame_.dlc =8;
  write_le<float>(vel_gain, can_frame_.data.begin());
  write_le<float>(vel_integrator_gain, can_frame_.data.begin() + 4);
  pub_can_->publish(std::move(can_frame_));

}

void odrive_node::get_torques_callback(const can_msgs::msg::Frame::SharedPtr msg){
  torque_msg_.torque_des.data = read_le<float>(msg->data.begin())*gear_ratio_;
  torque_msg_.torque_est.data = read_le<float>(msg->data.begin() + 4)*gear_ratio_;
  torque_msg_.header.stamp = msg->header.stamp;
  //pub_torque_->publish(torque_msg_);
  RCLCPP_ERROR(this->get_logger(), "Torques MSG received torques des %f torques est %f", torque_msg_.torque_des.data, torque_msg_.torque_est.data);
  sensor_msg_flag_ |= 0b001;
}

void odrive_node::get_torques(){
  can_msgs::msg::Frame can_frame_;
  can_frame_.id = node_id_ << 5 | CMD_IDS::Get_Torques;
  can_frame_.is_extended = false;
  can_frame_.is_rtr = true;
  can_frame_.is_error = false;
  can_frame_.dlc = 0;
  pub_can_->publish(std::move(can_frame_));

}

void odrive_node::get_iq(){
  can_msgs::msg::Frame can_frame_;
  can_frame_.id = node_id_ << 5 | CMD_IDS::Get_Iq;
  can_frame_.is_extended = false;
  can_frame_.is_rtr = true;
  can_frame_.is_error = false;
  can_frame_.dlc = 0;
  pub_can_->publish(std::move(can_frame_));

}

void odrive_node::get_controller_error_callback(const can_msgs::msg::Frame::SharedPtr msg){
  uint32_t controller_error = read_le<uint32_t>(msg->data.begin());
  if(controller_error != 0){
    RCLCPP_ERROR(this->get_logger(),"Controller Error: %d",controller_error);
  }
}

void odrive_node::can_callback(const can_msgs::msg::Frame::SharedPtr msg){
  if(node_id_ != (msg->id >> 5)){
    return;
  }  
  switch(msg->id & 0x1F){
    case CMD_IDS::Heartbeat:
      this->heartbeat_callback(msg);
      break;
    case CMD_IDS::Get_Error:
      this->get_error_callback(msg);
      break;
    case CMD_IDS::Get_Iq:
      this->get_iq_callback(msg);
      break;
    case CMD_IDS::Get_Encoder_Estimates:
      this->get_encoder_estimates_callback(msg);
      break;
    case CMD_IDS::Get_Bus_Voltage_Current:
      this->get_bus_voltage_callback(msg);
      break;
    case CMD_IDS::Get_Temperature:
      this->get_temp_callback(msg);
      break;
    case CMD_IDS::Get_Torques:
      this->get_torques_callback(msg);
      break;
    case CMD_IDS::Get_Controller_Error:
      this->get_controller_error_callback(msg);
      break;
    case CMD_IDS::Get_Version:
      this->get_version_callback(msg);
      break;

    break;
    default:
      RCLCPP_INFO(this->get_logger(), "Unknown Mesage received with ID %x", msg->id & 0x1F);
      }

  }


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(odrive_node)


