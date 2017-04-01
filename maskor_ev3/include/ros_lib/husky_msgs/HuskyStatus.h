#ifndef _ROS_husky_msgs_HuskyStatus_h
#define _ROS_husky_msgs_HuskyStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace husky_msgs
{

  class HuskyStatus : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint32_t uptime;
      double ros_control_loop_freq;
      double mcu_and_user_port_current;
      double left_driver_current;
      double right_driver_current;
      double battery_voltage;
      double left_driver_voltage;
      double right_driver_voltage;
      double left_driver_temp;
      double right_driver_temp;
      double left_motor_temp;
      double right_motor_temp;
      uint16_t capacity_estimate;
      double charge_estimate;
      bool timeout;
      bool lockout;
      bool e_stop;
      bool ros_pause;
      bool no_battery;
      bool current_limit;

    HuskyStatus():
      header(),
      uptime(0),
      ros_control_loop_freq(0),
      mcu_and_user_port_current(0),
      left_driver_current(0),
      right_driver_current(0),
      battery_voltage(0),
      left_driver_voltage(0),
      right_driver_voltage(0),
      left_driver_temp(0),
      right_driver_temp(0),
      left_motor_temp(0),
      right_motor_temp(0),
      capacity_estimate(0),
      charge_estimate(0),
      timeout(0),
      lockout(0),
      e_stop(0),
      ros_pause(0),
      no_battery(0),
      current_limit(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->uptime >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->uptime >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->uptime >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->uptime >> (8 * 3)) & 0xFF;
      offset += sizeof(this->uptime);
      union {
        double real;
        uint64_t base;
      } u_ros_control_loop_freq;
      u_ros_control_loop_freq.real = this->ros_control_loop_freq;
      *(outbuffer + offset + 0) = (u_ros_control_loop_freq.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ros_control_loop_freq.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ros_control_loop_freq.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ros_control_loop_freq.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_ros_control_loop_freq.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_ros_control_loop_freq.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_ros_control_loop_freq.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_ros_control_loop_freq.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->ros_control_loop_freq);
      union {
        double real;
        uint64_t base;
      } u_mcu_and_user_port_current;
      u_mcu_and_user_port_current.real = this->mcu_and_user_port_current;
      *(outbuffer + offset + 0) = (u_mcu_and_user_port_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mcu_and_user_port_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mcu_and_user_port_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mcu_and_user_port_current.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mcu_and_user_port_current.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mcu_and_user_port_current.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mcu_and_user_port_current.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mcu_and_user_port_current.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->mcu_and_user_port_current);
      union {
        double real;
        uint64_t base;
      } u_left_driver_current;
      u_left_driver_current.real = this->left_driver_current;
      *(outbuffer + offset + 0) = (u_left_driver_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_driver_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_driver_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_driver_current.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_left_driver_current.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_left_driver_current.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_left_driver_current.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_left_driver_current.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->left_driver_current);
      union {
        double real;
        uint64_t base;
      } u_right_driver_current;
      u_right_driver_current.real = this->right_driver_current;
      *(outbuffer + offset + 0) = (u_right_driver_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_driver_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_driver_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_driver_current.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_right_driver_current.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_right_driver_current.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_right_driver_current.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_right_driver_current.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->right_driver_current);
      union {
        double real;
        uint64_t base;
      } u_battery_voltage;
      u_battery_voltage.real = this->battery_voltage;
      *(outbuffer + offset + 0) = (u_battery_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery_voltage.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_battery_voltage.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_battery_voltage.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_battery_voltage.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_battery_voltage.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->battery_voltage);
      union {
        double real;
        uint64_t base;
      } u_left_driver_voltage;
      u_left_driver_voltage.real = this->left_driver_voltage;
      *(outbuffer + offset + 0) = (u_left_driver_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_driver_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_driver_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_driver_voltage.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_left_driver_voltage.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_left_driver_voltage.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_left_driver_voltage.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_left_driver_voltage.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->left_driver_voltage);
      union {
        double real;
        uint64_t base;
      } u_right_driver_voltage;
      u_right_driver_voltage.real = this->right_driver_voltage;
      *(outbuffer + offset + 0) = (u_right_driver_voltage.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_driver_voltage.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_driver_voltage.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_driver_voltage.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_right_driver_voltage.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_right_driver_voltage.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_right_driver_voltage.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_right_driver_voltage.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->right_driver_voltage);
      union {
        double real;
        uint64_t base;
      } u_left_driver_temp;
      u_left_driver_temp.real = this->left_driver_temp;
      *(outbuffer + offset + 0) = (u_left_driver_temp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_driver_temp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_driver_temp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_driver_temp.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_left_driver_temp.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_left_driver_temp.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_left_driver_temp.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_left_driver_temp.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->left_driver_temp);
      union {
        double real;
        uint64_t base;
      } u_right_driver_temp;
      u_right_driver_temp.real = this->right_driver_temp;
      *(outbuffer + offset + 0) = (u_right_driver_temp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_driver_temp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_driver_temp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_driver_temp.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_right_driver_temp.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_right_driver_temp.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_right_driver_temp.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_right_driver_temp.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->right_driver_temp);
      union {
        double real;
        uint64_t base;
      } u_left_motor_temp;
      u_left_motor_temp.real = this->left_motor_temp;
      *(outbuffer + offset + 0) = (u_left_motor_temp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_motor_temp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_motor_temp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_motor_temp.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_left_motor_temp.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_left_motor_temp.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_left_motor_temp.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_left_motor_temp.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->left_motor_temp);
      union {
        double real;
        uint64_t base;
      } u_right_motor_temp;
      u_right_motor_temp.real = this->right_motor_temp;
      *(outbuffer + offset + 0) = (u_right_motor_temp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_motor_temp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_motor_temp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_motor_temp.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_right_motor_temp.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_right_motor_temp.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_right_motor_temp.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_right_motor_temp.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->right_motor_temp);
      *(outbuffer + offset + 0) = (this->capacity_estimate >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->capacity_estimate >> (8 * 1)) & 0xFF;
      offset += sizeof(this->capacity_estimate);
      union {
        double real;
        uint64_t base;
      } u_charge_estimate;
      u_charge_estimate.real = this->charge_estimate;
      *(outbuffer + offset + 0) = (u_charge_estimate.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_charge_estimate.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_charge_estimate.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_charge_estimate.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_charge_estimate.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_charge_estimate.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_charge_estimate.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_charge_estimate.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->charge_estimate);
      union {
        bool real;
        uint8_t base;
      } u_timeout;
      u_timeout.real = this->timeout;
      *(outbuffer + offset + 0) = (u_timeout.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->timeout);
      union {
        bool real;
        uint8_t base;
      } u_lockout;
      u_lockout.real = this->lockout;
      *(outbuffer + offset + 0) = (u_lockout.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->lockout);
      union {
        bool real;
        uint8_t base;
      } u_e_stop;
      u_e_stop.real = this->e_stop;
      *(outbuffer + offset + 0) = (u_e_stop.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->e_stop);
      union {
        bool real;
        uint8_t base;
      } u_ros_pause;
      u_ros_pause.real = this->ros_pause;
      *(outbuffer + offset + 0) = (u_ros_pause.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ros_pause);
      union {
        bool real;
        uint8_t base;
      } u_no_battery;
      u_no_battery.real = this->no_battery;
      *(outbuffer + offset + 0) = (u_no_battery.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->no_battery);
      union {
        bool real;
        uint8_t base;
      } u_current_limit;
      u_current_limit.real = this->current_limit;
      *(outbuffer + offset + 0) = (u_current_limit.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->current_limit);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->uptime =  ((uint32_t) (*(inbuffer + offset)));
      this->uptime |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->uptime |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->uptime |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->uptime);
      union {
        double real;
        uint64_t base;
      } u_ros_control_loop_freq;
      u_ros_control_loop_freq.base = 0;
      u_ros_control_loop_freq.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_ros_control_loop_freq.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_ros_control_loop_freq.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_ros_control_loop_freq.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_ros_control_loop_freq.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_ros_control_loop_freq.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_ros_control_loop_freq.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_ros_control_loop_freq.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->ros_control_loop_freq = u_ros_control_loop_freq.real;
      offset += sizeof(this->ros_control_loop_freq);
      union {
        double real;
        uint64_t base;
      } u_mcu_and_user_port_current;
      u_mcu_and_user_port_current.base = 0;
      u_mcu_and_user_port_current.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mcu_and_user_port_current.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mcu_and_user_port_current.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mcu_and_user_port_current.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_mcu_and_user_port_current.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_mcu_and_user_port_current.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_mcu_and_user_port_current.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_mcu_and_user_port_current.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->mcu_and_user_port_current = u_mcu_and_user_port_current.real;
      offset += sizeof(this->mcu_and_user_port_current);
      union {
        double real;
        uint64_t base;
      } u_left_driver_current;
      u_left_driver_current.base = 0;
      u_left_driver_current.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_driver_current.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_driver_current.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_driver_current.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_left_driver_current.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_left_driver_current.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_left_driver_current.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_left_driver_current.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->left_driver_current = u_left_driver_current.real;
      offset += sizeof(this->left_driver_current);
      union {
        double real;
        uint64_t base;
      } u_right_driver_current;
      u_right_driver_current.base = 0;
      u_right_driver_current.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_driver_current.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_driver_current.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_driver_current.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_right_driver_current.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_right_driver_current.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_right_driver_current.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_right_driver_current.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->right_driver_current = u_right_driver_current.real;
      offset += sizeof(this->right_driver_current);
      union {
        double real;
        uint64_t base;
      } u_battery_voltage;
      u_battery_voltage.base = 0;
      u_battery_voltage.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery_voltage.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery_voltage.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery_voltage.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_battery_voltage.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_battery_voltage.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_battery_voltage.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_battery_voltage.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->battery_voltage = u_battery_voltage.real;
      offset += sizeof(this->battery_voltage);
      union {
        double real;
        uint64_t base;
      } u_left_driver_voltage;
      u_left_driver_voltage.base = 0;
      u_left_driver_voltage.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_driver_voltage.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_driver_voltage.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_driver_voltage.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_left_driver_voltage.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_left_driver_voltage.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_left_driver_voltage.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_left_driver_voltage.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->left_driver_voltage = u_left_driver_voltage.real;
      offset += sizeof(this->left_driver_voltage);
      union {
        double real;
        uint64_t base;
      } u_right_driver_voltage;
      u_right_driver_voltage.base = 0;
      u_right_driver_voltage.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_driver_voltage.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_driver_voltage.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_driver_voltage.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_right_driver_voltage.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_right_driver_voltage.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_right_driver_voltage.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_right_driver_voltage.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->right_driver_voltage = u_right_driver_voltage.real;
      offset += sizeof(this->right_driver_voltage);
      union {
        double real;
        uint64_t base;
      } u_left_driver_temp;
      u_left_driver_temp.base = 0;
      u_left_driver_temp.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_driver_temp.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_driver_temp.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_driver_temp.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_left_driver_temp.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_left_driver_temp.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_left_driver_temp.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_left_driver_temp.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->left_driver_temp = u_left_driver_temp.real;
      offset += sizeof(this->left_driver_temp);
      union {
        double real;
        uint64_t base;
      } u_right_driver_temp;
      u_right_driver_temp.base = 0;
      u_right_driver_temp.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_driver_temp.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_driver_temp.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_driver_temp.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_right_driver_temp.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_right_driver_temp.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_right_driver_temp.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_right_driver_temp.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->right_driver_temp = u_right_driver_temp.real;
      offset += sizeof(this->right_driver_temp);
      union {
        double real;
        uint64_t base;
      } u_left_motor_temp;
      u_left_motor_temp.base = 0;
      u_left_motor_temp.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_motor_temp.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_motor_temp.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_motor_temp.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_left_motor_temp.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_left_motor_temp.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_left_motor_temp.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_left_motor_temp.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->left_motor_temp = u_left_motor_temp.real;
      offset += sizeof(this->left_motor_temp);
      union {
        double real;
        uint64_t base;
      } u_right_motor_temp;
      u_right_motor_temp.base = 0;
      u_right_motor_temp.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_motor_temp.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_motor_temp.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_motor_temp.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_right_motor_temp.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_right_motor_temp.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_right_motor_temp.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_right_motor_temp.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->right_motor_temp = u_right_motor_temp.real;
      offset += sizeof(this->right_motor_temp);
      this->capacity_estimate =  ((uint16_t) (*(inbuffer + offset)));
      this->capacity_estimate |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->capacity_estimate);
      union {
        double real;
        uint64_t base;
      } u_charge_estimate;
      u_charge_estimate.base = 0;
      u_charge_estimate.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_charge_estimate.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_charge_estimate.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_charge_estimate.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_charge_estimate.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_charge_estimate.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_charge_estimate.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_charge_estimate.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->charge_estimate = u_charge_estimate.real;
      offset += sizeof(this->charge_estimate);
      union {
        bool real;
        uint8_t base;
      } u_timeout;
      u_timeout.base = 0;
      u_timeout.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->timeout = u_timeout.real;
      offset += sizeof(this->timeout);
      union {
        bool real;
        uint8_t base;
      } u_lockout;
      u_lockout.base = 0;
      u_lockout.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->lockout = u_lockout.real;
      offset += sizeof(this->lockout);
      union {
        bool real;
        uint8_t base;
      } u_e_stop;
      u_e_stop.base = 0;
      u_e_stop.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->e_stop = u_e_stop.real;
      offset += sizeof(this->e_stop);
      union {
        bool real;
        uint8_t base;
      } u_ros_pause;
      u_ros_pause.base = 0;
      u_ros_pause.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->ros_pause = u_ros_pause.real;
      offset += sizeof(this->ros_pause);
      union {
        bool real;
        uint8_t base;
      } u_no_battery;
      u_no_battery.base = 0;
      u_no_battery.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->no_battery = u_no_battery.real;
      offset += sizeof(this->no_battery);
      union {
        bool real;
        uint8_t base;
      } u_current_limit;
      u_current_limit.base = 0;
      u_current_limit.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->current_limit = u_current_limit.real;
      offset += sizeof(this->current_limit);
     return offset;
    }

    const char * getType(){ return "husky_msgs/HuskyStatus"; };
    const char * getMD5(){ return "fd724379c53d89ec4629be3b235dc10d"; };

  };

}
#endif