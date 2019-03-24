/****************************************************************************
 *  Copyright (C) 2019 MUC_701.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "mucinterface.h"
#include "roborts_msgs/MucInterface.h"
#include "../roborts_sdk/sdk.h"

namespace roborts_base{
MUCinterface::MUCinterface(std::shared_ptr<roborts_sdk::Handle> handle):
    handle_(handle){
  SDK_Init();
  ROS_Init();
  
}

void MUCinterface::SDK_Init(){
  handle_->CreateSubscriber<roborts_sdk::cmd_trigger_state>(TRIGGER_CMD_SET, CMD_PUSH_TRIG_STATE,
                                                          GIMBAL_ADDRESS, BROADCAST_ADDRESS,
                                                          std::bind(&MUCinterface::TriggerInfoCallback, this, std::placeholders::_1));
 
 }

void MUCinterface::ROS_Init(){
  //ros publisher
  ros_trigger_pub_ = ros_nh_.advertise<roborts_msgs::MucInterface>("car_supply", 30);

}

void MUCinterface::TriggerInfoCallback(const std::shared_ptr<roborts_sdk::cmd_trigger_state> cmd_trigger_sta){

  roborts_msgs::MucInterface muc_interface;

  muc_interface.need_supply = (cmd_trigger_sta->trigger==1);

  muc_interface.blood=1000;
  
  ros_trigger_pub_.publish(muc_interface);

  ROS_INFO("cmd_trigger_state:%d",cmd_trigger_sta->trigger);

}
}

