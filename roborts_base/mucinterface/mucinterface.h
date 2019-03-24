/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
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

#ifndef ROBORTS_BASE_MUCINTERFACE_H
#define ROBORTS_BASE_MUCINTERFACE_H

#include "../roborts_sdk/sdk.h"
#include "../ros_dep.h"

namespace roborts_base {
/**
 * @brief ROS API for gimbal module
 */
class MUCinterface {
 public:
  /**
   * @brief Constructor of gimbal including initialization of sdk and ROS
   * @param handle handler of sdk
   */
  MUCinterface(std::shared_ptr<roborts_sdk::Handle> handle);
    /**
   * @brief Destructor of gimbal
   */
  ~MUCinterface() = default;
 private:
  /**
   * @brief Initialization of sdk
   */
  void SDK_Init();
  /**
   * @brief Initialization of ROS
   */
  void ROS_Init();

  /**
   * @brief Gimbal information callback in sdk
   * @param gimbal_info Gimbal information
   */
  void TriggerInfoCallback(const std::shared_ptr<roborts_sdk::cmd_trigger_state> cmd_trigger_sta);
  
  //! sdk handler
  std::shared_ptr<roborts_sdk::Handle> handle_;

  //! ros publisher for odometry information
  ros::Publisher ros_trigger_pub_;

  //! ros node handler
  ros::NodeHandle    ros_nh_;

};
}
#endif //ROBORTS_BASE_GIMBAL_H
