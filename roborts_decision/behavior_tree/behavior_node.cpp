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

#include <chrono>
#include <thread>
#include <vector>

//#include "../blackboard/blackboard.h"
//#include "behavior_state.h"
#include "behavior_node.h"

namespace roborts_decision
{
class PatrolAction : public ActionNode
{
public:
  typedef std::shared_ptr<BehaviorNode> Ptr;

  PatrolAction(std::string name, const Blackboard::Ptr &blackboard_ptr, Blackboard *&blackboard, ChassisExecutor *&chassis_executor, const std::string &proto_file_path) : ActionNode::ActionNode(name, blackboard_ptr), patrol_behavior_(chassis_executor, blackboard, proto_file_path)
  {
  }
  ~PatrolAction() {}

private:
  //! executor
  ChassisExecutor *chassis_executor_;
  Blackboard::Ptr blackboard_ptr_;
  std::string proto_file_path_;
  roborts_decision::PatrolBehavior patrol_behavior_;

protected:
  void OnInitialize()
  {

    ROS_INFO("PatrolAction OnInitialize");
  }
  BehaviorState Update()
  {
    patrol_behavior_.Run();
    ROS_INFO("PatrolAction Update");
  }

  void OnTerminate(BehaviorState state)
  {

    ROS_INFO("PatrolAction OnTerminate");
  }
};

class ChaseAction : public ActionNode
{
public:
  typedef std::shared_ptr<BehaviorNode> Ptr;

  ChaseAction(std::string name, const Blackboard::Ptr &blackboard_ptr, Blackboard *&blackboard, ChassisExecutor *&chassis_executor, const std::string &proto_file_path) : ActionNode::ActionNode(name, blackboard_ptr), chase_behavior_(chassis_executor, blackboard, proto_file_path)
  {
  }
  ~ChaseAction() {}

private:
  //! executor
  ChassisExecutor *chassis_executor_;
  Blackboard::Ptr blackboard_ptr_;
  std::string proto_file_path_;
  roborts_decision::ChaseBehavior chase_behavior_;

protected:
  void OnInitialize()
  {

    ROS_INFO("ChaseAction OnInitialize");
  }
  BehaviorState Update()
  {
    chase_behavior_.Run();
    ROS_INFO("ChaseAction Update");
  }

  void OnTerminate(BehaviorState state)
  {

    ROS_INFO("ChaseAction OnTerminate");
  }
};

class EscapeAction : public ActionNode
{
public:
  typedef std::shared_ptr<BehaviorNode> Ptr;

  EscapeAction(std::string name, const Blackboard::Ptr &blackboard_ptr, Blackboard *&blackboard, ChassisExecutor *&chassis_executor, const std::string &proto_file_path) : ActionNode::ActionNode(name, blackboard_ptr), escape_behavior_(chassis_executor, blackboard, proto_file_path)
  {
  }
  ~EscapeAction() {}

private:
  //! executor
  ChassisExecutor *chassis_executor_;
  Blackboard::Ptr blackboard_ptr_;
  std::string proto_file_path_;
  roborts_decision::EscapeBehavior escape_behavior_;

protected:
  void OnInitialize()
  {

    ROS_INFO("EscapeAction OnInitialize");
  }
  BehaviorState Update()
  {
    std::cout << "启动成功逃跑" << std::endl;
    escape_behavior_.Run();
    
    ROS_INFO("EscapeAction Update");
  }

  void OnTerminate(BehaviorState state)
  {
    ROS_INFO("EscapeAction OnTerminate");
  }
};


class SupplyAction : public ActionNode
{
public:
  typedef std::shared_ptr<BehaviorNode> Ptr;

  SupplyAction(std::string name, const Blackboard::Ptr &blackboard_ptr, Blackboard *&blackboard, ChassisExecutor *&chassis_executor, const std::string &proto_file_path) : ActionNode::ActionNode(name, blackboard_ptr), supply_behavior_(chassis_executor, blackboard, proto_file_path)
  {
  }
  ~SupplyAction() {}

private:
  //! executor
  ChassisExecutor *chassis_executor_;
  Blackboard::Ptr blackboard_ptr_;
  std::string proto_file_path_;
  roborts_decision::SupplyBehavior supply_behavior_;

protected:
  void OnInitialize()
  {

    ROS_INFO("SupplyAction OnInitialize");
  }
  BehaviorState Update()
  {
    
    supply_behavior_.Run();
    
    ROS_INFO("SupplyAction Update");
  }

  void OnTerminate(BehaviorState state)
  {

    ROS_INFO("SupplyAction OnTerminate");
  }
};


class SearchAction : public ActionNode
{
public:
  typedef std::shared_ptr<BehaviorNode> Ptr;

  SearchAction(std::string name, const Blackboard::Ptr &blackboard_ptr, Blackboard *&blackboard, ChassisExecutor *&chassis_executor, const std::string &proto_file_path) : ActionNode::ActionNode(name, blackboard_ptr), search_behavior_(chassis_executor, blackboard, proto_file_path)
  {
    this->blackboard_=blackboard;
  }
  ~SearchAction() {}

private:
  //! executor
  ChassisExecutor *chassis_executor_;
  Blackboard *blackboard_;
  std::string proto_file_path_;
  roborts_decision::SearchBehavior search_behavior_;

protected:
  void OnInitialize()
  {

    ROS_INFO("SearchAction OnInitialize");
  }
  BehaviorState Update()
  {
    //敌方消失位置传入
    search_behavior_.SetLastPosition(blackboard_->GetGoal());
    search_behavior_.Run();
    ROS_INFO("SearchAction Update");
  }

  void OnTerminate(BehaviorState state)
  {

    ROS_INFO("SearchAction OnTerminate");
  }
};
} // namespace roborts_decision
