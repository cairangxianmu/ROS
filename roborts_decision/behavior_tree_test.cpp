#include <ros/ros.h>
#include <iostream>
#include <functional>
#include "executor/chassis_executor.h"

#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/chase_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"
#include "example_behavior/supply_behavior.h"
#include "behavior_tree/behavior_node.cpp"
#include "behavior_tree/behavior_tree.h"
#include "behavior_tree/behavior_node.h"

roborts_decision::PatrolBehavior *patrol_behavior_ptr;

bool goalUpdate()
{
  static int count = 0;
  count++;
  bool flag = false;
  if (count % 1000 > 500)
  {
    flag = true;
  }
  ROS_INFO("goalUpdate %d state:%d-------------------", count,flag);

  return flag;
}

bool supplyUpdate()
{
  static int count = 0;
  count++;
  bool flag = false;
  if (count % 2000 < 100)
  {
    flag = true;
  }
  ROS_INFO("supplyUpdate %d state:%d-------------------", count,flag);

  return flag;
}

bool searchUpdate()
{
  static int count = 0;
  count++;
  bool flag = false;
  if (count % 200 > 100)
  {
    flag = true;
  }
  ROS_INFO("searchUpdate %d state:%d-------------------", count,flag);

  return flag;
}

bool escapeUpdate()
{
  static int count = 0;
  count++;
  bool flag = false;
  if (count % 200 > 100)
  {
    flag = true;
  }
  ROS_INFO("escapeUpdate %d state:%d-------------------", count,flag);

  return flag;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavior_tree_test");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto blackboard = new roborts_decision::Blackboard(full_path);

  roborts_decision::Blackboard::Ptr blackboard_ptr(blackboard);

  std::function<bool()> goal_precondition = std::bind(&roborts_decision::Blackboard::IsEnemyDetected, blackboard);

  std::function<bool()> supply_precondition = std::bind(&roborts_decision::Blackboard::IsNeedSupply, blackboard);

  std::function<bool()> search_precondition = std::bind(&roborts_decision::Blackboard::IsSearch, blackboard);

  std::function<bool()> escape_precondition = std::bind(&roborts_decision::Blackboard::IsEscape, blackboard);

  roborts_decision::SelectorNode::Ptr root_node_ptr(new roborts_decision::SelectorNode("root_node", blackboard_ptr));

  roborts_decision::SelectorNode::Ptr goal_select_node_ptr(new roborts_decision::SelectorNode("goal_select_node", blackboard_ptr));

  roborts_decision::SelectorNode::Ptr search_select_node_ptr(new roborts_decision::SelectorNode("search_select_node", blackboard_ptr));

  roborts_decision::SequenceNode::Ptr sequence_node_ptr(new roborts_decision::SequenceNode("sequence_node", blackboard_ptr));

  roborts_decision::ActionNode::Ptr patrol_action_node_ptr(new roborts_decision::PatrolAction("patrol_action_node", blackboard_ptr, blackboard, chassis_executor, full_path));

  roborts_decision::ActionNode::Ptr chase_action_node_ptr(new roborts_decision::ChaseAction("chase_action_node", blackboard_ptr, blackboard, chassis_executor, full_path));

  roborts_decision::ActionNode::Ptr supply_action_node_ptr(new roborts_decision::SupplyAction("supply_action_node", blackboard_ptr, blackboard, chassis_executor, full_path));

  roborts_decision::ActionNode::Ptr search_action_node_ptr(new roborts_decision::SearchAction("search_action_node", blackboard_ptr, blackboard, chassis_executor, full_path));

  roborts_decision::ActionNode::Ptr escape_action_node_ptr(new roborts_decision::EscapeAction("escape_action_node", blackboard_ptr, blackboard, chassis_executor, full_path));

  roborts_decision::PreconditionNode::Ptr goal_precondition_node_ptr(new roborts_decision::PreconditionNode("goal_precondition_node", blackboard_ptr,
                                                                                                            goalUpdate, roborts_decision::AbortType::BOTH));

  roborts_decision::PreconditionNode::Ptr supply_precondition_node_ptr(new roborts_decision::PreconditionNode("supply_precondition_node", blackboard_ptr,
                                                                                                              supplyUpdate, roborts_decision::AbortType::BOTH));

  roborts_decision::PreconditionNode::Ptr search_precondition_node_ptr(new roborts_decision::PreconditionNode("search_precondition_node", blackboard_ptr,
                                                                                                              searchUpdate, roborts_decision::AbortType::BOTH));

  roborts_decision::PreconditionNode::Ptr escape_precondition_node_ptr(new roborts_decision::PreconditionNode("escape_precondition_node", blackboard_ptr,
                                                                                                              escapeUpdate, roborts_decision::AbortType::BOTH));

  
  // root_node_ptr->AddChildren(supply_precondition_node_ptr);
  root_node_ptr->AddChildren(escape_precondition_node_ptr);
  // root_node_ptr->AddChildren(goal_select_node_ptr);
  escape_precondition_node_ptr->SetChild(escape_action_node_ptr);
  // supply_precondition_node_ptr->SetChild(supply_action_node_ptr);

  // goal_select_node_ptr->AddChildren(goal_precondition_node_ptr);
  // goal_precondition_node_ptr->SetChild(chase_action_node_ptr);
  // goal_select_node_ptr->AddChildren(search_select_node_ptr);
  // search_select_node_ptr->AddChildren(search_precondition_node_ptr);
  // search_precondition_node_ptr->SetChild(search_action_node_ptr);
  // search_select_node_ptr->AddChildren(patrol_action_node_ptr);

  roborts_decision::BehaviorTree node_tree(root_node_ptr, 100);

  node_tree.Run();

  return 0;
}

// root_node_ptr->AddChildren(supply_precondition_node_ptr);
// root_node_ptr->AddChildren(supply_action_node_ptr);
// supply_precondition_node_ptr->SetChild(goal_select_node_ptr);
// goal_select_node_ptr->AddChildren(goal_precondition_node_ptr);
// goal_precondition_node_ptr->SetChild(chase_action_node_ptr);
// goal_select_node_ptr->AddChildren(search_select_node_ptr);
// search_select_node_ptr->AddChildren(search_precondition_node_ptr);
// search_precondition_node_ptr->SetChild(search_action_node_ptr);
// search_select_node_ptr->AddChildren(patrol_action_node_ptr);

// ros::Rate rate(10);
// while(ros::ok()){
//     ros::spinOnce();
//      patrol_behavior.Run();

//     rate.sleep();
//   }
