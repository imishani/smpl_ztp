//
// Created by itamar on 3/14/23.
//

#ifndef SMPL_ZTP_COLLISION_SPACE_SCENE_MOVEIT_HPP
#define SMPL_ZTP_COLLISION_SPACE_SCENE_MOVEIT_HPP

#include <memory>
#include <vector>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <octomap_msgs/OctomapWithPose.h>

#include <smpl_ztp/ros/moveit_collision_interface.hpp>

#include <sbpl_collision_checking/collision_space.h>
#include <sbpl_collision_checking/shapes.h>

class CollisionSpaceSceneMoveit
{
public:

    void SetCollisionSpace(smpl::collision::moveit_collision_interface* cspace);

    bool SetRobotState(const moveit_msgs::RobotState& robot_state);

    bool ProcessCollisionObjectMsg(const moveit_msgs::CollisionObject& object);
    bool AddCollisionObjectMsg(const moveit_msgs::CollisionObject& object);
    bool RemoveCollisionObjectMsg(const moveit_msgs::CollisionObject& object);
    bool AppendCollisionObjectMsg(const moveit_msgs::CollisionObject& object);
    bool MoveCollisionObjectMsg(const moveit_msgs::CollisionObject& object);

    bool ProcessOctomapMsg(const octomap_msgs::OctomapWithPose& octomap);

    bool ProcessAttachedCollisionObject(
            const moveit_msgs::AttachedCollisionObject& obj);

    bool UpdatePlanningSceneWorld(
            const moveit_msgs::PlanningSceneWorld& world,
            bool is_diff);

    bool UpdatePlanningScene(const moveit_msgs::PlanningScene& scene);

private:


    std::vector<std::unique_ptr<octomap::OcTree>> m_octrees;
    std::vector<std::unique_ptr<smpl::collision::CollisionShape>> m_collision_shapes;
    std::vector<std::unique_ptr<smpl::collision::CollisionObject>> m_collision_objects;

//    smpl::collision::CollisionSpace *m_cspace = nullptr;
    smpl::collision::moveit_collision_interface *m_cspace = nullptr;

    auto FindCollisionObject(const std::string& id) const
    -> smpl::collision::CollisionObject*;
    // -> CollisionObject2*;

    bool CheckCollisionObjectSanity(const moveit_msgs::CollisionObject& object) const;
    bool CheckInsertOctomap(const octomap_msgs::OctomapWithPose& octomap) const;
};

#endif //SMPL_ZTP_COLLISION_SPACE_SCENE_MOVEIT_HPP
