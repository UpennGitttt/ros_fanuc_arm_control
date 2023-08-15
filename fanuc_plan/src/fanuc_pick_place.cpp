#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <iostream>

#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseArray.h>

struct ObjectPoseInfo
{
    double totalDistance;
    double startDistance;
    double endDistance;
    geometry_msgs::Pose pose;
};

class PickAndPlaceManager
{
public:
    Eigen::Vector3d objectTranslation;
    Eigen::Quaterniond objectOrientation;
    bool isFirstRecognition;
    bool isSecondRecognition;
    bool isReadyForPick;
    bool isReadyForPlace;

public:
    void objectVisualCallback(const geometry_msgs::PoseStamped &odomData);
    void generateObjectPoses(geometry_msgs::PoseArray &objectPoses);
    std::vector<ObjectPoseInfo> findKClosestPoses(const geometry_msgs::PoseArray &objectPoses,
                                                  const geometry_msgs::PoseStamped &start,
                                                  const geometry_msgs::PoseStamped &end,
                                                  int k);
    std::vector<ObjectPoseInfo> determineSequence(std::vector<ObjectPoseInfo> &nearbyPoses,
                                                                       int count,
                                                                       const geometry_msgs::PoseStamped &initialPose);
};

void PickAndPlaceManager::generateObjectPoses(geometry_msgs::PoseArray &objectPoses)
{
    srand(static_cast<unsigned>(time(0)));
    for (int i = 0; i < 20; i++)
    {
        Eigen::Vector3f randomPosition;
        geometry_msgs::Pose tempPose;
        double lowerBound = -0.37;
        double upperBound = 0.37;

        randomPosition(0) = 0.5 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (0.8 - 0.55)));
        randomPosition(1) = -0.3 + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (0.25 - (-0.3))));

        tempPose.position.x = randomPosition(0);
        tempPose.position.y = randomPosition(1);
        tempPose.position.z = 0.05;
        objectPoses.poses.push_back(tempPose);
    }
}

std::vector<ObjectPoseInfo> PickAndPlaceManager::findKClosestPoses(const geometry_msgs::PoseArray &objectPoses,
                                                                   const geometry_msgs::PoseStamped &start,
                                                                   const geometry_msgs::PoseStamped &end,
                                                                   int k)
{
    int totalPoses = objectPoses.poses.size();
    std::vector<ObjectPoseInfo> poseDetails;
    ObjectPoseInfo tempPoseInfo;
    for (int i = 0; i < totalPoses; i++)
    {
        double dxFromStart = objectPoses.poses[i].position.x - start.pose.position.x;
        double dyFromStart = objectPoses.poses[i].position.y - start.pose.position.y;
        double dzFromStart = objectPoses.poses[i].position.z - start.pose.position.z;
        double dxFromEnd = objectPoses.poses[i].position.x - end.pose.position.x;
        double dyFromEnd = objectPoses.poses[i].position.y - end.pose.position.y;
        double dzFromEnd = objectPoses.poses[i].position.z - end.pose.position.z;
        tempPoseInfo.startDistance = std::sqrt(dxFromStart * dxFromStart + dyFromStart * dyFromStart + dzFromStart * dzFromStart);
        tempPoseInfo.endDistance = std::sqrt(dxFromEnd * dxFromEnd + dyFromEnd * dyFromEnd + dzFromEnd * dzFromEnd);
        tempPoseInfo.totalDistance = tempPoseInfo.startDistance + tempPoseInfo.endDistance;
        tempPoseInfo.pose = objectPoses.poses[i];
        poseDetails.push_back(tempPoseInfo);
    }
    sort(poseDetails.begin(), poseDetails.end(), [](ObjectPoseInfo a, ObjectPoseInfo b)
    { return a.totalDistance <= b.totalDistance; });

    std::vector<ObjectPoseInfo> closestPoses(poseDetails.begin(), poseDetails.begin() + k);
    return closestPoses;
}

std::vector<ObjectPoseInfo> PickAndPlaceManager::determineSequence(std::vector<ObjectPoseInfo>& nearbyPoses, int k, const geometry_msgs::PoseStamped& referencePose)
{
    std::vector<ObjectPoseInfo> sequence;

    for (int i = 0; i < k; i++)
    {
        sort(nearbyPoses.begin(), nearbyPoses.end(), [](ObjectPoseInfo poseA, ObjectPoseInfo poseB)
        { return poseA.startDistance < poseB.startDistance; });

        sequence.push_back(nearbyPoses[0]);
        for (int j = 1; j < nearbyPoses.size(); j++)
        {
            double dx = nearbyPoses[j].pose.position.x - nearbyPoses[0].pose.position.x;
            double dy = nearbyPoses[j].pose.position.y - nearbyPoses[0].pose.position.y;
            double dz = nearbyPoses[j].pose.position.z - nearbyPoses[0].pose.position.z;
            nearbyPoses[j].startDistance = std::sqrt(dx * dx + dy * dy + dz * dz);
        }
        nearbyPoses.erase(nearbyPoses.begin());
    }

    return sequence;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_pick_place");
    ros::NodeHandle nodeHandle;
    ros::AsyncSpinner asyncSpinner(7);
    asyncSpinner.start();

    PickAndPlaceManager pickPlaceManager;

    bool isPrePick = true;
    bool isPick = false;
    bool isPrePlace = false;
    bool isPlace = false;
    bool isSequence = true;

    geometry_msgs::PoseArray objectPoses;
    std::vector<ObjectPoseInfo> closestPoses;
    std::vector<ObjectPoseInfo> sequence;
    geometry_msgs::PoseStamped startPose;
    geometry_msgs::PoseStamped endPose;
    startPose.pose.position.x = 0.56;
    startPose.pose.position.y = 0.12;
    startPose.pose.position.z = 0.11;
    endPose.pose.position.x = 0.32;
    endPose.pose.position.y = -0.12;
    endPose.pose.position.z = 0.03;

    pickPlaceManager.generateObjectPoses(objectPoses);
    closestPoses = pickPlaceManager.findKClosestPoses(objectPoses, startPose, endPose, 12);
    sequence = pickPlaceManager.determineSequence(closestPoses, 12, startPose);

    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    arm.allowReplanning(true);
    arm.setGoalJointTolerance(0.001);
    arm.setGoalPositionTolerance(0.001);
    arm.setGoalOrientationTolerance(0.01);
    arm.setMaxAccelerationScalingFactor(1.0);
    arm.setMaxVelocityScalingFactor(1.0);

    std::string endEffector = arm.getEndEffectorLink();
    std::cout << "End effector link: " << endEffector << std::endl;
    std::string referenceFrame = "base_link";
    arm.setPoseReferenceFrame(referenceFrame);

    ros::Rate loopRate(10);

    std::cout << "Moving to the pre-pick pose!" << std::endl;
    arm.setNamedTarget("1st");
    arm.move();

    geometry_msgs::Pose currentPose = arm.getCurrentPose(endEffector).pose;

    geometry_msgs::Pose goal_platform_pose;
    goal_platform_pose.orientation.x = 1.0;
    goal_platform_pose.orientation.y = 0.0;
    goal_platform_pose.orientation.z = 0.0;
    goal_platform_pose.orientation.w = 0.0;
    goal_platform_pose.position.x = 0.6;
    goal_platform_pose.position.y = -0.45;
    goal_platform_pose.position.z = 0.6787;

    geometry_msgs::Pose goal_platform_pose2;
    goal_platform_pose2.orientation.x = 0.92388;
    goal_platform_pose2.orientation.y = 0.3826;
    goal_platform_pose2.orientation.z = 0.0;
    goal_platform_pose2.orientation.w = 0.0;
    goal_platform_pose2.position.x = 0.6;
    goal_platform_pose2.position.y = -0.45;
    goal_platform_pose2.position.z = 0.3787;

    moveit_msgs::RobotTrajectory trajectory;
    robot_trajectory::RobotTrajectory rt(arm.getCurrentState()->getRobotModel(), "manipulator");
    //rt.setRobotTrajectoryMsg(*arm.getCurrentState(), trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    for (int i = 0; i < 12; i++)
    {
        std::cout << "sequence num. " << i << ": "
                  << sequence[i].pose.position.x << ", "
                  << sequence[i].pose.position.y << ", "
                  << sequence[i].pose.position.z << std::endl;


        std::vector<geometry_msgs::Pose> waypoints;
        geometry_msgs::Pose pose1;
        pose1.position.x = sequence[i].pose.position.x;
        pose1.position.y = sequence[i].pose.position.y;
        pose1.position.z = sequence[i].pose.position.z + 0.3;
        pose1.orientation.x = 1.0;
        pose1.orientation.y = 0.0;
        pose1.orientation.z = 0.0;
        pose1.orientation.w = 0.0;
        waypoints.push_back(pose1);

        geometry_msgs::Pose pose2;
        pose2.position.x = sequence[i].pose.position.x;
        pose2.position.y = sequence[i].pose.position.y;
        pose2.position.z = sequence[i].pose.position.z;
        pose2.orientation.x = 1.0;
        pose2.orientation.y = 0.0;
        pose2.orientation.z = 0.0;
        pose2.orientation.w = 0.0;

        waypoints.push_back(pose2);
        waypoints.push_back(pose1);
        waypoints.push_back(goal_platform_pose);
        waypoints.push_back(goal_platform_pose2);

        const double jump_threshold = 0.0;
        const double eef_step = 0.02;
        double fraction = 0.0;
        int maxtries = 100;
        int attempts = 0;

        while (fraction < 1.0 && attempts < maxtries)
        {
            fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            attempts++;
            if (attempts % 10 == 0)
                ROS_INFO("Still trying after %d attempts...", attempts);
        }

        if (fraction == 1.0)
        {
            ROS_INFO("Path computed successfully. Moving the arm.");
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            rt.setRobotTrajectoryMsg(*arm.getCurrentState(), trajectory);
            bool flag = iptp.computeTimeStamps(rt, 1.0, 1.0);
            if(flag){
                ROS_INFO("Smoothed Path computed successfully. Moving the arm.");
                plan.trajectory_ = trajectory;
                // execute
                arm.execute(plan);
            }
            else
            {
                ROS_INFO("Smoothed Path computed failed. Moving the arm.");
                plan.trajectory_ = trajectory;
                // execute
                arm.execute(plan);
            }

        }
        else
        {
            ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
        }
    }

    arm.setNamedTarget("1st");
    arm.move();

    ros::waitForShutdown();

    return 0;
}
