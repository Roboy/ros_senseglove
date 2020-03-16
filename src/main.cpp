
#include <iostream>
#include <thread>
#include <chrono>         // std::chrono::seconds

#include "DeviceList.h"
#include "SenseGlove.h"

#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "sensor_msgs/JointState.h"

#include <ros/ros.h>
using namespace std;
int main(int argc, char** argv) {
    ros::init(argc, argv, "senseglove");
    ros::NodeHandle nh;
    auto vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);
    auto fingertip_pub = nh.advertise<geometry_msgs::PoseStamped>("/senseglove/fingertip", 1);
    auto joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states",1);
    ros::Rate rate(10);
    auto gloves = SGCore::SG::SenseGlove::GetSenseGloves();
    auto leftGlove = gloves[1];


    while(ros::ok()) {

        SGCore::SG::SG_GlovePose glovePose;
        if (leftGlove.GetGlovePose(glovePose)) {
            visualization_msgs::MarkerArray vis_msg;

            SGCore::SG::SG_HandProfile handProfile = SGCore::SG::SG_HandProfile::Default(leftGlove.IsRight());
            SGCore::Kinematics::HandInterpolator handInterpolator = SGCore::Kinematics::HandInterpolator::Default(leftGlove.IsRight());
            SGCore::HandPose handPose;
            leftGlove.GetHandPose(handProfile, SGCore::SG::SG_Solver::Interpolation, handPose);

            int count = 2;
            for (int i=0; i<handPose.jointPositions.size();i++) {
                for (int j=0; j<handPose.jointPositions[i].size();j++) {
//                    if (i==1 && j==0) {
//                        ROS_INFO_STREAM(handPose.jointRotations[i][j].ToEuler().ToString());
//                    }
                    count ++;
                    visualization_msgs::Marker marker;
                    marker.header.frame_id = "world";
                    marker.id = 5*i+j;
                    marker.type = 3; //10;
//                    marker.mesh_resource = "package://asr_cyberglove_visualization/meshes/bone_0_"+to_string(count)+".stl";
//                    marker.mesh_use_embedded_materials = true;
                    marker.action = 0;
                    marker.pose.position.x = handPose.jointPositions[i][j].x/1000;
                    marker.pose.position.y = handPose.jointPositions[i][j].y/1000;
                    marker.pose.position.z = handPose.jointPositions[i][j].z/1000;
                    marker.pose.orientation.x = handPose.jointRotations[i][j].x;
                    marker.pose.orientation.y = handPose.jointRotations[i][j].y;
                    marker.pose.orientation.z = handPose.jointRotations[i][j].z;
                    marker.pose.orientation.w = handPose.jointRotations[i][j].w;
                    marker.scale.x = 0.01;
                    marker.scale.y = 0.01;
                    marker.scale.z = 0.01;
                    marker.color.a = 100;
                    vis_msg.markers.push_back(marker);
                }
            }

//            std::vector<SGCore::Kinematics::Vect3D> tipPositions = glovePose.CalculateFingerTips(handProfile);
//            for (int f = 0; f < tipPositions.size(); f++) {
//                visualization_msgs::Marker marker;
//                marker.id = f;
//                marker.type = 2;
//                marker.action = 0;
//                marker.header.frame_id = "world";
//                marker.pose.position.x = tipPositions[f].x/1000.0;
//                marker.pose.position.y = tipPositions[f].y/1000.0;
//                marker.pose.position.z = tipPositions[f].z/1000.0;
//                marker.scale.x = 0.08;
//                marker.scale.y = 0.08;
//                marker.scale.z = 0.08;
//                marker.color.a = 100;
//                vis_msg.markers.push_back(marker);
//            }

            vis_pub.publish(vis_msg);
        }

        ros::spinOnce();
        rate.sleep();
    }
}
