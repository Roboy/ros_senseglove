
#include <iostream>
#include <thread>
#include <chrono>         // std::chrono::seconds

#include "DeviceList.h"
#include "SenseGlove.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "sensor_msgs/JointState.h"
#include "roboy_middleware_msgs/MotorCommand.h"

#include <ros/ros.h>

using namespace std;

vector<float> bionic_cmd(vector<float> steps, vector<SGCore::Kinematics::Vect3D> initTipPositions, vector<SGCore::Kinematics::Vect3D> tipPositions) {
    vector<float> cmds;
    float cmd;
    for (int i=0;i<tipPositions.size()-1;i++) {
        auto cur = tipPositions[i];
        auto init = initTipPositions[i];
        auto dist = cur.DistTo(init);
        cmd = dist/steps[i];
        if (cmd > 700) {
            cmd = 700;
        } else if (cmd < 0) {
            cmd = 0;
        }
        cmds.push_back(cmd);
    }
    return cmds;
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "senseglove");
    ros::NodeHandle nh;
    auto vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 1);
    auto fingertip_pub = nh.advertise<geometry_msgs::PoseStamped>("/senseglove/fingertip", 1);
    auto joint_pub = nh.advertise<sensor_msgs::JointState>("/senseglove/joint_states",1);
    auto motorcmd_pub = nh.advertise<roboy_middleware_msgs::MotorCommand>("/roboy/middleware/MotorCommand",1);

    ros::Rate rate(10);
    auto gloves = SGCore::SG::SenseGlove::GetSenseGloves();
    auto leftGlove = gloves[0];
    SGCore::SG::SG_HandProfile handProfile = SGCore::SG::SG_HandProfile::Default(leftGlove.IsRight());

    auto fl = handProfile.handModel.fingerLengths;
    vector<vector<float>> my_fl;

    my_fl.push_back({40, 35, 30});
    my_fl.push_back({50,30,20});
    my_fl.push_back({60,38,23});
    my_fl.push_back({45,28,22});
    auto jp = handProfile.handModel.startJointPositions;
    ROS_INFO_STREAM_ONCE(handProfile.handModel.fingerLengths[0][0]);
    auto myHandModel = SGCore::Kinematics::BasicHandModel(false,my_fl,jp);
    SGCore::Kinematics::HandInterpolator handInterpolator = SGCore::Kinematics::HandInterpolator::Default(leftGlove.IsRight());
    auto myHandProfile = SGCore::SG::SG_HandProfile(false,myHandModel,handInterpolator,handProfile.fingerThimbleOffset);

    vector<float> steps;
    vector<SGCore::Kinematics::Vect3D> maxTipPositions, minTipPositions;

    bool min_calibrated = 0, max_calibrated =0;
    while(ros::ok()) {
        SGCore::SG::SG_GlovePose glovePose;
        SGCore::HandPose handPose;

        leftGlove.GetGlovePose(glovePose);
        leftGlove.GetHandPose(handProfile, SGCore::SG::SG_Solver::Interpolation, handPose);

        if (leftGlove.GetGlovePose(glovePose)) {
//            SGCore::SG::SG_GlovePose glovePose;

            if (!max_calibrated) {
                ROS_INFO_STREAM("Open your hand. Press ENTER to continue...");
                cin.get();
                leftGlove.GetGlovePose(glovePose);
                leftGlove.GetHandPose(handProfile, SGCore::SG::SG_Solver::Interpolation, handPose);
//                leftGlove.GetHandPose(handProfile, SGCore::SG::SG_Solver::Interpolation, handPose);
                handProfile = SGCore::SG::SG_HandProfile::Default(leftGlove.IsRight());
                maxTipPositions = glovePose.CalculateFingerTips(handProfile);
                for (auto t: maxTipPositions) ROS_INFO_STREAM(t.ToString());
                max_calibrated = true;
                continue;
            }

            if (!min_calibrated){
                ROS_INFO_STREAM("Close your hand. Presse ENTER to continue...");
                cin.get();
                leftGlove.GetGlovePose(glovePose);
                leftGlove.GetHandPose(handProfile, SGCore::SG::SG_Solver::Interpolation, handPose);
                handProfile = SGCore::SG::SG_HandProfile::Default(leftGlove.IsRight());
                auto minTipPositions = glovePose.CalculateFingerTips(handProfile);
                for (auto t: minTipPositions) ROS_INFO_STREAM(t.ToString());
                min_calibrated = true;
                for (int i=0; i<minTipPositions.size();i++) {
                    auto min = minTipPositions[i];
                    auto max = maxTipPositions[i];
                    auto dist = min.DistTo(max);
                    auto step = dist/800.0;
                    steps.push_back(step);
                }
                continue;
            }
            visualization_msgs::MarkerArray vis_msg;


            std::vector<SGCore::Kinematics::Vect3D> tipPositions = glovePose.CalculateFingerTips(handProfile);

            roboy_middleware_msgs::MotorCommand motorCommand;
            motorCommand.legacy = false;
            motorCommand.motor = {6,7,8,9};
            motorCommand.setpoint = bionic_cmd(steps,maxTipPositions,tipPositions);
            motorcmd_pub.publish(motorCommand);


            int id = 0;
            for (auto t: tipPositions) {
                geometry_msgs::PoseStamped msg1;
                msg1.header.frame_id = to_string(id);
                visualization_msgs::Marker msg;
                msg.header.frame_id = "hand_left";
                msg.id = 200+id;
                id++;
                msg.type = 2;
                msg.action = 0;
                geometry_msgs::Point point;
                point.z = - t.x/1000.0;
                point.y = t.y/1000;
                point.x = t.z/1000+0.03;
                msg.pose.position = point;
                msg1.pose.position = point;
                msg.scale.x = 0.01;
                msg.scale.y = 0.01;
                msg.scale.z = 0.01;

                msg.color.a = 100;
                vis_msg.markers.push_back(msg);
                fingertip_pub.publish(msg1);
            }


            int count = 2;
            visualization_msgs::Marker marker;
            marker.header.frame_id = "hand_left";
            for (int i=0; i<handPose.jointPositions.size();i++) {
                marker = visualization_msgs::Marker();
                std_msgs::ColorRGBA color;
                color.r = 1;
                color.g = 1;
                color.b = 1;
                color.a = 100;

                for (int j=1; j<handPose.jointPositions[i].size();j++) {
//                    if (i==1 && j==0) {
//                        ROS_INFO_STREAM(handPose.jointRotations[i][j].ToEuler().ToString());
//                    }
                    count ++;

                    marker.header.frame_id = "hand_left";
                    marker.id = 5*i+j;
                    marker.type = 4; //10;



//                    marker.mesh_resource = "package://asr_cyberglove_visualization/meshes/bone_0_"+to_string(count)+".stl";
//                    marker.mesh_use_embedded_materials = true;
                    marker.action = 0;
//                    marker.pose.position.x = handPose.jointPositions[i][j].x/1000;
//                    marker.pose.position.y = handPose.jointPositions[i][j].y/1000;
//                    marker.pose.position.z = handPose.jointPositions[i][j].z/1000;
//                    marker.pose.orientation.x = handPose.jointRotations[i][j].x;
//                    marker.pose.orientation.y = handPose.jointRotations[i][j].y;
//                    marker.pose.orientation.z = handPose.jointRotations[i][j].z;
//                    marker.pose.orientation.w = handPose.jointRotations[i][j].w;
                    geometry_msgs::Point p;
                    p.z = -handPose.jointPositions[i][j-1].x/1000;
                    p.y = handPose.jointPositions[i][j-1].y/1000;
                    p.x = handPose.jointPositions[i][j-1].z/1000+0.03;
                    marker.points.push_back(p);
                    marker.colors.push_back(color);
                    p.z = -handPose.jointPositions[i][j].x/1000;
                    p.y = handPose.jointPositions[i][j].y/1000;
                    p.x = handPose.jointPositions[i][j].z/1000+0.03;
                    marker.points.push_back(p);
                    marker.colors.push_back(color);

                    marker.scale.x = 0.01;
                    marker.scale.y = 0.01;
                    marker.scale.z = 0.01;

                    marker.color.a = 100;
//                    marker.color.r = 255;
//                    marker.color.g = 0;
//                    marker.color.b = 0;

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

            sensor_msgs::JointState jointState;
            auto ha = handPose.handAngles;
            for (auto finger: handPose.handAngles) {
                for (auto joint: finger) {
                    jointState.position.push_back(joint.y);
                }
            }
            joint_pub.publish(jointState);


        }

        ros::spinOnce();
        rate.sleep();
    }
}
