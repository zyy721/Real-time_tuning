
#include "online_tuning/controller.h"
#include <sstream>
#include <stdio.h> 
#include <time.h>
#include <random>
#include <tf/transform_listener.h>

#define PRINT_TRAJECTORY
// #define PRINT_FITNESS
// #define OUTPUT_PLOT_DATA
// #define USE_VISION

double best_weights[4]={1,0,0,0};
double temp_weghts[4];
double last_best_weights[4]={0,0,0,0};
void bestweightsCallback(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    for(int i=0;i<3;i++){
        best_weights[i] = msg->data[i];
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "bayesian_opt");
    ros::NodeHandle nh("~");
    
    Controller controller(nh);
    ros::Rate loop_rate(100);
    ros::Publisher best_weights_pub = nh.advertise<std_msgs::Float64MultiArray>("best_weights", 20);
    std_msgs::Float64MultiArray pub_best_weights;
    std::vector<double> none = {1,1,1}; 
    pub_best_weights.data = none;
    tf::TransformListener listener;

    double factor = 0.2;

    #ifdef PRINT_TRAJECTORY
        ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("trajectory",1, true);
        nav_msgs::Path path;
        geometry_msgs::PoseStamped this_pose_stamped;

        ros::Publisher path_pub_circle = nh.advertise<nav_msgs::Path>("circle",1, true);
        nav_msgs::Path path_circle;
        geometry_msgs::PoseStamped this_pose_stamped_circle;

        ros::Publisher path_pub_xyz = nh.advertise<std_msgs::Float64MultiArray>("path_pub_xyz",10, true);
        std_msgs::Float64MultiArray xyz;
        xyz.data = none;
    #endif
    
    #ifdef PRINT_FITNESS
        ros::Publisher fitness_pub = nh.advertise<std_msgs::Float64MultiArray>("fitness_pub",10, true);
        std_msgs::Float64MultiArray fitness;//f1 f2 f4
        std::vector<double> none2 = {1,2,4}; 
        fitness.data = none2;
    #endif

    #ifdef OUTPUT_PLOT_DATA
        ofstream out_file1,out_file2,out_file3;
        out_file1.open("/home/neousys/lee/robot_data.txt");
        out_file1 << "w1 " << "w2 " << "w4 "  << "t_e1 " << "t_e2 " << "t_e4 " << "x " << "y " << "z " << endl;
        out_file2.open("/home/neousys/lee/robot_joint_data.txt");
        out_file2 << "p1 " << "p2 " << "p3 "  << "p4 " << "p5 " << "p6 " << "v1 " << "v2 " << "v3 " << "v4 " << "v5 " << "v6 " << endl;
        out_file3.open("/home/neousys/lee/vel_command.txt");
        out_file3 << "vc1 " << "vc2 " << "vc3 "  << "vc4 " << "vc5 " << "vc6 " << endl;
    #endif

    std::default_random_engine generator;
    std::normal_distribution<double> dist(0.0, 0.0005);
    std::normal_distribution<double> dist2(0.0, 0.0001);
    
    while(!controller.start_sig)
    {
        controller.initiate_robot_configuration();
        loop_rate.sleep();
    }
    controller.start_sig = 0;
    double time_start;



    time_start = ros::Time::now().toSec();
    while(ros::ok())
    {
        ros::spinOnce();//update joint values
        Vector3d p;
        p << controller.obstable_cylinder(0)+0.1*sin(ros::Time::now().toSec()*10),
             controller.obstable_cylinder(1) +0.1*cos(ros::Time::now().toSec()*10),
             controller.obstable_cylinder(2);

        controller.update_robot_state(controller.actual_joint_values);
        controller.calculation_QP();
        controller.solve_QP();

        controller.tuning_weights();
        for(int i=0;i<3;i++){
            best_weights[i] = controller.a[i];
            pub_best_weights.data[i] = best_weights[i];
        }

        best_weights_pub.publish(pub_best_weights);
        controller.publish_command(best_weights);


        #ifdef USE_VISION
            tf::StampedTransform transform;
            try{
                listener.lookupTransform("/base_link", "/ar_marker_3",  
                                        ros::Time(0), transform);
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }

            Vector3d dynamic_p;
            Vector3d default_p;
            Vector3d dd_p;
            dynamic_p << transform.getOrigin().x(),
                        transform.getOrigin().y()+0.035,
                        transform.getOrigin().z();
            default_p << 0.6,
                        -0.50,
                        0.118879;
            dd_p = dynamic_p-default_p;
            if(dd_p.norm()>0.15){
                controller.set_desired_position(default_p);
                cout << "error: detection out of bound" << endl;
            }
            else{
                controller.set_desired_position(dynamic_p);
            }
        #endif


        #ifdef PRINT_TRAJECTORY
            path.header.stamp=ros::Time::now();
            path.header.frame_id="base_link";
            geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(0);
            this_pose_stamped.pose.orientation.x = goal_quat.x;
            this_pose_stamped.pose.orientation.y = goal_quat.y;
            this_pose_stamped.pose.orientation.z = goal_quat.z;
            this_pose_stamped.pose.orientation.w = goal_quat.w;
            this_pose_stamped.header.stamp=ros::Time::now();
            this_pose_stamped.pose.position.x = p(0);
            this_pose_stamped.pose.position.y = p(1);
            this_pose_stamped.pose.position.z = p(2);
            this_pose_stamped.header.frame_id="base_link";
            path.poses.push_back(this_pose_stamped);
            path_pub.publish(path);

            path_circle.header.stamp=ros::Time::now();
            path_circle.header.frame_id="base_link";
            this_pose_stamped_circle.pose.orientation.x = goal_quat.x;
            this_pose_stamped_circle.pose.orientation.y = goal_quat.y;
            this_pose_stamped_circle.pose.orientation.z = goal_quat.z;
            this_pose_stamped_circle.pose.orientation.w = goal_quat.w;
            this_pose_stamped_circle.header.stamp=ros::Time::now();
            this_pose_stamped_circle.pose.position.x = controller.end_effector_position(0);
            this_pose_stamped_circle.pose.position.y = controller.end_effector_position(1);
            this_pose_stamped_circle.pose.position.z = p(2);
            this_pose_stamped_circle.header.frame_id="base_link";
            path_circle.poses.push_back(this_pose_stamped_circle);
            path_pub_circle.publish(path_circle);

            for(int i=0;i<3;i++){
                xyz.data[i] = controller.end_effector_position(i);
            }
            path_pub_xyz.publish(xyz);
        #endif

        #ifdef PRINT_FITNESS
            fitness.data[0] = controller.f1(0);
            fitness.data[1] = controller.f5(0);
            fitness.data[2] = -(controller.b2 - controller.f2);
            fitness_pub.publish(fitness);
        #endif

        #ifdef OUTPUT_PLOT_DATA
            // out_file1 << "w1" << "w2" << "w4"  << "t_e1" << "t_e2" << "t_e4" << "x" << "y" << "z" << endl;
            out_file1 << best_weights[0] << " " << best_weights[1] << " " << best_weights[2]
                    << " " << controller.f1(0) << " " << -(controller.b2 - controller.f2) << " " << controller.f6
                    << " " << controller.end_effector_position(0) << " " << controller.end_effector_position(1) << " " << controller.end_effector_position(2) << endl;
            // out_file2 << "p1" << "p2" << "p3"  << "p4" << "p5" << "p6" << "v1" << "v2" << "v3" << "v4" << "v5" << "v6" << endl;
            out_file2 << controller.actual_joint_values(0) << " " << controller.actual_joint_values(1) << " " << controller.actual_joint_values(2)
                    << " " << controller.actual_joint_values(3) << " " << controller.actual_joint_values(4) << " " << controller.actual_joint_values(5)
                    << " " << controller.joint_velocity_actual[0] << " " << controller.joint_velocity_actual[1] << " " << controller.joint_velocity_actual[2]
                    << " " << controller.joint_velocity_actual[3] << " " << controller.joint_velocity_actual[4] << " " << controller.joint_velocity_actual[5] << endl;

            out_file3 << controller.joint_command_vel.data[0] << " " << controller.joint_command_vel.data[1] << " " << controller.joint_command_vel.data[2]
                    << " " << controller.joint_command_vel.data[3] << " " << controller.joint_command_vel.data[4] << " " << controller.joint_command_vel.data[5] << endl;
        #endif

        loop_rate.sleep();

        if( sqrt(controller.f1(0))<0.02 ){
            cout << "distance: " << sqrt(controller.f1(0)) << endl;
            cout << "f2: " << controller.f2 << endl;
            cout << "cost time: " << ros::Time::now().toSec() - time_start  << endl;
            #ifdef OUTPUT_PLOT_DATA
                out_file1.close();
                out_file2.close();
                out_file3.close();
                cout << "files closed" << endl;
            #endif
            break;
        }
    }

    while(ros::ok()){
        for (int i = 0; i < 6; i++) {
                controller.joint_command_vel.data[i] = 0; 
        }
        controller.joint_cammand_pub.publish(controller.joint_command_vel);
        loop_rate.sleep();
    }
    // controller.initiate_robot_configuration();
    return 0;
}
