#ifndef CONTROLLER_H
#define CONTROLLER_H

#define SIMULATION

#include <iostream>
#include <fstream>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include "sensor_msgs/JointState.h"

#include "QuadProg++/Array.hh" 
#include "QuadProg++/QuadProg++.hh" 

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues> 
#include <Eigen/Dense>

#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Float64.h"

#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

using namespace std;
using namespace Eigen;

class Controller
{
    public:
        Controller(ros::NodeHandle &nh);
        virtual ~Controller();
        void init_ur5_kinematic_model();
        void constrained_bayesian_opt();
        void update_robot_state(VectorXd joint_value_list);
        void calculation_QP();
        void solve_QP();
        void evaluation_cal();
        void state_machine();
        void jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg);
        void actioncommandCallback(const std_msgs::Int16MultiArrayConstPtr& msg);
        void publish_command(double* weights_pointer);
        void initiate_robot_configuration();
        void set_desired_position(Vector3d p);
        void tuning_weights();


        //bo_bridge
        int bo_opt_sample = 5;
        int bo_opt_iteration = 20;
        ifstream in_file;
        ofstream out_file;
        double weights[3];
        double best_weights[3] = {1, 1, 1};
        float evaluation;
        const int dim_in_3 = 3;
        const int dim_in_2 = 2;
        char bo_weights_sig;

        VectorXd actual_joint_values;



        ros::NodeHandle nh_;
        ros::Subscriber joint_states_sub;
        ros::Subscriber action_command_sub;
        #ifdef SIMULATION
        ros::Publisher joint_cammand_pub;
        #else
        ros::Publisher joint_cammand_pub;
        #endif
        ros::Publisher evaluation_pub;
        robot_model_loader::RobotModelLoader robot_model_loader;
        robot_model::RobotModelPtr kinematic_model;
        robot_state::RobotStatePtr kinematic_state;
        robot_state::JointModelGroup* joint_model_group;

        std_msgs::Float64MultiArray joint_command_vel;
        std_msgs::Float64 evaluation_plot;
        double param_1,param_2,param_3;

        //task mark
        int start_sig;
        int process_order;
        int weights_dim;
        char bo_task;
        #ifdef SIMULATION
        float pid_p = 0.9;
        float pid_d = 0.02;
        #else
        float pid_p = 0.1;
        float pid_d = 0.02;
        #endif
        float joint_angle_target[6]={-2.9891236464129847, -1.5848787466632288, -2.775407854710714, -0.01725799242128545, -1.6374219099627894, 1.4961683750152588};
        //-2.808974806462423, -1.578759495412008, -3.093093220387594, 0.11541104316711426, -1.6217687765704554, 3.091593027114868
        //-2.9891236464129847, -1.5848787466632288, -2.775407854710714, -0.01725799242128545, -1.6374219099627894, 1.4961683750152588
        int bo_frequency;
        //publisher
        //joint command

        //subscriber
        //joint_states
        //start_command


        //robot states
        
        VectorXd predicted_joint_values;
        float joint_velocity_actual[6];
        std::vector<double> joint_values;
        Vector3d reference_point_position;
        MatrixXd jaco;
        Vector3d end_effector_position;//end effector position and orientation
        Vector3d insert_position;
        Vector3d desired_position;
        Vector3d desired_position_dt;//relative to world frame
        Vector3d desired_orientation;
        Vector3d desired_orientation_dt;
        Vector3d end_effector_y_axis;
        Matrix3d y_axis_hat;
        Vector3d desired_orientation_x;
        Vector3d end_effector_x_axis;
        Matrix3d x_axis_hat;
        MatrixXd rotation_mtr;
        MatrixXd jacobian;//jacobian
        MatrixXd JTxJ;
        MatrixXd jacob_p;
        MatrixXd jacob_w;

        //other object info ,door/d2w transform/obstacle
        Isometry3d world2door;
        AngleAxisd rotation_vector;
        Vector3d static_normal_door;
        double door_radius, door_omega, theta;
        double traj_time, door_Time, door_start_time;
        Vector3d obstable_cylinder;

        VectorXd insert_joint_values;

        //solve QP 
        double zero = 0.0;
        quadprogpp::Matrix<double>G; 
        quadprogpp::Vector<double>g;
        quadprogpp::Matrix<double>CE; 
        quadprogpp::Vector<double>ce; 
        quadprogpp::Matrix<double>CE_door; 
        quadprogpp::Vector<double>ce_door; 
        quadprogpp::Matrix<double>CI;
        quadprogpp::Vector<double>ci;
        quadprogpp::Vector<double>u;
        quadprogpp::Vector<double>u1;
        quadprogpp::Vector<double>u2;
        quadprogpp::Vector<double>u3;
        quadprogpp::Vector<double>u4;
        // quadprogpp::Vector<double>last_u;
        quadprogpp::Vector<double>a;

        
        //object function 1 is tracking, a pt/trajectory;
        //object function 2 is keeping the y axis of wrist3 normal to a surface
        //object function 3 is avoiding sigularities
        //object function 4 is avoiding collision with obstable
        VectorXd f1,f5; 
        double f2,f3,f4,f6;
        MatrixXd f1_dq_T,f2_dq_T,f3_dq_T,f4_dq_T,f5_dq_T,f6_dq_T;
        VectorXd f1_dt,f5_dt;
        double f2_dt,f3_dt,f4_dt,f6_dt;
        VectorXd b1,b5;
        double b2,b3,b4,b6;
        float k;
        double fitness_1=0.5,fitness_2=1,fitness_3=1,fitness_u=0.37;
        float dt;
        float dq = 0.01;
        int Time_0 = 15;

        double small_max_vel_step = 2e-2;
        double last_u[6]={0,0,0,0,0,0};
    private:
};

#endif
