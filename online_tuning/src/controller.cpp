#include "online_tuning/controller.h"

Controller::Controller(ros::NodeHandle &nh):nh_(nh),robot_model_loader("robot_description"), reference_point_position(0.0,0.0,0.089),
                             jacobian(6,6), JTxJ(6,6), rotation_mtr(3,3), jacob_p(3,6), jacob_w(3,6),
                             G(0.0,6,6), g(0.0,6), CE(0.0,6, 1), ce(0.0,1), CI(0.0,6, 2), ci(0.0,1),
                             u(0.0,6),u1(0.0,6),u2(0.0,6),u3(0.0,6),u4(0.0,6),a(0.0,3),
                             f1(1), b1(1),f2_dq_T(1,6),f3_dq_T(1,6),f4_dq_T(1,6),
                             rotation_vector(M_PI/2.0, Vector3d(0, 0, 1)),
                             actual_joint_values(6),insert_joint_values(6)

{
    joint_states_sub = nh_.subscribe("/joint_states" , 1, &Controller::jointstatesCallback, this);
    action_command_sub = nh_.subscribe("/action_command" , 1, &Controller::actioncommandCallback, this);
    #ifdef SIMULATION
    joint_cammand_pub = nh_.advertise<std_msgs::Float64MultiArray>("/arm_controller/command", 20);
    #else
    joint_cammand_pub = nh_.advertise<std_msgs::Float64MultiArray>("/joint_group_vel_controller/command", 20);
    #endif
    evaluation_pub = nh_.advertise<std_msgs::Float64>("/evaluation_pub", 100);

    std::vector<double> none = {1,2,3,4,5,6}; 
    joint_command_vel.data = none;
    nh_.param("x", param_1, 0.01);
    nh_.param("y", param_2, 0.185);
    nh_.param("z", param_3, 0.1);
/*    cout << reference_point_position << endl;
    cout << jacobian.size() << endl;*/

    process_order = 1; //task1 2 3  weights_dim = 2;bo_task = '2'
    k = 0.8;
    b2 = -cos(0.0/180*M_PI); // normal to a surface with 5 degrees error max
    b3 = 0.0001;
    b4 = -cos(0.0/180*M_PI);
    b6=0.0;
    //orientation task
    door_radius = 0.3;
    door_Time = 10;//time interval for openning the door
    door_omega = 0.5*M_PI/door_Time;
    
    world2door = Eigen::Isometry3d::Identity();
    world2door.rotate(rotation_vector);
    world2door.pretranslate(Eigen::Vector3d(0.6, 0.1, 0.2));

    static_normal_door << 0,
                          -1,
                          0;
    //first desired point
    desired_position << 0.6,
                        -0.50,
                        0.418879;
                        // 0.35;

    insert_position << 0.6,
                       0.28,
                       0.244;

    insert_joint_values << -2.8,
                           -2.0,
                           -1.156,
                            0.0,
                            0.0,
                            0.0;

    desired_orientation_x << 0.0,
                             0.0,
                             1.0;
    //obstacle
    obstable_cylinder << 0.3,
                         -0.25,
                         0.19;
    //QP_solver
    
    for(int i=0;i<6;i++) {
        G[i][i] = 2*0.001;   //G=2*Q  Q=diag(0.5)
    }
    bo_frequency = 10;
    dt = float(1.0/bo_frequency)*1.5;



    //kinematic model
    kinematic_model = robot_model_loader.getModel();
    robot_state::RobotStatePtr ks(new robot_state::RobotState(kinematic_model));
    kinematic_state = ks;
    kinematic_state->setToDefaultValues();
    joint_model_group = kinematic_model->getJointModelGroup("sucker");
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

    start_sig = 0;
}

Controller::~Controller(void)
{
    cout << "destroy controller" << endl;
}

void Controller::update_robot_state(VectorXd temp_joint_value)
{
    for(int i=0;i<6;i++) {
        joint_values[i] = temp_joint_value(i);
    }
    kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
    const Eigen::Affine3d& end_effector_state = kinematic_state->getGlobalLinkTransform("sucker_link");
    

    rotation_mtr = end_effector_state.rotation();
    end_effector_y_axis = rotation_mtr.block(0,2,3,1);
    Vector3d x = end_effector_y_axis;
    y_axis_hat << 0, -x(2), x(1),
                    x(2), 0, -x(0),
                    -x(1), x(0), 0;
    end_effector_x_axis = rotation_mtr.block(0,0,3,1);
    x = end_effector_x_axis;
    x_axis_hat << 0, -x(2), x(1),
                  x(2), 0, -x(0),
                 -x(1), x(0), 0;

    end_effector_position = end_effector_state.translation();
    kinematic_state->getJacobian(joint_model_group,
                        kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                        reference_point_position, jaco);//update jacobian from moveit group
    jacobian = jaco.block(0,0,6,6);
    JTxJ = jacobian.transpose() * jacobian;
    jacob_p = jacobian.block(0,0,3,6);
    jacob_w = jacobian.block(3,0,3,6);
}

void Controller::state_machine()
{
}

void Controller::calculation_QP()
{
    f1.resize(1);
    f1_dq_T.resize(1,6);
    f1_dt.resize(1);
    b1.resize(1);
    b1 << 0;
    Vector3d P_Pobj;
    P_Pobj = end_effector_position - desired_position;
    desired_orientation = static_normal_door;

    f1(0) = P_Pobj.transpose()*P_Pobj;
    f1_dq_T = 2*P_Pobj.transpose()*jacob_p;
    f1_dt(0) = 0.0;

    f5.resize(1);
    f5_dq_T.resize(1,6);
    f5_dt.resize(1);
    b5.resize(1);
    b5 << -0.1*0.1;
    Vector3d P_Pobj2;
    P_Pobj2 = end_effector_position - obstable_cylinder;
    f5(0) = -P_Pobj2.transpose()*P_Pobj2;
    f5_dq_T = -2*P_Pobj2.transpose()*jacob_p;
    f5_dt(0) = 0.0;

    VectorXd J_Jobj;
    J_Jobj = actual_joint_values - insert_joint_values;
    J_Jobj(3)=0;J_Jobj(4)=0;J_Jobj(5)=0;//disable wrist 123
    f6 = J_Jobj.transpose()*J_Jobj;
    f6_dq_T = 2*J_Jobj;
    f6_dt = 0.0;

    f2 = -end_effector_y_axis.transpose()*desired_orientation;
    f2_dq_T = -desired_orientation.transpose()*(-y_axis_hat*jacob_w);
    f2_dt = 0.0;
    f3 = -0.5*JTxJ.determinant();
    for(int i=0;i<6;i++) { //dj/dq
        float f3_new = 0;
        MatrixXd jacobian;
        MatrixXd jaco;
        MatrixXd JTxJ;
        VectorXd joint_values_temp = actual_joint_values;
        joint_values_temp[i] += dq;

        kinematic_state->setJointGroupPositions(joint_model_group, joint_values_temp);
        kinematic_state->getJacobian(joint_model_group,
                        kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                        reference_point_position, jaco);//update jacobian from moveit group
        jacobian = jaco.block(0,0,6,6);
        JTxJ = jacobian.transpose() * jacobian;
        f3_new = -0.5*JTxJ.determinant();
        f3_dq_T(i) = (f3_new - f3)/dq;
    }// df_3dq = 6x1
    f3_dt =0.0;
    // f4
    f4 = -end_effector_x_axis.transpose()*desired_orientation_x;
    f4_dq_T = -desired_orientation_x.transpose()*(-x_axis_hat*jacob_w);
    f4_dt = 0.0;

}

void Controller::solve_QP(void)
{

    // CE.resize(6,0);
    // ce.resize(0);
    // CI.resize(6,0); 
    // ci.resize(0);

    for(int i=0;i<6;i++) {
        G[i][i] = 2*0.5;   //G=2*Q  Q=diag(0.5)
    }
    // for(int i=0;i<6;i++) {
    //     g[i] = f1_dq_T(i);
    // }
    // solve_quadprog(G, g, CE, ce, CI, ci, u1); 

    for(int i=0;i<6;i++) {
        u1[i] = -f1_dq_T(i)/G[i][i];    // df/dx=0 -> x = -cT*Q^(-1)/2
    }

    //task2 elbow goto point
    // for(int i=0;i<6;i++) {
    //     CE[i][0] = f5_dq_T(i);
    //     CI[i][0] = 0;
    // }
    // ce[0] = k*(f5(0)-b5(0))+f5_dt(0);
    // ci[0] = 0;
    // solve_quadprog(G, g, CE, ce, CI, ci, u2); 



    // task3 orientation task
    // CE.resize(6,1);
    // ce.resize(1);
    // CI.resize(6,1); 
    // ci.resize(1);
    // for(int i=0;i<6;i++) {
    //     CE[i][0] = f2_dq_T(i);
    //     CI[i][0] = 0;
    // }
    // ce[0] = k*(f2-b2)+f2_dt;
    // ci[0] = 0;
    // solve_quadprog(G, g, CE, ce, CI, ci, u3); 

    for(int i=0;i<6;i++) {
        u3[i] = -f2_dq_T(i)/G[i][i];    // df/dx=0 -> x = -cT*Q^(-1)/2
    }


    //task4 : first 3 joint
    // CE.resize(6,1);
    // ce.resize(1);
    // CI.resize(6,1); 
    // ci.resize(1);
    // for(int i=0;i<6;i++) {
    //     CE[i][0] = f6_dq_T(i);
    //     CI[i][0] = 0;
    //     g[i] = 0;
    // }
    // ce[0] = k*(f6-b6)+f6_dt;
    // ci[0] = 0;
    // solve_quadprog(G, g, CE, ce, CI, ci, u4); 
    for(int i=0;i<6;i++) {
        u4[i] = -f6_dq_T(i)/G[i][i];    // df/dx=0 -> x = -cT*Q^(-1)/2
    }
}

void Controller::tuning_weights(void)
{
	// double f1_scale = 1.0;
	double f24_scale = 0.2;//0.5;
    double dt = 0.01;
    double constant;
    double Px,Py,Pz,Xo1,Xo2,Xo3,Xosb1,Xosb2,Xosb3;
    double f1_test=0.0;
    double f24_test=0.0;

    CE.resize(3,0);
    ce.resize(0);
    CI.resize(3,7);
    ci.resize(7);

    CI = quadprogpp::Matrix<double>(0.0,3,7);
    quadprogpp::Matrix<double>GG(0.0,3,3);
    quadprogpp::Vector<double>gg(0.0,3);
    // cout << "f1_test: " << f1_test << endl;
    // cout << "f24_test: " << f24_test << endl;
    VectorXd t_u1(6),t_u3(6),t_u4(6),t_CE1(1),t_CE2(1),t_CE3(1),t_gg1(1),t_gg2(1),t_gg3(1);
    for(int i=0;i<6;i++){
    	t_u1(i) = u1[i];
    	t_u3(i) = u3[i];
    	t_u4(i) = u4[i];
	}
    

    double temp_factor;
    temp_factor = 0.39;//0.3;
	t_gg1 = (f1_dq_T + temp_factor*f2_dq_T)*t_u1;
	t_gg2 = (f1_dq_T + temp_factor*f2_dq_T)*t_u3;
	t_gg3 = (f1_dq_T + temp_factor*f2_dq_T)*t_u4;
    gg[0] = t_gg1(0);
    gg[1] = t_gg2(0);
    gg[2] = t_gg3(0);
	{
		t_CE1 = -f5_dq_T*t_u1;
	    t_CE2 = -f5_dq_T*t_u3;
	    t_CE3 = -f5_dq_T*t_u4;
	    CI[0][0] = t_CE1(0);
	    CI[1][0] = t_CE2(0);
	    CI[2][0] = t_CE3(0);
	    ci[0] = 0.1*k*(b5(0)-f5(0))-f5_dt(0);;//cinstant item
	}

    for(int i=0;i<3;i++) {
	    GG[i][i] = 2* 0.5*fabs(gg[i]) + 1e-10; //G=2*Q   
	    // gg[i] = 0.0;
	    CI[i][i+1] = 1.0;
	    ci[i+1] = 0.0;
	    CI[i][i+1+3] = -1.0;
	    ci[i+1+3] = 1.0;
	}
    solve_quadprog(GG, gg, CE, ce, CI, ci, a); 

}

void Controller::constrained_bayesian_opt(void)
{
    switch(1)
    {
        case 1: {weights_dim = 3;bo_task = '1';}break;
        case 2: {weights_dim = 2;bo_task = '2';}break;
        case 3: {weights_dim = 2;bo_task = '2';}break;
        case 4: {weights_dim = 2;bo_task = '2';}break;
        default : cout << "weights dimension error"<< endl;break;
    }
    // if(process_order==2||process_order==4) break;
    // cout << "BO start signal,launch task1" << endl;
    out_file.open("/home/desmond/limbo/task_order.txt", ios::trunc);
    out_file << "1" ;//BO start signal,launch task1
    out_file.close();
    out_file.open("/home/desmond/limbo/bo_start_signal.txt", ios::trunc);
    out_file << "1" ;//BO start signal,launch task1
    out_file.close();

    for(int j=0;j<(bo_opt_sample+bo_opt_iteration);j++) {

        while(bo_weights_sig != '1') {  //waiting for bo_weights_sig
            in_file.open("/home/desmond/limbo/bo_weights_sig.txt");
            in_file >> bo_weights_sig;
            in_file.close();
        }
        bo_weights_sig = '0';
        out_file.open("/home/desmond/limbo/bo_weights_sig.txt", ios::trunc);
        out_file.close();

        in_file.open("/home/desmond/limbo/weights.txt"); 
        for(int i=0;i<weights_dim;i++){
            in_file >> weights[i] ;
        }
        in_file.close();
        // cout << "wights: ";
        // for(int i=0;i<weights_dim;i++){
        //     cout << weights[i] << "     ";
        // }
        // cout << endl;

        // solve_QP(weights);
        u = weights[0]*u1 + weights[1]*u3 + weights[2]*u4;

        //update_robot_state(VectorXd temp_joint_value);
        evaluation_cal();

        out_file.open("/home/desmond/limbo/evaluation.txt", ios::trunc);
        out_file << evaluation ;//third line weights wrriten completion signal
        out_file.close();

        out_file.open("/home/desmond/limbo/evaluation_cal_sig.txt", ios::trunc);
        out_file << "1" ;//third line weights wrriten completion signal
        out_file.close();

        // cout << "evaluation:  " << -evaluation << endl;
        // evaluation_plot.data = -evaluation;
        // evaluation_pub.publish(evaluation_plot);

        // cout << "u: " << u << "u1: " << u1 << "u2: " << u2 << "u4: " << u4 << endl;

    }


    while(bo_weights_sig != '1') {  //waiting for bo_weights_sig
        in_file.open("/home/desmond/limbo/bo_weights_sig.txt");
        in_file >> bo_weights_sig;
        in_file.close();
    }
    bo_weights_sig = '0';
    out_file.open("/home/desmond/limbo/bo_weights_sig.txt", ios::trunc);
    out_file.close();

    in_file.open("/home/desmond/limbo/weights.txt"); 
    for(int i=0;i<weights_dim;i++){
        in_file >> best_weights[i] ;
    }
    in_file.close();
    // evaluation_plot.data = -10;
    // evaluation_pub.publish(evaluation_plot);
    cout << "evaluation:  " << -evaluation << endl;
}

void Controller::evaluation_cal()
{
    VectorXd joint_vel(6);
    for(int i=0;i<6;i++){
        joint_vel(i) = u[i];//weights[0]*u1[i] + weights[1]*u2[i] + weights[2]*u3[i]; 
    }
    predicted_joint_values = actual_joint_values + dt*joint_vel;
    // cout << "joint_vel: " << joint_vel << endl;
    // cout << "actual_joint_values: " << actual_joint_values << endl;
    // cout << "predicted_joint_values: " << predicted_joint_values << endl;
    update_robot_state(predicted_joint_values);
    double f1_lc,f2_lc,f4_lc;
    Vector3d P_Pobj;
    Vector3d P_Pobst;
    P_Pobj = end_effector_position - desired_position;
    P_Pobst = end_effector_position - obstable_cylinder;
    P_Pobst(2)=0.0;//infinite high
    f1_lc = pow(P_Pobj.norm(), 2);
    f2_lc = -end_effector_y_axis.transpose()*desired_orientation;
    f4_lc = -end_effector_x_axis.transpose()*desired_orientation_x;
    // f2_lc = (f2_lc<b2) ? b2 : f2_lc;
    // f4_lc = (f4_lc<b4) ? b4 : f4_lc;
    // cout << "f2_lc: " << f2_lc << "  f4_lc:" << f4_lc << endl;
    // cout << "dis:" << P_Pobst.norm() << endl;
    if(P_Pobst.norm() < 0.10){
        evaluation = 100; //penalty        
        cout << "warning" << endl;
    }
    else
        evaluation = 11.0*fabs(0-f1_lc)/fitness_1 + 
                     2.0*(fabs(b2-f2_lc)/fitness_2 + fabs(b4-f4_lc)/fitness_2) + 
                     0.1*double(joint_vel.transpose()*joint_vel)/fitness_u;
    evaluation = -evaluation;//minimum
    // cout << "fit1:" << fabs(0-f1_lc)/fitness_1 << endl;
}

double get_actual_evaluation()
{
    double actual_eval;
    
    return actual_eval;
}

void Controller::jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
    for(int i=0;i<6;i++) {
        actual_joint_values(i) = msg->position[i];
        joint_velocity_actual[i] = msg->velocity[i];
    }
    actual_joint_values(0) = msg->position[2];
    actual_joint_values(2) = msg->position[0];
    joint_velocity_actual[0] = msg->velocity[2];
    joint_velocity_actual[2] = msg->velocity[0];

}

void Controller::actioncommandCallback(const std_msgs::Int16MultiArrayConstPtr& msg)
{
    start_sig = 1;
}

void Controller::publish_command(double* weights_pointer)
{
    for (int i = 0; i < 6; i++){
        joint_command_vel.data[i] = weights_pointer[0]*u1[i] + weights_pointer[1]*u3[i] + 
                                    weights_pointer[2]*u4[i] ;//+ weights_pointer[3]*u4[i]; 

    joint_cammand_pub.publish(joint_command_vel);
}

void Controller::initiate_robot_configuration()
{
    ros::spinOnce();
    for (int i = 0; i < 6; i++) {
        u[i] = (joint_angle_target[i] - actual_joint_values(i)) * pid_p +
                                  - joint_velocity_actual[i] * pid_d; 


        if(fabs(joint_angle_target[i] - actual_joint_values(i)) < 0.05)
            joint_command_vel.data[i] = 0;
        else
            joint_command_vel.data[i] = u[i]; 

     }
     joint_cammand_pub.publish(joint_command_vel);
}

void Controller::set_desired_position(Vector3d p)
{
    desired_position = p;
}