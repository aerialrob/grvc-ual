//----------------------------------------------------------------------------------------------------------------------
// GRVC UAL
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2016 GRVC University of Seville
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------


#include <ual_backend_dji_ros/ual_backend_dji_ros.h>
#include <Eigen/Eigen>
#include <ros/ros.h>


geometry_msgs::Pose::_orientation_type q;
// geometry_msgs::QuaternionStamped current_attitude;
double roll;
double pitch;
double yaw;

double altitude_offset;
double alt_1;
int alt_counter = 0;
float target_z;
bool going_up;


namespace grvc { namespace ual {

BackendDjiRos::BackendDjiRos()
    : Backend()
{
    // Parse arguments
    ros::NodeHandle pnh("~");
    pnh.param<int>("uav_id", robot_id_, 1);
    pnh.param<std::string>("pose_frame_id", pose_frame_id_, "");
    float position_th_param, orientation_th_param;
    pnh.param<float>("position_th", position_th_param, 0.33);
    pnh.param<float>("orientation_th", orientation_th_param, 0.6);
    position_th_ = position_th_param*position_th_param;
    orientation_th_ = 0.5*(1 - cos(orientation_th_param));

    pnh.param<bool>("simulation", simulation, false);
    // ROS_INFO_STREAM("BackendDjiRos simulation  " << simulation);
    ROS_INFO_STREAM_COND(simulation == 0, "simulation = False");
    ROS_INFO_STREAM_COND(simulation == 1, "simulation = True");

    float vel_factor_param;
    pnh.param<float>("vel_factor", vel_factor_param, 0.7);
    vel_factor_max = vel_factor_param;
    
    pnh.param<bool>("laser_altimeter", laser_altimeter, false);
    pnh.param<bool>("self_arming", self_arming, false);

    pnh.param<float>("xy_vel_max", mpc_xy_vel_max, 2.3);
    // pnh.param<float>("z_vel_max", mpc_z_vel_max, 1.0);
    pnh.param<float>("z_vel_max_up", mpc_z_vel_max_up, 2.0);
    pnh.param<float>("z_vel_max_dn", mpc_z_vel_max_dn, 2.0);
    pnh.param<float>("yawrate_max", mc_yawrate_max, 0.8);

    ROS_INFO("BackendDjiRos constructor with id %d",robot_id_);
    ROS_INFO("BackendDjiRos: thresholds = POS %f ORI %f", position_th_, orientation_th_);

    pnh.param<float>("a_land", a_land, 0.7);
    pnh.param<float>("b_land", b_land, 0.3);
    pnh.param<float>("z0_land", z0_land, 1);

    // // Init ros communications
    ros::NodeHandle nh;
    std::string dji_ns = "dji_sdk";

    //std::string drone_ns = "firefly";
        
    //ROS services
    std::string activation_srv = dji_ns + "/activation";
    std::string arming_srv = dji_ns + "/drone_arm_control";
    std::string set_local_pos_ref_srv = dji_ns + "/set_local_pos_ref";
    std::string sdk_control_authority_srv = dji_ns + "/sdk_control_authority";
    std::string drone_task_control_srv = dji_ns + "/drone_task_control";
    std::string mission_waypoint_upload_srv = dji_ns + "/mission_waypoint_upload";
    std::string mission_waypoint_setSpeed_srv = dji_ns + "/mission_waypoint_setSpeed";
    std::string mission_waypoint_action_srv = dji_ns + "/mission_waypoint_action";
    std::string mission_waypoint_reached_topic = "ual_dji/waypoint_reached";

    // ROS subscribed topics
    std::string get_position_topic = dji_ns + "/local_position";
    std::string get_linear_velocity_topic = dji_ns + "/velocity";
    std::string get_angular_velocity_topic = dji_ns + "/angular_velocity_fused";
    std::string get_position_global_topic = dji_ns + "/gps_position";
    std::string get_attitude_topic = dji_ns + "/attitude";
    std::string get_status_topic = dji_ns + "/flight_status";
    std::string get_mode_topic = dji_ns + "/display_mode";

    std::string get_laser_altitude_topic = "/laser_altitude";

    //(NEW)
    std::string get_odometry_topic = "/odometry/filtered_map";
    std::string get_wp_list_topic = "/sdronef1/waypoint_list_g_planner";

    // ROS published topics
    std::string flight_control_topic = dji_ns + "/flight_control_setpoint_generic";
    std::string command_pose_topic = "/sdronef1/command/pose";
    std::string free_wp_topic = "/sdronef1/mav_local_planner/wp_free";
    std::string goal_wp_local_topic ="/sdronef1/mav_local_planner/goal_local_pose";

    // ROS services' Clients
    activation_client_ = nh.serviceClient<dji_sdk::Activation>(activation_srv.c_str());
    arming_client_ = nh.serviceClient<dji_sdk::DroneArmControl>(arming_srv.c_str());
    set_local_pos_ref_client_ = nh.serviceClient<dji_sdk::SetLocalPosRef>(set_local_pos_ref_srv.c_str());
    sdk_control_authority_client_ = nh.serviceClient<dji_sdk::SDKControlAuthority>(sdk_control_authority_srv.c_str());
    drone_task_control_client_ = nh.serviceClient<dji_sdk::DroneTaskControl>(drone_task_control_srv.c_str());
    mission_waypoint_upload_client = nh.serviceClient<dji_sdk::MissionWpUpload>(mission_waypoint_upload_srv.c_str());
    mission_waypoint_setSpeed_client = nh.serviceClient<dji_sdk::MissionWpSetSpeed>(mission_waypoint_setSpeed_srv.c_str());
    mission_waypoint_action_client = nh.serviceClient<dji_sdk::MissionWpAction>(mission_waypoint_action_srv.c_str());

    flight_control_pub_ = nh.advertise<sensor_msgs::Joy>(flight_control_topic.c_str(), 1);
    command_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(command_pose_topic.c_str(), 1);
    _pubRviz = nh.advertise<visualization_msgs::Marker>("ual_dji/vis_marker_ref_waypoints", 1000);
    waypoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/sdronef1/waypoint", 1, true);
    trajectory_pub_ = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/sdronef1/command/trajectory", 1, true);
    waypoint_list_pub_ = nh.advertise<geometry_msgs::PoseArray>("/sdronef1/waypoint_list", 1, true);
    waypoint_reached_pub_ = nh.advertise<std_msgs::Bool>(mission_waypoint_reached_topic.c_str(), 1);

    // mavros_ref_pose_global_pub_ = nh.advertise<mavros_msgs::GlobalPositionTarget>(set_pose_global_topic.c_str(), 1);
    

    //test publishers
    lookahead_pub = nh.advertise<std_msgs::Float64>("lookahead", 1);
    offset_y_pub = nh.advertise<std_msgs::Float64>("offset_y", 1);
    ref_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("ref_pose_y", 1);
    ///

    //local_goal_point
    goal_wp_local_planner_ = nh.subscribe<Waypoint>(goal_wp_local_topic.c_str(), 1, \
    [this](const Waypoint::ConstPtr& _msg){     
        Waypoint local_goal_point_;        
        local_goal_point_ = *_msg;
        ROS_INFO_ONCE("[UAL] local_goal_point_ Subscribed");
        ROS_INFO_STREAM("[UAL] local_goal_point_ : "  <<  " : [" << 
        local_goal_point_.pose.position.x << ";" <<
        local_goal_point_.pose.position.y << ";" <<
        local_goal_point_.pose.position.z << "]");   
        control_mode_ = eControlMode::IDLE;  
        reference_pose_.pose = local_goal_point_.pose;
        publishWPForRviz(reference_pose_, 0);    // type 0 = planner         
    });


    free_waypoint_list_sub_ = nh.subscribe<WaypointList>(free_wp_topic.c_str(), 1, \
        [this](const WaypointList::ConstPtr& _msg) {
        WaypointList freewaypoints_;
        freewaypoints_ = *_msg;
        ROS_INFO_ONCE("[UAL] freewaypoints_ Subscribed");
        ROS_INFO_STREAM("[UAL] WPlist_free!!  size: " << freewaypoints_.poses.size() );
        for (int i=0 ; i < freewaypoints_.poses.size(); i++)
        {
            ROS_INFO_STREAM("[UAL] WPlist_free : "  <<  " : [" << 
            freewaypoints_.poses[i].position.x << ";" <<
            freewaypoints_.poses[i].position.y << ";" <<
            freewaypoints_.poses[i].position.z << "]");            
        }

        control_mode_ = eControlMode::IDLE;  
        reference_pose_.pose = freewaypoints_.poses[freewaypoints_.poses.size() - 1];
        publishWPForRviz(reference_pose_, 0);    // type 0 = planner
    });

    flight_status_sub_ = nh.subscribe<std_msgs::UInt8>(get_status_topic.c_str(), 1, \
        [this](const std_msgs::UInt8::ConstPtr& _msg) {
            this->flight_status_ = *_msg;
    });

    display_mode_sub_ = nh.subscribe<std_msgs::UInt8>(get_mode_topic.c_str(), 1, \
        [this](const std_msgs::UInt8::ConstPtr _msg) {
            this->display_mode_ = *_msg;
    });
    
    position_sub_ = nh.subscribe<geometry_msgs::PointStamped>(get_position_topic.c_str(), 1, \
        [this](const geometry_msgs::PointStamped::ConstPtr& _msg) {
            // this->current_position_ = *_msg;
            // this->cur_pose_.pose.position = this->current_position_.point;
    });

    position_global_sub_ = nh.subscribe<sensor_msgs::NavSatFix>(get_position_global_topic.c_str(), 1, \
        [this](const sensor_msgs::NavSatFix::ConstPtr& _msg) {
            this->current_position_global_ = *_msg;
    });
    
    attitude_sub_ = nh.subscribe<geometry_msgs::QuaternionStamped>(get_attitude_topic.c_str(), 1, \
        [this](const geometry_msgs::QuaternionStamped::ConstPtr& _msg) {
            // this->current_attitude_ = *_msg;
            // this->cur_pose_.pose.orientation = this->current_attitude_.quaternion;
    });

    laser_altitude_sub_ = nh.subscribe<std_msgs::Float64>(get_laser_altitude_topic.c_str(), 1, \
        [this](const std_msgs::Float64::ConstPtr& _msg) {
            this->current_laser_altitude_ = *_msg;
    });

    linear_velocity_sub_ = nh.subscribe<geometry_msgs::Vector3Stamped>(get_linear_velocity_topic.c_str(), 1, \
        [this](const geometry_msgs::Vector3Stamped::ConstPtr& _msg) {
            // this->current_linear_velocity_ = *_msg;
            // this->cur_vel_.twist.linear = this->current_linear_velocity_.vector;
    });

    angular_velocity_sub_ = nh.subscribe<geometry_msgs::Vector3Stamped>(get_angular_velocity_topic.c_str(), 1, \
        [this](const geometry_msgs::Vector3Stamped::ConstPtr& _msg) {
            // this->current_angular_velocity_ = *_msg;
            // this->cur_vel_.twist.angular = this->current_angular_velocity_.vector;
    });

    // Odometry subscriber
    odometry_sub_ = nh.subscribe<nav_msgs::Odometry>(get_odometry_topic.c_str(), 1, \
        [this](const nav_msgs::Odometry::ConstPtr& _msg) {
            this->current_odometry_ = *_msg;   
            //Position
            this->current_position_.point = this->current_odometry_.pose.pose.position;
            this->cur_pose_.pose.position = this->current_position_.point;
            //Orientation
            this->current_attitude_.quaternion = this->current_odometry_.pose.pose.orientation;
            this->cur_pose_.pose.orientation = this->current_attitude_.quaternion;
            //Angular Velocity
            this->current_angular_velocity_.vector = this->current_odometry_.twist.twist.angular;
            this->cur_vel_.twist.angular = this->current_angular_velocity_.vector;
            //Linear Velocity
            this->current_linear_velocity_.vector = this->current_odometry_.twist.twist.linear;
            this->cur_vel_.twist.linear = this->current_linear_velocity_.vector;
                                                                                       
    });

    waypoint_list_sub_ = nh.subscribe<geometry_msgs::PoseArray>(get_wp_list_topic.c_str(), 1, \
        [this](const geometry_msgs::PoseArray::ConstPtr& _msg){
            this->current_wplist_ = *_msg;
            // ROS_INFO_STREAM("[ual] WPlist received! Points:  " << this->current_wplist_.poses.size());
            // Waypoint next_w;        
            // next_w.pose = this->current_wplist_.poses[this->current_wpindex_];
            // goToWaypoint(next_w);
            // waypoint_transition();
        });



    // // TODO: Check this and solve frames issue
    // // Wait until we have pose
    // while (!mavros_has_pose_ && ros::ok()) {
    //     // ROS_INFO("BackendDjiRos: Waiting for pose");
    //     std::this_thread::sleep_for(std::chrono::milliseconds(200));
    // }
    // initHomeFrame();

    control_thread_ = std::thread(&BackendDjiRos::controlThread, this);
        
    // Wait for dji_sdk_node is running
    if (!simulation)
    {
        ros::service::waitForService("dji_sdk/activation");
        dji_sdk::Activation activation;
        activated_ = activation_client_.call(activation);
    }
    else
    {
        activated_ = true;
    }


    ROS_INFO("BackendDjiRos %d running!", robot_id_);
}

void BackendDjiRos::controlThread() {
    ros::param::param<double>("~dji_offboard_rate", control_thread_frequency_, 30.0);
    double hold_pose_time = 3.0;  // [s]  TODO param?
    int buffer_size = std::ceil(hold_pose_time * control_thread_frequency_);
    position_error_.set_size(buffer_size);
    orientation_error_.set_size(buffer_size);
    ros::Rate rate(control_thread_frequency_);

    //test
    std_msgs::Float64 offset_y_;

    geometry_msgs::PoseStamped ref_pose_controller;
    

    ROS_INFO("controlThread_INIT");

    while (ros::ok()) {
        sensor_msgs::Joy reference_joy;
        float control_flag;

        switch(control_mode_) {
        case eControlMode::IDLE:
            break;
        case eControlMode::LOCAL_VEL:

            control_flag = (DJISDK::HORIZONTAL_VELOCITY |
                DJISDK::VERTICAL_VELOCITY       |
                DJISDK::YAW_RATE            |
                DJISDK::HORIZONTAL_GROUND |
                DJISDK::STABLE_ENABLE);

            reference_joy.axes.push_back(reference_vel_.twist.linear.x);
            reference_joy.axes.push_back(reference_vel_.twist.linear.y);
            reference_joy.axes.push_back(reference_vel_.twist.linear.z);
            reference_joy.axes.push_back(reference_vel_.twist.angular.z);
            reference_joy.axes.push_back(control_flag);
            // flight_control_pub_.publish(reference_joy);

            // reference_pose_.pose.position = current_position_.point;
            if ( ros::Time::now().toSec() - last_command_time_.toSec() >=0.5 ) {
                control_mode_ = eControlMode::IDLE;
            }
            // mavros_ref_vel_pub_.publish(ref_vel_);
            // ref_pose_ = cur_pose_;
            break;

        case eControlMode::LOCAL_POSE: 
                        
            BackendDjiRos::Quaternion2EulerAngle(q, roll, pitch, yaw);
             
            control_flag = (DJISDK::HORIZONTAL_POSITION |
                DJISDK::VERTICAL_POSITION       |
                DJISDK::YAW_ANGLE            |
                DJISDK::HORIZONTAL_GROUND |
                DJISDK::STABLE_ENABLE);
            //flag = (0x80 | 0x10 | 0x00 | 0x02 | 0x01);

            offset_x = reference_pose_.pose.position.x - current_position_.point.x;
            offset_y = reference_pose_.pose.position.y - current_position_.point.y;
            offset_xy = sqrt(offset_x*offset_x + offset_y*offset_y);
            offset_x1 = offset_x / offset_xy * std::min(10.0, offset_xy);
            offset_y1 = offset_y / offset_xy * std::min(10.0, offset_xy);
            reference_joy.axes.push_back(vel_factor * offset_x1);
            reference_joy.axes.push_back(vel_factor * offset_y1);
            
            //test
            offset_y_.data = offset_y;
            offset_y_pub.publish(offset_y_);
            ref_pose_pub.publish(reference_pose_);
            ///

            // Publish to Reference Command Pose Controller
            ref_pose_controller.pose.position.x = reference_pose_.pose.position.x;
            ref_pose_controller.pose.position.y = reference_pose_.pose.position.y;
            ref_pose_controller.pose.position.z = reference_pose_.pose.position.z;
            ref_pose_controller.pose.orientation = reference_pose_.pose.orientation;
            command_pose_pub_.publish(ref_pose_controller);

            // std::cout << current_laser_altitude_ << std::endl;

            break; 
        case eControlMode::GLOBAL_POSE:

            control_flag = (DJISDK::HORIZONTAL_POSITION |
                DJISDK::VERTICAL_POSITION       |
                DJISDK::YAW_ANGLE            |
                DJISDK::HORIZONTAL_GROUND |
                DJISDK::STABLE_ENABLE);

            reference_joy.axes.push_back(100000*(reference_pose_global_.longitude - current_position_global_.longitude));
            reference_joy.axes.push_back(100000*(reference_pose_global_.latitude - current_position_global_.latitude));
            reference_joy.axes.push_back(reference_pose_global_.altitude);
            reference_joy.axes.push_back(yaw);
            reference_joy.axes.push_back(control_flag);

            // flight_control_pub_.publish(reference_joy);

            break;
        }
        // // Error history update
        // double dx = ref_pose_.pose.position.x - cur_pose_.pose.position.x;
        // double dy = ref_pose_.pose.position.y - cur_pose_.pose.position.y;
        // double dz = ref_pose_.pose.position.z - cur_pose_.pose.position.z;
        // double positionD = dx*dx + dy*dy + dz*dz; // Equals distance^2

        // double quatInnerProduct = ref_pose_.pose.orientation.x*cur_pose_.pose.orientation.x + \
        // ref_pose_.pose.orientation.y*cur_pose_.pose.orientation.y + \
        // ref_pose_.pose.orientation.z*cur_pose_.pose.orientation.z + \
        // ref_pose_.pose.orientation.w*cur_pose_.pose.orientation.w;
        // double orientationD = 1.0 - quatInnerProduct*quatInnerProduct;  // Equals (1-cos(rotation))/2

        // position_error_.update(positionD);
        // orientation_error_.update(orientationD);

        // State update
        this->state_ = guessState();

        rate.sleep();
    }
}

grvc::ual::State BackendDjiRos::guessState() {
    // Sequentially checks allow state deduction
    if (!simulation)        // UAL STATE for DJI A3/N3
    {        
        if (!this->isReady()) { return uav_abstraction_layer::State::UNINITIALIZED; }
        if (this->flight_status_.data == DJISDK::FlightStatus::STATUS_STOPPED) { 
            if (self_arming) {
                return uav_abstraction_layer::State::LANDED_ARMED;            
            } else {
                return uav_abstraction_layer::State::LANDED_DISARMED; 
            }
        }
        if (this->flight_status_.data == DJISDK::FlightStatus::STATUS_ON_GROUND) { return uav_abstraction_layer::State::LANDED_ARMED;}
        if (this->calling_takeoff && this->flight_status_.data == DJISDK::FlightStatus::STATUS_IN_AIR ) 
            { return uav_abstraction_layer::State::TAKING_OFF;}
        if (this->calling_land && this->flight_status_.data == DJISDK::FlightStatus::STATUS_IN_AIR ) 
            { return uav_abstraction_layer::State::LANDING;}
        if (!this->calling_takeoff && !this->calling_land 
            && this->flight_status_.data == DJISDK::FlightStatus::STATUS_IN_AIR 
            // && this->display_mode_.data == 6) 
            && this->display_mode_.data == DJISDK::DisplayMode::MODE_NAVI_SDK_CTRL) 
            // && this->display_mode_.data == DJISDK::DisplayMode::MODE_P_GPS) 
            {         
                //if (this->calling_waypoint)
                //    return uav_abstraction_layer::State::FLYING_WAYPOINT;
                //else
                //{
                return uav_abstraction_layer::State::FLYING_AUTO;
                //}
            } 

        return uav_abstraction_layer::State::FLYING_MANUAL;
    }
    else
    {
        /// SIMULATION 
        //////////////////////////////////////////////
        // Sequentially checks allow state deduction
        if (!this->isReady()) { return uav_abstraction_layer::State::UNINITIALIZED; }
        if (this->flight_status_.data == DJISDK::FlightStatus::STATUS_STOPPED && current_position_.point.z < 0.35) { 
            if (self_arming) {
                return uav_abstraction_layer::State::LANDED_ARMED;            
            } else {
                return uav_abstraction_layer::State::LANDED_DISARMED; 
            }
        }
        if (this->flight_status_.data == DJISDK::FlightStatus::STATUS_ON_GROUND) { return uav_abstraction_layer::State::LANDED_ARMED;}
        if (this->calling_takeoff ) 
            { return uav_abstraction_layer::State::TAKING_OFF;}
        if (this->calling_land  ) 
            { return uav_abstraction_layer::State::LANDING;}
        if (!this->calling_takeoff && !this->calling_land && current_position_.point.z > 0.5) 
            {                 
                //if (this->calling_waypoint)
                //    return uav_abstraction_layer::State::FLYING_WAYPOINT;
                //else
                //{
                return uav_abstraction_layer::State::FLYING_AUTO;
                //}
            } 
        return uav_abstraction_layer::State::FLYING_MANUAL;
    }    
}

bool BackendDjiRos::altimeter_fail() {
    double alt_2 = current_laser_altitude_.data;
    if(alt_1 == alt_2 && alt_1 != 0.0){
        alt_counter ++;
    }
    else {
        alt_counter = 0;
    }
    // std::cout << "alt_counter  "<< alt_counter << std::endl;
    alt_1 = alt_2;
    if (alt_counter > 100) return true;
    else return false;
}

void BackendDjiRos::Quaternion2EulerAngle(const geometry_msgs::Pose::_orientation_type& q, double& roll, double& pitch, double& yaw)
{
	// roll (x-axis rotation)
	double sinr = +2.0 * (q.w * q.x + q.y * q.z);
	double cosr = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
	roll = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w * q.y - q.z * q.x);
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (q.w * q.z + q.x * q.y);
	double cosy = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
	yaw = atan2(siny, cosy);
}

void BackendDjiRos::setArmed(bool _value) {
    int arm;
    if(_value) {arm=1;}
    else if(!_value) {arm=0;}
    dji_sdk::DroneArmControl arming_service;
    arming_service.request.arm = _value;
    arming_client_.call(arming_service);
}

// void BackendDjiRos::setFlightMode(const std::string& _flight_mode) {
//     mavros_msgs::SetMode flight_mode_service;
//     flight_mode_service.request.base_mode = 0;
//     flight_mode_service.request.custom_mode = _flight_mode;
//     // Set mode: unabortable?
//     while (mavros_state_.mode != _flight_mode && ros::ok()) {
//         if (!flight_mode_client_.call(flight_mode_service)) {
//             ROS_ERROR("Error in set flight mode [%s] service calling!", _flight_mode.c_str());
//         }
//         std::this_thread::sleep_for(std::chrono::milliseconds(300));
// #ifdef MAVROS_VERSION_BELOW_0_20_0
//         ROS_INFO("Set flight mode [%s] response.success = %s", _flight_mode.c_str(), \
//             flight_mode_service.response.success ? "true" : "false");
// #else
//         ROS_INFO("Set flight mode [%s] response.success = %s", _flight_mode.c_str(), \
//             flight_mode_service.response.mode_sent ? "true" : "false");
// #endif
//         ROS_INFO("Trying to set [%s] mode; mavros_state_.mode = [%s]", _flight_mode.c_str(), mavros_state_.mode.c_str());
//     }
// }

void BackendDjiRos::recoverFromManual() {
    if (display_mode_.data != DJISDK::DisplayMode::MODE_P_GPS) {
        ROS_ERROR("Unable to recover from manual. Not in P_GPS MODE");
        ROS_INFO("Please switch rc to P_GPS MODE");
        return;
    }
    dji_sdk::SDKControlAuthority sdk_control_authority;
    sdk_control_authority.request.control_enable = dji_sdk::SDKControlAuthority::Request::REQUEST_CONTROL;
    sdk_control_authority_client_.call(sdk_control_authority);

    reference_vel_.twist.linear.x = 0;
    reference_vel_.twist.linear.y = 0;
    reference_vel_.twist.linear.z = 0;
    reference_vel_.twist.angular.z = 0;

    control_mode_ = eControlMode::LOCAL_VEL;
    ros::Duration(0.5).sleep();

    control_mode_ = eControlMode::IDLE;  
    
    if(sdk_control_authority.response.result){
        ROS_INFO("Recovered from manual mode!");
    } else {
        ROS_WARN("Unable to recover from manual mode (not in manual!)");
    }
    this->state_ = guessState();
}

void BackendDjiRos::setHome(bool set_z) {

    dji_sdk::SetLocalPosRef set_local_pos_ref;
    set_local_pos_ref_client_.call(set_local_pos_ref);
    home_set_ = true;
    // local_start_pos_ = -Eigen::Vector3d(cur_pose_.pose.position.x, \
    //     cur_pose_.pose.position.y, cur_pose_.pose.position.z);
}

void BackendDjiRos::takeOff(double _height) {
    if (_height < 0.0) {
        ROS_ERROR("[UAL] Takeoff height must be positive!");
        return;
    }

    // if(!home_set_){
    //     dji_sdk::SetLocalPosRef set_local_pos_ref;
    //     set_local_pos_ref_client_.call(set_local_pos_ref);
    //     home_set_ = true;
    // }
    
    // Request Control Authority SDK
    // dji_sdk::SDKControlAuthority sdk_control_authority;
    // sdk_control_authority.request.control_enable = dji_sdk::SDKControlAuthority::Request::REQUEST_CONTROL;
    // sdk_control_authority_client_.call(sdk_control_authority);

    // dji_sdk::DroneTaskControl drone_task_control;
    // drone_task_control.request.task = dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF;
    // drone_task_control_client_.call(drone_task_control);
    ROS_INFO("[UAL] Taking Off...");

   
    reference_pose_.pose.position.x = current_position_.point.x;
    reference_pose_.pose.position.y = current_position_.point.y;
    reference_pose_.pose.position.z = _height;
    reference_pose_.pose.orientation = this->current_attitude_.quaternion;
    setPose(reference_pose_);   //Set pose to controller

    calling_takeoff = true; 
    ROS_INFO("[UAL] takeoff_transition = True");

    // while (current_position_.point.z < 1.0) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));        
    // }
    // target_z = 2.0;
    // // going_up = true;
    // if (_height > 1.2) {going_up = true;}
    // else {going_up = false;}




    // while ( !(fabs(_height - current_position_.point.z) < 1.5*position_th_) ) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));        
    // }

    // Not abortable
    // 95% reference altitude
    // while ( !(fabs(current_position_.point.z) >  (0.95*_height)) ) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }

        // Wait until we arrive
    while(!referencePoseReached()  && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }


    // if (flight_status_.data == DJISDK::FlightStatus::STATUS_IN_AIR) {
    // // if(flight_status_ == DJISDK::FlightStatus::STATUS_IN_AIR) {
       
    //     ROS_INFO("Flying!");
    // }
    
     
    calling_takeoff = false;
    ROS_INFO("[UAL] takeoff_transition = false");

    // Update state right now!
    this->state_ = guessState();

    control_mode_ = eControlMode::IDLE;    //Disable control in position

}

void BackendDjiRos::publishWPForRviz(const Waypoint& wpsToPaint, int type) {
    visualization_msgs::MarkerArray markerArray;

    visualization_msgs::Marker marker;
    marker.header.frame_id     = "beacon_map";
    marker.type                = visualization_msgs::Marker::ARROW;
    marker.action              = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x  = 0.0;
    marker.pose.orientation.y  = 0.7071068;
    marker.pose.orientation.z  = 0.0;
    marker.pose.orientation.w  = 0.7071068;  
    marker.scale.x             = 1.0;
    marker.scale.y             = 0.1;
    marker.scale.z             = 0.1;
   
    marker.header.stamp     = ros::Time();
    marker.id               = 0;
    marker.pose.position.x  = wpsToPaint.pose.position.x;
    marker.pose.position.y  = wpsToPaint.pose.position.y;
    marker.pose.position.z  = wpsToPaint.pose.position.z + 1.0;

    if (type == 0)   //planner wp
    {
        marker.color.r = 0.0;
        marker.color.g = 0.7;
        marker.color.b = 1.0;   
        marker.color.a = 0.7; 
    }
    else if (type == 1) //controller wp
    {
        marker.color.r = 1.0;
        marker.color.g = 0.2;
        marker.color.b = 0.8;   
        marker.color.a = 0.7; 
    }


    _pubRviz.publish(marker);
}

double getNextHeight(double freq, int it, double a, double b) {
 return (exp(-1*a*it*1/freq)-b);
}


double getNextHeight_lineal(double z0, float it, double a, double b) {
 return (-a*it + (z0-b));
}

void BackendDjiRos::land() {
        
    dji_sdk::DroneTaskControl drone_task_control;

    ROS_INFO("[UAL] Landing to point...");
    calling_land = true;     
    ROS_INFO("[UAL] landing_transition = True");

    //lineal 
    double freq = 10;
    ros::Rate rate(freq);
    float ros_it = 0;

    Waypoint next_wp, des_land_z;    
    next_wp.pose.position.x = current_position_.point.x;
    next_wp.pose.position.y = current_position_.point.y;
    next_wp.pose.position.z = current_position_.point.z;    
    // Minimum z value
    des_land_z.pose.position.z = 0;
    
    // Current z point is the start point for lineal function
    z0_land = current_position_.point.z;

    // Landing abortable!
    while (!abort_ && ros::ok()) {
        //Get next height from lineal function
        next_wp.pose.position.z = getNextHeight_lineal(z0_land, ros_it, a_land, b_land);
        //ROS_INFO_STREAM("z = " <<  next_wp.pose.position.z);
        setPose(next_wp);   //Set pose to controller


        //If current and desired are minimum or DJI status stopped
        if (simulation)
        {
            ROS_WARN("Simulation %f", z0_land);
            if ((fabs(des_land_z.pose.position.z - current_position_.point.z) < 0.5*position_th_) || (current_position_.point.z <= 0.2))
            {            
                break;  // Out-of-while condition
            }
        }
        else
        {
            if ((fabs(des_land_z.pose.position.z - current_position_.point.z) < 0.5*position_th_) || (this->flight_status_.data == DJISDK::FlightStatus::STATUS_STOPPED)) 
            {            
                break;  // Out-of-while condition
            }
        }
                   
        ros_it = ros_it+0.01;   
        // ROS_INFO_STREAM("ros_it = " <<  ros_it);     
        rate.sleep();
        // ros::spinOnce();
    }

    control_mode_ = eControlMode::IDLE;  

    calling_land = false;
    ROS_INFO("[UAL] landing_transition = False");
    // Update state right now!
    this->state_ = guessState();

    ROS_INFO("Landed! -> Disarm");  // Now disarm!
    // if (!simulation)
    // {setArmed(false);}    
        
}

void BackendDjiRos::land_point(const Waypoint& _wp) {
    
    dji_sdk::DroneTaskControl drone_task_control;

    ROS_INFO("[UAL] Landing to point...");
    calling_land = true;     
    ROS_INFO("[UAL] landing_transition = True");

    //lineal 
    double freq = 10;
    ros::Rate rate(freq);
    float ros_it = 0;

    Waypoint next_wp, des_land_z;    
    next_wp = _wp;        
    // Minimum z value
    des_land_z.pose.position.z = 0;
    
    // Current z point is the start point for lineal function
    z0_land = current_position_.point.z;

    // Landing abortable!
    while (!abort_ && ros::ok()) {
        //Get next height from lineal function
        next_wp.pose.position.z = getNextHeight_lineal(z0_land, ros_it, a_land, b_land);
        ROS_INFO_STREAM("z = " <<  next_wp.pose.position.z);
        setPose(next_wp);   //Set pose to controller
        
        //If current and desired are minimum or DJI status stopped
        if (simulation)
        {
            if ((fabs(des_land_z.pose.position.z - current_position_.point.z) < 0.5*position_th_) || (current_position_.point.z <= 0.2))
            {            
                break;  // Out-of-while condition
            }
        }
        else
        {
            if ((fabs(des_land_z.pose.position.z - current_position_.point.z) < 0.5*position_th_) || (this->flight_status_.data == DJISDK::FlightStatus::STATUS_STOPPED)) 
            {            
                break;  // Out-of-while condition
            }
        }
           
        ros_it = ros_it+0.01;   
        // ROS_INFO_STREAM("ros_it = " <<  ros_it);     
        rate.sleep();
        ros::spinOnce();
    }

    control_mode_ = eControlMode::IDLE;  

    calling_land = false;
    ROS_INFO("[UAL] landing_transition = False");
    // Update state right now!
    this->state_ = guessState();

    ROS_INFO("Landed! -> Disarm");  // Now disarm!
    // if (!simulation)
    // {setArmed(false);}        
}

// ros::ServiceServer go_home_service =
//                 nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(
//                 go_home_srv,
//                 [this](std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
//                 return this->goHome();


void BackendDjiRos::plan(const Waypoint& _wp_start, const Waypoint& _wp_goal) {
   
    ROS_INFO_STREAM("[ual]    Planner wp_start " << ": [" <<
        _wp_start.pose.position.x << ";" <<
        _wp_start.pose.position.y << ";" <<
        _wp_start.pose.position.z << "]" <<                      
        " and yaw: ");   

    ROS_INFO_STREAM("[ual]    Planner wp_goal " << ": [" <<
        _wp_goal.pose.position.x << ";" <<
        _wp_goal.pose.position.y << ";" <<
        _wp_goal.pose.position.z << "]" <<                      
        " and yaw: ");   

    ///// Calling global planner service to  plan wp from start to goal
    std::string service_name ="/sdronef1/global_planner/plan";  
    mav_msgs::EigenTrajectoryPoint start_point, goal_point;
    start_point.position_W.x() = _wp_start.pose.position.x;
    start_point.position_W.y() = _wp_start.pose.position.y;
    start_point.position_W.z() = _wp_start.pose.position.z;
    start_point.orientation_W_B.w() = _wp_start.pose.orientation.w;    
    start_point.orientation_W_B.x() = _wp_start.pose.orientation.x;    
    start_point.orientation_W_B.y() = _wp_start.pose.orientation.y;    
    start_point.orientation_W_B.z() = _wp_start.pose.orientation.z;     
    // start_point.setFromYaw(0.0);

    goal_point.position_W.x() = _wp_goal.pose.position.x;
    goal_point.position_W.y() = _wp_goal.pose.position.y;
    goal_point.position_W.z() = _wp_goal.pose.position.z;
    goal_point.orientation_W_B.w() = _wp_goal.pose.orientation.w;    
    goal_point.orientation_W_B.x() = _wp_goal.pose.orientation.x;    
    goal_point.orientation_W_B.y() = _wp_goal.pose.orientation.y;    
    goal_point.orientation_W_B.z() = _wp_goal.pose.orientation.z;    
    // goal_point.setFromYaw(0.0);
                           
    mav_planning_msgs::PlannerService req;
    mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(start_point,
                                                    &req.request.start_pose);
    mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(goal_point,
                                                    &req.request.goal_pose);

    try {
    ROS_DEBUG_STREAM("Service name: " << service_name);
    if (!ros::service::call(service_name, req)) {
        ROS_WARN_STREAM("Couldn't call service: " << service_name);
    }
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Service Exception: " << e.what());
    }     
}

void BackendDjiRos::goHome() {
    dji_sdk::DroneTaskControl drone_task_control;
    drone_task_control.request.task = dji_sdk::DroneTaskControl::Request::TASK_GOHOME;
    drone_task_control_client_.call(drone_task_control);

    control_mode_ = eControlMode::IDLE; 
}

void BackendDjiRos::setVelocity(const Velocity& _vel) {
    control_mode_ = eControlMode::LOCAL_VEL;  // Velocity control!
    reference_vel_ = _vel;

    last_command_time_ = ros::Time::now();
}

bool BackendDjiRos::isReady() const {    
        return activated_;
}
   
void BackendDjiRos::setPose(const geometry_msgs::PoseStamped& _world) {
    
    // Send to controller
    control_mode_ = eControlMode::LOCAL_POSE;   

    publishWPForRviz(_world, 1);    // type 1 = controller

    vel_factor = vel_factor_max;
    reference_pose_ = _world;
    q.x = _world.pose.orientation.x;
    q.y = _world.pose.orientation.y;
    q.z = _world.pose.orientation.z;
    q.w = _world.pose.orientation.w;

    // control_mode_ = eControlMode::IDLE; //CHANGE to sent 1 Wp controller
}

void BackendDjiRos::setPosePlanner(const geometry_msgs::PoseStamped& _world) {

    // local planner
    
    ROS_INFO_STREAM("[UAL] To Planner goToWaypoint " << " : [" <<
        _world.pose.position.x << ";" <<
        _world.pose.position.y << ";" <<
        _world.pose.position.z << "]" <<                      
        " and yaw: ");  

    publishWPForRviz(_world, 0);    // type 0 = planner

    waypoint_pub_.publish(_world);
    control_mode_ = eControlMode::IDLE;   

    vel_factor = vel_factor_max;
    reference_pose_ = _world;
    q.x = reference_pose_.pose.orientation.x;
    q.y = reference_pose_.pose.orientation.y;
    q.z = reference_pose_.pose.orientation.z;
    q.w = reference_pose_.pose.orientation.w;
}

// TODO: Move from here?
struct PurePursuitOutput {
    geometry_msgs::Point next;
    float t_lookahead;
};

// TODO: Move from here?
PurePursuitOutput DjiPurePursuit(geometry_msgs::Point _current, geometry_msgs::Point _initial, geometry_msgs::Point _final, float _lookahead) {

    PurePursuitOutput out;
    out.next = _current;
    out.t_lookahead = 0;
    if (_lookahead <= 0) {
        ROS_ERROR("Lookahead must be non-zero positive!");
        return out;
    }

    Eigen::Vector3f x0 = Eigen::Vector3f(_current.x, _current.y, _current.z);
    Eigen::Vector3f x1 = Eigen::Vector3f(_initial.x, _initial.y, _initial.z);
    Eigen::Vector3f x2 = Eigen::Vector3f(_final.x, _final.y, _final.z);
    Eigen::Vector3f p = x0;

    Eigen::Vector3f x_21 = x2 - x1;
    float d_21 = x_21.norm();
    float t_min = - x_21.dot(x1-x0) / (d_21*d_21);

    Eigen::Vector3f closest_point = x1 + t_min*(x2-x1);
    float distance = (closest_point - x0).norm();

    float t_lookahead = t_min;
    if (_lookahead > distance) {
        float a = sqrt(_lookahead*_lookahead - distance*distance);
        t_lookahead = t_min + a/d_21;
    }

    if (t_lookahead <= 0.0) {
        p = x1;
        t_lookahead = 0.0;
        // ROS_INFO("p = x1");
    } else if (t_lookahead >= 1.0) {
        p = x2;
        t_lookahead = 1.0;
        // ROS_INFO("p = x2");
    } else {
        p = x1 + t_lookahead*(x2-x1);
        // ROS_INFO("L = %f; norm(x0-p) = %f", _lookahead, (x0-p).norm());
    }

    out.next.x = p(0);
    out.next.y = p(1);
    out.next.z = p(2);
    out.t_lookahead = t_lookahead;
    return out;
}

void BackendDjiRos::goToWaypoint(const Waypoint& _world) {

    // Set pose!
    this->reference_reached_.data = false;

 
    setPosePlanner(_world);

    calling_waypoint = true;

    // Wait until we arrive: abortable
    while(!referencePoseReached() && !abort_ && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    calling_waypoint = false;
    // Update state right now!
    this->state_ = guessState();

    // this->reference_reached_.data = false;
    // waypoint_reached_pub_.publish(this->reference_reached_);
    

    // Freeze in case it's been aborted
    if (abort_ && freeze_) {
        ROS_WARN_STREAM("[UAL] Thread Aborted!! ");
        reference_pose_ = cur_pose_;
        this->reference_reached_.data = false;
        // setPosePlanner(reference_pose_);
        // waypoint_reached_pub_.publish(this->reference_reached_);
    }

    // this->current_wpindex_++;
    // if(this->current_wpindex_ < this->current_wplist_.poses.size() && !abort_ && ros::ok()){
    //     Waypoint next_wp;                      
    //     next_wp.pose = this->current_wplist_.poses[this->current_wpindex_];     
    //     goToWaypoint(next_wp);
    // }
    // else
    // {
    //     this->current_wpindex_ = 0;
    // }     
    
}

void BackendDjiRos::goToTrajectory_controller(const trajectory_msgs::MultiDOFJointTrajectory &trajectory) {

    // Set pose!
    this->reference_reached_.data = false;
     
    ROS_INFO_STREAM("[UAL] Send trajectory to controller ");  

    trajectory_pub_.publish(trajectory);
    control_mode_ = eControlMode::IDLE;   

    vel_factor = vel_factor_max;
    reference_pose_.pose.position.x = trajectory.points[1].transforms[0].translation.x;
    reference_pose_.pose.position.y = trajectory.points[1].transforms[0].translation.y;
    reference_pose_.pose.position.z = trajectory.points[1].transforms[0].translation.z;
   
    reference_pose_.pose.orientation.x = trajectory.points[1].transforms[0].rotation.x;
    reference_pose_.pose.orientation.y = trajectory.points[1].transforms[0].rotation.y;
    reference_pose_.pose.orientation.z = trajectory.points[1].transforms[0].rotation.z;
    reference_pose_.pose.orientation.w = trajectory.points[1].transforms[0].rotation.w;

    // Wait until we arrive: abortable
    while(!referencePoseReached_controller() && !abort_ && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Update state right now!
    this->state_ = guessState();

    // Freeze in case it's been aborted
    if (abort_ && freeze_) {
        ROS_WARN_STREAM("[UAL] Thread Aborted!! ");
        reference_pose_ = cur_pose_;
        this->reference_reached_.data = false;
    }   
    
}

void BackendDjiRos::goToWaypoint_controller(const Waypoint& _world) {

    // Set pose!
    this->reference_reached_.data = false;

 
    setPose(_world);    //Set position to controller


    // Wait until we arrive: abortable
    while(!referencePoseReached_controller() && !abort_ && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Update state right now!
    this->state_ = guessState();

    // this->reference_reached_.data = false;
    // waypoint_reached_pub_.publish(this->reference_reached_);
    

    // Freeze in case it's been aborted
    if (abort_ && freeze_) {
        ROS_WARN_STREAM("[UAL] Thread Aborted!! ");
        reference_pose_ = cur_pose_;
        this->reference_reached_.data = false;
        // setPosePlanner(reference_pose_);
        // waypoint_reached_pub_.publish(this->reference_reached_);
    }          
}

void    BackendDjiRos::goToListofWaypoint(const WaypointList& _wp){


    // show rviz last wp in list
    int size = _wp.poses.size();
    Waypoint last_wp_, next_wp_;
    last_wp_.pose = _wp.poses[_wp.poses.size()-1];
    // next_wp_.pose = _wp.poses[1];
    publishWPForRviz(last_wp_, 0);
    // publishWPForRviz(next_wp_, 0);


    reference_pose_ = last_wp_;
    // reference_pose_ = next_wp_;
    
    //publi  sh
    waypoint_list_pub_.publish(_wp);
    ROS_INFO_STREAM("[UAL] goToListofWaypoint!! ");
    calling_waypoint = true;

    // Wait until we arrive: abortable
    while(!referencePoseReached() && !abort_ && ros::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    calling_waypoint = false;
    // Update state right now!
    this->state_ = guessState();

    // this->reference_reached_.data = false;
    // waypoint_reached_pub_.publish(this->reference_reached_);
    

    // Freeze in case it's been aborted
    if (abort_ && freeze_) {
        ROS_WARN_STREAM("[UAL] Thread Aborted!! ");
        reference_pose_ = cur_pose_;
        this->reference_reached_.data = false;
        // setPosePlanner(reference_pose_);
        // waypoint_reached_pub_.publish(this->reference_reached_);
    }

}

void	BackendDjiRos::goToWaypointGeo(const WaypointGeo& _wp){
    
    dji_sdk::MissionWpAction mission_waypoint_action;
    dji_sdk::MissionWpUpload mission_waypoint_upload;
    
    dji_sdk::MissionWaypointTask mission_waypoint_task;
    dji_sdk::MissionWaypoint current_waypoint;
    dji_sdk::MissionWaypoint mission_waypoint;
    dji_sdk::MissionWaypointAction waypoint_action;

    // waypoint_action.action_repeat = 0;

    std::cout << "test2" << std::endl;

    // current_waypoint.latitude = current_position_global_.latitude;
    // current_waypoint.longitude = current_position_global_.longitude;
    // current_waypoint.altitude = current_position_global_.altitude;

    mission_waypoint.latitude = _wp.latitude;
    mission_waypoint.longitude = _wp.longitude;
    mission_waypoint.altitude = _wp.altitude;
    mission_waypoint.damping_distance = 0;
    mission_waypoint.target_yaw = 0;
    mission_waypoint.target_gimbal_pitch = 0;
    mission_waypoint.turn_mode = 0;
    mission_waypoint.has_action = 0;
    // mission_waypoint.action_time_limit = 100;
    // mission_waypoint.waypoint_action = waypoint_action;

    mission_waypoint_task.mission_waypoint.push_back(mission_waypoint);

    mission_waypoint_task.velocity_range = 2.0;
    mission_waypoint_task.idle_velocity = 5;
    mission_waypoint_task.action_on_finish = dji_sdk::MissionWaypointTask::FINISH_NO_ACTION;
    mission_waypoint_task.mission_exec_times = 1;
    mission_waypoint_task.yaw_mode = dji_sdk::MissionWaypointTask::YAW_MODE_AUTO;
    mission_waypoint_task.trace_mode = dji_sdk::MissionWaypointTask::TRACE_POINT;
    mission_waypoint_task.action_on_rc_lost = dji_sdk::MissionWaypointTask::ACTION_AUTO;
    mission_waypoint_task.gimbal_pitch_mode =  dji_sdk::MissionWaypointTask::GIMBAL_PITCH_FREE;
    std::cout << "test4" << std::endl;

    // mission_waypoint_task.mission_waypoint[0] = mission_waypoint[0];
    std::cout << "test5" << std::endl;

    mission_waypoint_upload.request.waypoint_task = mission_waypoint_task;
    std::cout << "test6" << std::endl;

    mission_waypoint_upload_client.call(mission_waypoint_upload);
   
    mission_waypoint_action.request.action = 0;
    mission_waypoint_action_client.call(mission_waypoint_action);
    
    
    // control_mode_ = eControlMode::GLOBAL_POSE; // Control in position
    
    // reference_pose_global_.latitude = _wp.latitude;
    // reference_pose_global_.longitude = _wp.longitude;
    // reference_pose_global_.altitude = _wp.altitude;
    

    // // Wait until we arrive: abortable
    // while(!referencePoseReached() && !abort_ && ros::ok()) {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }
    // // Freeze in case it's been aborted
    // if (abort_ && freeze_) {
    //     ref_pose_ = cur_pose_;
    // }
}

/*void BackendDjiRos::trackPath(const WaypointList &_path) {
    // TODO: basic imlementation, ideally different from a stack of gotos
}*/

Pose BackendDjiRos::pose() {
        Pose out;

        out.pose.position.x = current_position_.point.x;
        out.pose.position.y = current_position_.point.y;
        if (laser_altimeter == true && current_laser_altitude_.data != 0.0 && !altimeter_fail() ) {
            out.pose.position.z = current_laser_altitude_.data;
        } else {
            out.pose.position.z = current_position_.point.z;
        }
        out.pose.orientation.x = current_attitude_.quaternion.x;
        out.pose.orientation.y = current_attitude_.quaternion.y;
        out.pose.orientation.z = current_attitude_.quaternion.z;
        out.pose.orientation.w = current_attitude_.quaternion.w;

        // if (pose_frame_id_ == "") {
        //     // Default: local pose
        //     out.header.frame_id = uav_home_frame_id_;
        // }
        // else {
        //     // Publish pose in different frame
        //     Pose aux = out;
        //     geometry_msgs::TransformStamped transformToPoseFrame;
        //     std::string pose_frame_id_map = "inv_" + pose_frame_id_;

        //     if ( cached_transforms_.find(pose_frame_id_map) == cached_transforms_.end() ) {
        //         // inv_pose_frame_id_ not found in cached_transforms_
        //         tf2_ros::Buffer tfBuffer;
        //         tf2_ros::TransformListener tfListener(tfBuffer);
        //         transformToPoseFrame = tfBuffer.lookupTransform(pose_frame_id_,uav_home_frame_id_, ros::Time(0), ros::Duration(1.0));
        //         cached_transforms_[pose_frame_id_map] = transformToPoseFrame; // Save transform in cache
        //     } else {
        //         // found in cache
        //         transformToPoseFrame = cached_transforms_[pose_frame_id_map];
        //     }

        //     tf2::doTransform(aux, out, transformToPoseFrame);
        //     out.header.frame_id = pose_frame_id_;
        // }

        return out;
}

std_msgs::Bool BackendDjiRos::referenceReached(){
    return reference_reached_;
}

WaypointList BackendDjiRos::waypoint_list() const {

}

Pose BackendDjiRos::referencePose() {
    return reference_pose_;
}

Velocity BackendDjiRos::velocity() const {
    Velocity out;

    out.twist.linear = current_linear_velocity_.vector;
    out.twist.angular = current_angular_velocity_.vector;

    return out;
}

Odometry BackendDjiRos::odometry() const {
    Odometry out;

    out = current_odometry_;

    return out;
}

Transform BackendDjiRos::transform() const {
    Transform out;
    out.header.stamp = ros::Time::now();
    out.header.frame_id = "map";
    out.child_frame_id = "uav_" + std::to_string(robot_id_);

    out.transform.translation.x = current_position_.point.x;
    out.transform.translation.y = current_position_.point.y;
    out.transform.translation.z = current_position_.point.z;
    out.transform.rotation = current_attitude_.quaternion;        

    return out;
}

bool BackendDjiRos::referencePoseReached() {


    double dx = reference_pose_.pose.position.x - cur_pose_.pose.position.x;
    double dy = reference_pose_.pose.position.y - cur_pose_.pose.position.y;
    double dz = reference_pose_.pose.position.z - cur_pose_.pose.position.z;
    double positionD = dx*dx + dy*dy + dz*dz; // Equals distance^2

    double quatInnerProduct = reference_pose_.pose.orientation.x*cur_pose_.pose.orientation.x + \
    reference_pose_.pose.orientation.y*cur_pose_.pose.orientation.y + \
    reference_pose_.pose.orientation.z*cur_pose_.pose.orientation.z + \
    reference_pose_.pose.orientation.w*cur_pose_.pose.orientation.w;
    double orientationD = 1 - quatInnerProduct*quatInnerProduct;  // Equals (1-cos(rotation))/2

    // ROS_INFO("ref_position = [%f, %f, %f] cur_position = [%f, %f, %f]", \
    // reference_pose_.pose.position.x, reference_pose_.pose.position.y, reference_pose_.pose.position.z, \
    // cur_pose_.pose.position.x, cur_pose_.pose.position.y, cur_pose_.pose.position.z);
    // ROS_INFO("pD = %f,\t oD = %f", positionD, orientationD);

    // if ((positionD > position_th_) || (orientationD > orientation_th_) ) {
    //     return false;
    // } else {
    //     // cur_pose_ = ref_pose_;
    //     return true;
    // }

    
    if ((positionD > position_th_)) {
        this->reference_reached_.data = false;        
        return false;
    } else {
        // cur_pose_ = ref_pose_;
        this->reference_reached_.data = true;
        waypoint_reached_pub_.publish(this->reference_reached_);
        ROS_INFO("[UAL] WP reached");
        return true;
    }

}

bool BackendDjiRos::referencePoseReached_controller() {


    double dx = reference_pose_.pose.position.x - cur_pose_.pose.position.x;
    double dy = reference_pose_.pose.position.y - cur_pose_.pose.position.y;
    double dz = reference_pose_.pose.position.z - cur_pose_.pose.position.z;
    double positionD = dx*dx + dy*dy + dz*dz; // Equals distance^2

    double quatInnerProduct = reference_pose_.pose.orientation.x*cur_pose_.pose.orientation.x + \
    reference_pose_.pose.orientation.y*cur_pose_.pose.orientation.y + \
    reference_pose_.pose.orientation.z*cur_pose_.pose.orientation.z + \
    reference_pose_.pose.orientation.w*cur_pose_.pose.orientation.w;
    double orientationD = 1 - quatInnerProduct*quatInnerProduct;  // Equals (1-cos(rotation))/2

    //ROS_INFO("ref_position = [%f, %f, %f] cur_position = [%f, %f, %f]", \
    reference_pose_.pose.position.x, reference_pose_.pose.position.y, reference_pose_.pose.position.z, \
    cur_pose_.pose.position.x, cur_pose_.pose.position.y, cur_pose_.pose.position.z);
    //ROS_INFO("pD = %f,\t oD = %f", positionD, orientationD);
    
    if ((positionD > 0.1 ) || (orientationD > (orientation_th_))  ) {
        this->reference_reached_.data = false;        
        return false;
    } else {
        // cur_pose_ = ref_pose_;
        this->reference_reached_.data = true;
        waypoint_reached_pub_.publish(this->reference_reached_);
        ROS_INFO("[UAL] WP controller reached");
        return true;
    }

}

// void BackendDjiRos::initHomeFrame() {

//     uav_home_frame_id_ = "uav_" + std::to_string(robot_id_) + "_home";
//     local_start_pos_ << 0.0, 0.0, 0.0;

//     // Get frame from rosparam
//     std::string parent_frame;
//     std::vector<double> home_pose(3, 0.0);

//     ros::param::get("~home_pose",home_pose);
//     ros::param::param<std::string>("~home_pose_parent_frame", parent_frame, "map");

//     geometry_msgs::TransformStamped static_transformStamped;

//     static_transformStamped.header.stamp = ros::Time::now();
//     static_transformStamped.header.frame_id = parent_frame;
//     static_transformStamped.child_frame_id = uav_home_frame_id_;
//     static_transformStamped.transform.translation.x = home_pose[0];
//     static_transformStamped.transform.translation.y = home_pose[1];
//     static_transformStamped.transform.translation.z = home_pose[2];

//     if(parent_frame == "map" || parent_frame == "") {
//         static_transformStamped.transform.rotation.x = 0;
//         static_transformStamped.transform.rotation.y = 0;
//         static_transformStamped.transform.rotation.z = 0;
//         static_transformStamped.transform.rotation.w = 1;
//     }
//     else {
//         tf2_ros::Buffer tfBuffer;
//         tf2_ros::TransformListener tfListener(tfBuffer);
//         geometry_msgs::TransformStamped transform_to_map;
//         transform_to_map = tfBuffer.lookupTransform(parent_frame, "map", ros::Time(0), ros::Duration(2.0));
//         static_transformStamped.transform.rotation = transform_to_map.transform.rotation;
//     }

//     static_tf_broadcaster_ = new tf2_ros::StaticTransformBroadcaster();
//     static_tf_broadcaster_->sendTransform(static_transformStamped);
// }

}}	// namespace grvc::ual
