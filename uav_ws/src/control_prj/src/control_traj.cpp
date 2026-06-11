/**
 * Logistics UAV Mission Controller 
 * Features: Visual Servoing, Absolute Position Lock for Blind Drop/Grab, GPIO Integration.
 * Filtered Initialization & Vertical Takeoff Enforced.

 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/SetMode.h>
#include <tf/tf.h>
#include <cmath>
#include <algorithm>
#include <mavros_msgs/PositionTarget.h> // 新增

const double DT = 0.05; // 20Hz 主控频率

enum MissionState {
    WAIT_FOR_WAYPOINTS,    
    WAITING_FOR_OFFBOARD,  
    TAKEOFF,               
    NAVIGATE_SEARCH,       
    VISUAL_ALIGN,
    WAITING_FOR_STEADY,          
    DESCEND_TO_GRAB,       
    ASCEND_TO_CRUISE,      
    NAVIGATE_DROP,         
    DROP_OBJECT,
    ASCEND_TO_DROP,        
    RETURN_HOME,           
    LAND,
    FINISH
};

enum VisualAlignPhase {
    VIS_SAMPLE,   // 悬停采样视觉误差
    VIS_MOVE,     // 根据平均误差飞一次固定修正
    VIS_SETTLE    // 到点后悬停稳定，再重新采样
};

class LogisticsMissionController {
private:
    ros::NodeHandle nh_;
    ros::Timer control_timer_;
    ros::Time last_req_time_;

    // ROS 订阅与发布
    ros::Subscriber state_sub_, pose_sub_;
    ros::Subscriber cam_detect_sub_, cam_offset_sub_;
    ros::Subscriber search_point_sub_, drop_point_sub_, emergency_sub_;
    
    ros::Publisher cmd_pose_pub_; 
    ros::Publisher gpio_trigger_pub_;
    ros::Publisher vision_switch_pub_;
    ros::Publisher land_done_pub_; 
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    // 状态与传感器数据
    mavros_msgs::State mavros_state_;
    geometry_msgs::PoseStamped current_pose_;
    
    // 目标航点与视觉数据
    geometry_msgs::Point search_point_, drop_point_, object_offset_;
    
    bool search_point_received_ = false, drop_point_received_ = false;
    bool object_detected_ = false, home_recorded_ = false;

    // 逻辑控制与标志位
    MissionState current_state_ = WAIT_FOR_WAYPOINTS;
    int hover_ticks_ = 0;
    int err_ticks_ = 0;
    int lost_ticks_ = 0; 
    
    // 绝对位置锚点 
    double home_x_ = 0.0, home_y_ = 0.0, home_z_ = 0.0;
    double lock_x_ = 0.0, lock_y_ = 0.0; 
    double initial_yaw_ = 0.0;
    bool yaw_recorded_ = false;
    int fool_sum = 0;
    int fool_idiot = 0;
    bool fool_check = false;
    bool fool_land = true;
    bool switch_vision = true; 
    bool enkill_ = false; 
    double dist_land = 0.05;
    int collect_z = 1;
    int init_pose_count_ = 0;
    const int INIT_POSE_FRAMES = 50; 
    const int FOOL_FRAMES_DEL = 500;
    double sum_x_ = 0.0, sum_y_ = 0.0 , sum_z_ = 0.0;
    double sum_sin_yaw_ = 0.0, sum_cos_yaw_ = 0.0; 
    double sp_x_ = 0.0, sp_y_ = 0.0, sp_z_ = 0.0; // 虚拟目标点
    bool sp_init_ = false;                        

    bool visual_align_init_ = false;
    VisualAlignPhase visual_phase_ = VIS_SAMPLE;

    double visual_hold_x_ = 0.0;
    double visual_hold_y_ = 0.0;
    double visual_move_target_x_ = 0.0;
    double visual_move_target_y_ = 0.0;

    double visual_sample_sum_x_ = 0.0;
    double visual_sample_sum_y_ = 0.0;
    int visual_sample_count_ = 0;

    int visual_phase_ticks_ = 0;
    int visual_move_round_ = 0;


    const double MAX_XY_VEL = 0.2;   // 巡航平移最高限速 (m/s)
    const double MAX_Z_VEL  = 0.4;   // 垂直升降最高限速 (m/s)
    const double MAX_ACC_XY = 0.4;   // 水平加速度限制 (m/s^2)

    const double CRUISE_HEIGHT = 1.3;   
    const double GRAB_HEIGHT = 0.86;    
    const double POSITION_TOLERANCE = 0.1; 
    const double ALIGN_TOLERANCE = 0.035;
    const int ACTION_DELAY_TICKS = 20;

    const double KP_CAM = 0.8;
    const double KP_FALL = 0.1;

    // DESCEND_TO_GRAB 阶段 XY 速度前馈 PID：
    // 只输出 velocity feedforward，不改 lock 点，不改下降状态机逻辑。
    const double DESCEND_XY_PID_KP = 0.6;
    const double DESCEND_XY_PID_KI = 0;
    const double DESCEND_XY_PID_KD = 0.06;
    const double DESCEND_XY_PID_VEL_MAX = 0.5;
    const double DESCEND_XY_PID_I_MAX = 0.20;

    double descend_pid_ix_ = 0.0;
    double descend_pid_iy_ = 0.0;
    double descend_pid_prev_ex_ = 0.0;
    double descend_pid_prev_ey_ = 0.0;
    bool descend_pid_init_ = false;

    // 离散式视觉对准参数：
    // 采样平均 -> 尽量一次修正到目标上方 -> 到点稳定 -> 再采样判断。
    const int VIS_SAMPLE_FRAMES = 30;          // 每次累计多少帧视觉误差，减少采样等待时间
    const double VIS_STEP_TOL = 0.04;         // 平均误差小于该值即认为对准，单位 m
    const double VIS_LOCK_TOL = 0.04;         // 小误差直接锁点下降，不再反复小步修正
    const double VIS_MAX_STEP = 0.2;         // 单次最大水平修正距离，单位 m
    const double VIS_MOVE_REACH_TOL = 0.04;   // 到达单次修正点的 XY 容差，单位 m
    const int VIS_SETTLE_TICKS = 20;           // 到点后悬停稳定时间，20Hz下约1s
    const int VIS_MOVE_TIMEOUT_TICKS = 80;     // 单次移动超时，20Hz下80tick约4s
    const int VIS_MAX_ROUNDS = 2;              // 最多修正次数，避免反复来回调整
    const int VIS_LOST_MAX_TICKS = 200;        // 采样阶段连续丢目标多久返回搜索点


    struct AxisSmoother {
        double p = 0.0;
        double v = 0.0; 
        double max_v;   
        double max_a;   

        AxisSmoother(double v_limit, double a_limit) : max_v(v_limit), max_a(a_limit) {}

        void update(double target_p, double dt, double kp_smooth = 0.6) {

            double err = target_p - p;
            double desired_v = err * kp_smooth;

            desired_v = std::max(-max_v, std::min(max_v, desired_v));

            double dv = desired_v - v;
            double max_dv = max_a * dt;
            dv = std::max(-max_dv, std::min(max_dv, dv));

            v += dv;
            p += v * dt;
        }
        
        void reset(double current_p) {
            p = current_p;
            v = 0.0;
        }
    };

    AxisSmoother smooth_x_{MAX_XY_VEL, MAX_ACC_XY};
    AxisSmoother smooth_y_{MAX_XY_VEL, MAX_ACC_XY};
    AxisSmoother smooth_z_{MAX_Z_VEL, 0.2}; 

public:
    LogisticsMissionController() : nh_("~") {
        initROS();
        ros::Duration(1).sleep(); 
        control_timer_ = nh_.createTimer(ros::Duration(DT), &LogisticsMissionController::controlLoop, this);
        ROS_INFO(">>> Logistics Controller Started. Waiting for waypoints... <<<");
        ROS_INFO(">>> Switch to Kinematic Smoother Position Control <<<");
    }

private:
    void state_cb(const mavros_msgs::State::ConstPtr& msg) { mavros_state_ = *msg; }
    
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
        tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        if (!home_recorded_ && current_pose_.pose.position.z > -0.5 && ++fool_sum >= FOOL_FRAMES_DEL) {
            sum_x_ += current_pose_.pose.position.x;
            sum_y_ += current_pose_.pose.position.y;
            sum_z_ += current_pose_.pose.position.z;
            sum_sin_yaw_ += std::sin(yaw);
            sum_cos_yaw_ += std::cos(yaw);
    
            init_pose_count_++;
            
            if (init_pose_count_ >= INIT_POSE_FRAMES) {
                home_x_ = sum_x_ / INIT_POSE_FRAMES;
                home_y_ = sum_y_ / INIT_POSE_FRAMES;
                home_z_ = sum_z_ / INIT_POSE_FRAMES;
                initial_yaw_ = std::atan2(sum_sin_yaw_, sum_cos_yaw_);
                
                home_recorded_ = true;
                yaw_recorded_ = true; // 航向同步锁死
                ROS_INFO(">>> Home Locked (50-frame filtered): X:%.2f, Y:%.2f, Z:%.2f, Yaw:%.2f", home_x_, home_y_, home_z_, initial_yaw_);
            }
        }
    }

    void cam_detect_cb(const std_msgs::Bool::ConstPtr& msg) { object_detected_ = msg->data; }
    void cam_offset_cb(const geometry_msgs::Point::ConstPtr& msg) { object_offset_ = *msg; }
    void search_point_cb(const geometry_msgs::Point::ConstPtr& msg) { 
        search_point_ = *msg; search_point_received_ = true;
        ROS_INFO("Search Point Locked: (%.2f, %.2f) | Cruise: (%.2f)", search_point_.x, search_point_.y,search_point_.z);
    }
    void drop_point_cb(const geometry_msgs::Point::ConstPtr& msg) {
        drop_point_ = *msg; drop_point_received_ = true; 
        ROS_INFO("Drop Point Locked: (%.2f, %.2f) | Cruise: (%.2f)", drop_point_.x, drop_point_.y,drop_point_.z + 0.04);
    }
    void emergency_cb(const std_msgs::Bool::ConstPtr& msg) { 
        enkill_ = msg->data; 
        if (enkill_){
            switchState(RETURN_HOME);
            ROS_WARN(">>> Trigger to return! Please check <<< ");
        }
    }

    void initROS() {
        state_sub_ = nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &LogisticsMissionController::state_cb, this);
        pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &LogisticsMissionController::pose_cb, this);
        cam_detect_sub_ = nh_.subscribe<std_msgs::Bool>("/camera/object_detected", 10, &LogisticsMissionController::cam_detect_cb, this);
        cam_offset_sub_ = nh_.subscribe<geometry_msgs::Point>("/camera/object_offset", 10, &LogisticsMissionController::cam_offset_cb, this);
        search_point_sub_ = nh_.subscribe<geometry_msgs::Point>("/mission/search_point", 10, &LogisticsMissionController::search_point_cb, this);
        drop_point_sub_ = nh_.subscribe<geometry_msgs::Point>("/mission/drop_point", 10, &LogisticsMissionController::drop_point_cb, this);
        emergency_sub_ = nh_.subscribe<std_msgs::Bool>("/mission/rtl", 10, &LogisticsMissionController::emergency_cb, this);

        // cmd_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
        cmd_pose_pub_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        gpio_trigger_pub_ = nh_.advertise<std_msgs::Bool>("/gpio_trigger", 10);
        vision_switch_pub_ = nh_.advertise<std_msgs::Bool>("/camera/mode_switch", 10);
        land_done_pub_ = nh_.advertise<std_msgs::Bool>("/land_done", 10); 
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode"); 
    }

    void triggerGrabMechanism() { ROS_INFO("[Hardware] Reached grab point. Magnetic attachment engaged."); }
    void triggerDropMechanism() {
        ROS_INFO("[Hardware] Sending DROP signal to magnetic attachment...");
        std_msgs::Bool trigger_msg; trigger_msg.data = true; 
        gpio_trigger_pub_.publish(trigger_msg);
    }
    void landdone(){
        ROS_INFO("[Land] Send landing to host...");
        std_msgs::Bool done_msg; done_msg.data = true;
        land_done_pub_.publish(done_msg);
    }
    void switch2Grab() {
        ROS_INFO("[Vision] Switch to Grab...");
        std_msgs::Bool switch_msg; switch_msg.data = true; 
        vision_switch_pub_.publish(switch_msg);
    }
    void switch2Land() {
        ROS_INFO("[Vision] Switch to Land...");
        std_msgs::Bool switch_msg; switch_msg.data = false; 
        vision_switch_pub_.publish(switch_msg);
    }

    double clampValue(double v, double min_v, double max_v) {
        return std::max(min_v, std::min(max_v, v));
    }

    void resetDescendXYPID() {
        descend_pid_ix_ = 0.0;
        descend_pid_iy_ = 0.0;
        descend_pid_prev_ex_ = 0.0;
        descend_pid_prev_ey_ = 0.0;
        descend_pid_init_ = false;
    }

    void resetVisualAlignSample() {
        visual_sample_sum_x_ = 0.0;
        visual_sample_sum_y_ = 0.0;
        visual_sample_count_ = 0;
    }

    void resetVisualAlignCache() {
        visual_align_init_ = false;
        visual_phase_ = VIS_SAMPLE;

        visual_hold_x_ = 0.0;
        visual_hold_y_ = 0.0;
        visual_move_target_x_ = 0.0;
        visual_move_target_y_ = 0.0;

        resetVisualAlignSample();

        visual_phase_ticks_ = 0;
        visual_move_round_ = 0;
    }

    void switchState(MissionState new_state) {
        current_state_ = new_state;
        hover_ticks_ = 0;
        lost_ticks_ = 0;
        err_ticks_ = 0;
        last_req_time_ = ros::Time::now();

        if (new_state != VISUAL_ALIGN) {
            resetVisualAlignCache();
        }

        if (new_state != DESCEND_TO_GRAB) {
            resetDescendXYPID();
        }
    }

    void controlLoop(const ros::TimerEvent&) {
        if (!mavros_state_.connected) return;

        // geometry_msgs::PoseStamped cmd_pose;
        // cmd_pose.header.stamp = ros::Time::now();
        // cmd_pose.header.frame_id = "map";
        
        // cmd_pose.pose.orientation.x = 0.0;
        // cmd_pose.pose.orientation.y = 0.0;
        // cmd_pose.pose.orientation.z = 0.0;
        // cmd_pose.pose.orientation.w = 1.0; 
        // if (yaw_recorded_) {
        //     cmd_pose.pose.orientation = tf::createQuaternionMsgFromYaw(initial_yaw_);
        // }

        double target_x = current_pose_.pose.position.x;
        double target_y = current_pose_.pose.position.y;
        double target_z = current_pose_.pose.position.z;

        
        switch (current_state_) {
            case WAIT_FOR_WAYPOINTS:
                if (search_point_received_ && drop_point_received_) {
                    if (home_recorded_) {
                        ROS_WARN(">>> Mission coordinates and Home lock confirmed. Ready for OFFBOARD. <<<");
                        switchState(WAITING_FOR_OFFBOARD);
                    } else {
                        ROS_INFO_THROTTLE(1.0, "Waypoints received, waiting for 50-frame Home location filter to complete...");
                    }
                }
                break;

            case WAITING_FOR_OFFBOARD: {
                target_x = home_x_;
                target_y = home_y_;
                target_z = home_z_;

                if (ros::Time::now() - last_req_time_ > ros::Duration(5.0)) {
                    last_req_time_ = ros::Time::now();
                    if (mavros_state_.mode != "OFFBOARD") {
                        mavros_msgs::SetMode offb_set_mode;
                        offb_set_mode.request.custom_mode = "OFFBOARD";
                        if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) ROS_INFO("Offboard enabled");
                    } else if (!mavros_state_.armed) {
                        mavros_msgs::CommandBool arm_cmd;
                        arm_cmd.request.value = true;
                        if (arming_client_.call(arm_cmd) && arm_cmd.response.success) ROS_INFO("Vehicle armed");
                    }
                }

                if (mavros_state_.mode == "OFFBOARD" && mavros_state_.armed) {
                    ROS_WARN(">>> ALL SYSTEMS GO. TAKEOFF INITIATED. <<<");
                    switchState(TAKEOFF);
                }
                break;
            }

            case TAKEOFF:
                target_x = home_x_;
                target_y = home_y_;
                target_z = search_point_.z + home_z_;
                
                if (std::abs(target_z - current_pose_.pose.position.z) < POSITION_TOLERANCE) {
                    if (++hover_ticks_ > ACTION_DELAY_TICKS) {
                        ROS_INFO("Reached cruise altitude. Heading to Search Point.");
                        switchState(NAVIGATE_SEARCH);
                        switch2Grab();
                    }
                } else hover_ticks_ = 0;
                break;

            case NAVIGATE_SEARCH: {
                target_x = search_point_.x + home_x_;
                target_y = search_point_.y + home_y_;
                target_z = search_point_.z + home_z_;
                
                if (getDistanceXY(target_x, target_y) < POSITION_TOLERANCE) {
                    if (++hover_ticks_ > 30) {
                        ROS_INFO("Hovering at Search Area. Waiting for camera...");
                        switchState(VISUAL_ALIGN);
                        break;
                    }
                } else hover_ticks_ = 0;

                break;
            }

            case VISUAL_ALIGN: {

                if (!visual_align_init_) {
                    visual_hold_x_ = search_point_.x + home_x_;
                    visual_hold_y_ = search_point_.y + home_y_;
                    visual_move_target_x_ = visual_hold_x_;
                    visual_move_target_y_ = visual_hold_y_;

                    visual_phase_ = VIS_SAMPLE;
                    resetVisualAlignSample();
                    visual_phase_ticks_ = 0;
                    visual_move_round_ = 0;
                    visual_align_init_ = true;

                    ROS_WARN(">>> VISUAL_ALIGN discrete init at X=%.3f Y=%.3f <<<",
                             visual_hold_x_, visual_hold_y_);
                }

                target_z = search_point_.z + home_z_;

                if (visual_phase_ == VIS_SAMPLE) {
                    // 采样阶段只悬停，不根据实时视觉误差移动飞机
                    target_x = visual_hold_x_;
                    target_y = visual_hold_y_;

                    if (!object_detected_) {
                        if (++lost_ticks_ > VIS_LOST_MAX_TICKS) {
                            ROS_WARN("Target lost during visual sampling! Reverting to search.");
                            switchState(NAVIGATE_SEARCH);
                            break;
                        }
                        break;
                    }

                    lost_ticks_ = 0;

                    // 视觉节点已经按照旧视觉节点的 x/y 对应关系输出米制 XY 偏差：
                    // object_offset_.x 对应旧 dy，object_offset_.y 对应旧 dx。
                    visual_sample_sum_x_ += object_offset_.x;
                    visual_sample_sum_y_ += object_offset_.y;
                    visual_sample_count_++;

                    ROS_INFO_THROTTLE(
                        0.5,
                        "[VIS_SAMPLE] count=%d/%d, hold=(%.3f, %.3f), err_now=(%.3f, %.3f), area=%.0f, round=%d",
                        visual_sample_count_, VIS_SAMPLE_FRAMES,
                        visual_hold_x_, visual_hold_y_,
                        object_offset_.x, object_offset_.y, object_offset_.z,
                        visual_move_round_
                    );

                    if (visual_sample_count_ >= VIS_SAMPLE_FRAMES) {
                        double avg_err_x = visual_sample_sum_x_ / visual_sample_count_;
                        double avg_err_y = visual_sample_sum_y_ / visual_sample_count_;
                        double avg_err = std::sqrt(avg_err_x * avg_err_x + avg_err_y * avg_err_y);

                        ROS_WARN(">>> VIS_SAMPLE AVG err=(%.3f, %.3f), norm=%.3f, round=%d <<<",
                                 avg_err_x, avg_err_y, avg_err, visual_move_round_);

                        // 满足容差，或者已经达到最大修正次数：锁定目标物世界坐标并进入下降
                        if (avg_err < VIS_STEP_TOL || visual_move_round_ >= VIS_MAX_ROUNDS) {
                            lock_x_ = current_pose_.pose.position.x - avg_err_x;
                            lock_y_ = current_pose_.pose.position.y - avg_err_y;

                            ROS_WARN(">>> VISUAL LOCK OK: lock_x=%.3f lock_y=%.3f avg_err=%.3f round=%d <<<",
                                     lock_x_, lock_y_, avg_err, visual_move_round_);

                            switchState(WAITING_FOR_STEADY);
                            break;
                        }

                        // 不满足容差：根据平均误差尽量一次修正到目标上方。
                        // 保持旧控制方向：target = current - offset
                        double step_x = -avg_err_x;
                        double step_y = -avg_err_y;

                        double step_norm = std::sqrt(step_x * step_x + step_y * step_y);

                        // 大误差限幅，避免一次修正过猛
                        if (step_norm > VIS_MAX_STEP) {
                            double scale = VIS_MAX_STEP / step_norm;
                            step_x *= scale;
                            step_y *= scale;
                            step_norm = VIS_MAX_STEP;
                        }

                        // 小误差不再反复小步移动，直接锁点下降
                        if (avg_err < VIS_LOCK_TOL) {
                            lock_x_ = current_pose_.pose.position.x - avg_err_x;
                            lock_y_ = current_pose_.pose.position.y - avg_err_y;

                            ROS_WARN(">>> VISUAL LOCK BY TOL: lock_x=%.3f lock_y=%.3f avg_err=%.3f <<<",
                                     lock_x_, lock_y_, avg_err);

                            switchState(WAITING_FOR_STEADY);
                            break;
                        }

                        visual_move_target_x_ = current_pose_.pose.position.x + step_x;
                        visual_move_target_y_ = current_pose_.pose.position.y + step_y;
                        visual_move_round_++;
                        visual_phase_ticks_ = 0;
                        visual_phase_ = VIS_MOVE;

                        ROS_WARN(">>> VIS_MOVE round=%d step=(%.3f, %.3f), target=(%.3f, %.3f), avg_err=%.3f <<<",
                                 visual_move_round_, step_x, step_y,
                                 visual_move_target_x_, visual_move_target_y_, avg_err);

                        resetVisualAlignSample();
                    }

                    break;
                }

                if (visual_phase_ == VIS_MOVE) {
                    // 移动阶段不看实时视觉误差，只飞向上一轮平均误差计算出的固定目标点
                    target_x = visual_move_target_x_;
                    target_y = visual_move_target_y_;

                    double move_err = getDistanceXY(visual_move_target_x_, visual_move_target_y_);
                    if (move_err < VIS_MOVE_REACH_TOL) {
                        visual_phase_ = VIS_SETTLE;
                        visual_phase_ticks_ = 0;
                        ROS_INFO(">>> VIS_MOVE reached. Settle before next sampling. err=%.3f <<<", move_err);
                    } else if (++visual_phase_ticks_ > VIS_MOVE_TIMEOUT_TICKS) {
                        visual_phase_ = VIS_SETTLE;
                        visual_phase_ticks_ = 0;
                        ROS_WARN(">>> VIS_MOVE timeout. Force settle before next sampling. err=%.3f <<<", move_err);
                    }

                    break;
                }

                if (visual_phase_ == VIS_SETTLE) {
                    // 到点后固定悬停，等待位置环和机体稳定，再重新采样
                    target_x = visual_move_target_x_;
                    target_y = visual_move_target_y_;

                    if (++visual_phase_ticks_ > VIS_SETTLE_TICKS) {
                        // 使用实际当前位置作为下一轮悬停采样点，避免目标点和实际位置残差带入下一轮
                        visual_hold_x_ = current_pose_.pose.position.x;
                        visual_hold_y_ = current_pose_.pose.position.y;

                        resetVisualAlignSample();
                        visual_phase_ticks_ = 0;
                        visual_phase_ = VIS_SAMPLE;

                        ROS_INFO(">>> VIS_SETTLE done. Restart visual sampling at hold=(%.3f, %.3f) <<<",
                                 visual_hold_x_, visual_hold_y_);
                    }

                    break;
                }

                break;
            }

  
            case WAITING_FOR_STEADY:{
                target_x = lock_x_;
                target_y = lock_y_;
                target_z = search_point_.z+ home_z_;
                
                if (getDistanceXY(target_x, target_y) < 0.05) {
                    if (++hover_ticks_ > 30) {
                        // ROS_INFO("Hovering at Search Area. Waiting for camera...");
                        switchState(DESCEND_TO_GRAB);
                        break;
                    }
                } else hover_ticks_ = 0;

                break;
            }

            case DESCEND_TO_GRAB:
                target_x = lock_x_;
                target_y = lock_y_;
                target_z = drop_point_.z + home_z_;

                err_ticks_++;

                static bool grab_triggered = false; 
                if (!grab_triggered && std::abs(target_z - current_pose_.pose.position.z) < POSITION_TOLERANCE) {
                    triggerGrabMechanism();
                    grab_triggered = true;
                }

                if (std::abs(target_z - current_pose_.pose.position.z) < 0.1 && getDistanceXY(lock_x_, lock_y_) < 0.08){
                        hover_ticks_++;
                    }
                    else 
                    {
                        hover_ticks_ = 0;
                    }

                    if (hover_ticks_ > 80 || err_ticks_ > 220)
                    {
                        ROS_INFO("Grab sequence completed (Reached: %s). Ascending...", (err_ticks_ > 220) ? "TIMEOUT" : "SUCCESS");
                        ROS_INFO("use time %d", err_ticks_);

                        object_detected_ = false; 
                        grab_triggered = false;
                        switchState(ASCEND_TO_CRUISE); 
                        break;
                    }
                break;


            case ASCEND_TO_CRUISE:
                target_x = lock_x_;
                target_y = lock_y_;
                target_z = drop_point_.z + home_z_ + 0.35;
                if (std::abs(target_z - current_pose_.pose.position.z) < POSITION_TOLERANCE) {
                    ROS_INFO("Back to cruise height. Heading to Drop Point.");
                    switchState(NAVIGATE_DROP);
                }
                break;

            case NAVIGATE_DROP: {
                target_x = drop_point_.x + home_x_;
                target_y = drop_point_.y + home_y_;
                target_z = drop_point_.z + home_z_ + 0.35;
                if (getDistanceXY(target_x, target_y) < POSITION_TOLERANCE) {
                    if (++hover_ticks_ > 20) {
                        ROS_INFO("Arrived and stabilized at Drop Point.");
                        switchState(DROP_OBJECT);
                    }
                } else hover_ticks_ = 0;
                break;
            }

            case DROP_OBJECT: {
                target_x = drop_point_.x + home_x_;
                target_y = drop_point_.y + home_y_;
                target_z = 0.5 + home_z_;
                if(std::abs(target_z - current_pose_.pose.position.z) < POSITION_TOLERANCE) {
                    if(++hover_ticks_ == 19) { ROS_INFO("Drop engaged"); fool_check = true; }
                    if(hover_ticks_ > 20) { ROS_INFO("Drop finish"); switchState(ASCEND_TO_DROP); }
                } else hover_ticks_ = 0;

                if (fool_check) {
                    std_msgs::Bool trigger_msg; trigger_msg.data = true;
                    gpio_trigger_pub_.publish(trigger_msg);
                    ROS_INFO("[Hardware] Power ON! Demagnetizing to drop...");
                    fool_check = false;
                }
                break;
            }

            case ASCEND_TO_DROP: {
                target_x = drop_point_.x + home_x_;
                target_y = drop_point_.y + home_y_;
                target_z = search_point_.z + home_z_;

                if (std::abs(target_z - current_pose_.pose.position.z) < POSITION_TOLERANCE) {
                    if(++hover_ticks_ > 10) {
                        ROS_INFO("Returning home");
                        switchState(RETURN_HOME);
                        switch2Land();
                    }
                } else hover_ticks_ = 0;
                break;
            }

            case RETURN_HOME:
                target_x = home_x_;
                target_y = home_y_;
                target_z = search_point_.z + home_z_;
                if (getDistanceXY(home_x_, home_y_) < POSITION_TOLERANCE) {
                    if (++hover_ticks_ > 20) { ROS_INFO("Arrived Home. Landing..."); switchState(LAND); }
                } else hover_ticks_ = 0;
                break;

            case LAND:
                target_x = home_x_;
                target_y = home_y_;
                target_z = -0.2 + home_z_;
                
                if (current_pose_.pose.position.z - home_z_ < 0.05) {
                    if (++hover_ticks_ > 1) {
                        mavros_msgs::SetMode land_mode;
                        land_mode.request.custom_mode = "AUTO.LAND";
                        if (set_mode_client_.call(land_mode) && land_mode.response.mode_sent) {
                            ROS_INFO(">>> Descending finished. Switching to AUTO.LAND for auto-disarm. <<<");
                            switchState(FINISH);
                            landdone();
                        } else {
                            ROS_WARN_THROTTLE(1.0, "Wait for AUTO.LAND mode accept...");
                        }
                    }
                } else {
                    hover_ticks_ = 0; 
                }
                break;

            case FINISH:
                target_x = home_x_;
                target_y = home_y_;
                target_z = 0.0;
                break;
        }


        if (!sp_init_ && home_recorded_ && current_state_ == TAKEOFF) {
            smooth_x_.reset(current_pose_.pose.position.x);
            smooth_y_.reset(current_pose_.pose.position.y);
            smooth_z_.reset(current_pose_.pose.position.z);
            sp_init_ = true;
        }

        if (sp_init_) {
            if (current_state_ == VISUAL_ALIGN) {

                // 视觉对准阶段：目标点仍然来自离散采样状态机
                // 但输出给 PX4 前经过 Axis 平滑，避免单次修正目标跳变太猛
                smooth_x_.max_v = 0.15;
                smooth_y_.max_v = 0.15;
                smooth_x_.max_a = 0.25;
                smooth_y_.max_a = 0.25;

                smooth_x_.update(target_x, DT);
                smooth_y_.update(target_y, DT);

                sp_x_ = smooth_x_.p;
                sp_y_ = smooth_y_.p;

                // // Z 仍然保留平滑
                smooth_z_.max_v = MAX_Z_VEL;
                smooth_z_.max_a = 0.2;
                smooth_z_.update(target_z, DT);
                sp_z_ = smooth_z_.p;




                // // 抓取下降：XY 直接锁死到视觉锁定点
                // sp_x_ = target_x;
                // sp_y_ = target_y;
                // // Z 直接透传 target_z，不再经过 AxisSmoother
                // sp_z_ = target_z;
                // smooth_x_.reset(target_x);
                // smooth_y_.reset(target_y);
                // smooth_z_.reset(target_z);
            } else if (current_state_ == WAITING_FOR_STEADY) {

                // 视觉锁点后等待稳定：XY 直接锁到 lock 点
                sp_x_ = lock_x_;
                sp_y_ = lock_y_;

                // 高度保持搜索高度
                smooth_z_.max_v = MAX_Z_VEL;
                smooth_z_.max_a = 0.2;
                smooth_z_.update(target_z, DT);
                sp_z_ = smooth_z_.p;

                // 同步 XY smoother
                smooth_x_.reset(sp_x_);
                smooth_y_.reset(sp_y_);




                // // 抓取下降：XY 直接锁死到视觉锁定点
                // sp_x_ = target_x;
                // sp_y_ = target_y;
                // // Z 直接透传 target_z，不再经过 AxisSmoother
                // sp_z_ = target_z;
                // smooth_x_.reset(target_x);
                // smooth_y_.reset(target_y);
                // smooth_z_.reset(target_z);

            } else if (current_state_ == DESCEND_TO_GRAB) {

                // // 抓取下降：XY 直接锁死到视觉锁定点
                // sp_x_ = target_x;
                // sp_y_ = target_y;
                // // Z 直接透传 target_z，不再经过 AxisSmoother
                // sp_z_ = target_z;
                // smooth_x_.reset(target_x);
                // smooth_y_.reset(target_y);
                // smooth_z_.reset(target_z);



                // 同步 z smoother，避免切到 ASCEND_TO_CRUISE 后 smooth_z_ 还停在旧位置
                // 视觉对准阶段：XY 使用离散采样状态机给出的固定目标点，不再做实时视觉追点
                sp_x_ = lock_x_;
                sp_y_ = lock_y_;

                // Z 仍然保留平滑，防止高度目标突变
                smooth_z_.max_v = 0.08;
                smooth_z_.max_a = 10000;
                smooth_z_.update(target_z, DT, 1.8);
                sp_z_ = smooth_z_.p;

                // 同步 XY smoother，避免离开视觉状态后 smoother 仍停留在旧位置
                smooth_x_.reset(sp_x_);
                smooth_y_.reset(sp_y_);
                

            } else if (current_state_ == LAND) {

                // 回家降落：XY 直接锁死到 home 点
                smooth_x_.reset(home_x_);
                smooth_y_.reset(home_y_);

                sp_x_ = home_x_;
                sp_y_ = home_y_;

                smooth_z_.max_v = 0.3;
                smooth_z_.max_a = 0.2;
                smooth_z_.update(target_z, DT);
                sp_z_ = smooth_z_.p;



                // // 抓取下降：XY 直接锁死到视觉锁定点
                // sp_x_ = target_x;
                // sp_y_ = target_y;
                // // Z 直接透传 target_z，不再经过 AxisSmoother
                // sp_z_ = target_z;
                // smooth_x_.reset(target_x);
                // smooth_y_.reset(target_y);
                // smooth_z_.reset(target_z);

            } else {

                // 其他阶段正常平滑
                smooth_x_.max_v = MAX_XY_VEL;
                smooth_y_.max_v = MAX_XY_VEL;
                smooth_x_.max_a = MAX_ACC_XY;
                smooth_y_.max_a = MAX_ACC_XY;
                smooth_z_.max_v = MAX_Z_VEL;
                smooth_z_.max_a = 0.2;

                smooth_x_.update(target_x, DT);
                smooth_y_.update(target_y, DT);
                smooth_z_.update(target_z, DT);

                sp_x_ = smooth_x_.p;
                sp_y_ = smooth_y_.p;
                sp_z_ = smooth_z_.p;




                // // 抓取下降：XY 直接锁死到视觉锁定点
                // sp_x_ = target_x;
                // sp_y_ = target_y;
                // // Z 直接透传 target_z，不再经过 AxisSmoother
                // sp_z_ = target_z;
                // smooth_x_.reset(target_x);
                // smooth_y_.reset(target_y);
                // smooth_z_.reset(target_z);
            }
        } else {
            sp_x_ = current_pose_.pose.position.x;
            sp_y_ = current_pose_.pose.position.y;
            sp_z_ = current_pose_.pose.position.z;
        }
   
        mavros_msgs::PositionTarget target_raw;
        target_raw.header.stamp = ros::Time::now();
        target_raw.header.frame_id = "map";

        target_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

       
        target_raw.type_mask =
                mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        double vx_ff = 0.0;
        double vy_ff = 0.0;
        double vz_ff = 0.0;

        if (sp_init_) {
            vx_ff = smooth_x_.v;
            vy_ff = smooth_y_.v;
            vz_ff = smooth_z_.v;
        }

        if (current_state_ == DESCEND_TO_GRAB) {
            double ex = lock_x_ - current_pose_.pose.position.x;
            double ey = lock_y_ - current_pose_.pose.position.y;

            if (!descend_pid_init_) {
                descend_pid_prev_ex_ = ex;
                descend_pid_prev_ey_ = ey;
                descend_pid_init_ = true;
            }

            descend_pid_ix_ = clampValue(descend_pid_ix_ + ex * DT,
                                         -DESCEND_XY_PID_I_MAX,
                                          DESCEND_XY_PID_I_MAX);
            descend_pid_iy_ = clampValue(descend_pid_iy_ + ey * DT,
                                         -DESCEND_XY_PID_I_MAX,
                                          DESCEND_XY_PID_I_MAX);

            double dex = (ex - descend_pid_prev_ex_) / DT;
            double dey = (ey - descend_pid_prev_ey_) / DT;

            vx_ff = DESCEND_XY_PID_KP * ex +
                    DESCEND_XY_PID_KI * descend_pid_ix_ +
                    DESCEND_XY_PID_KD * dex;

            vy_ff = DESCEND_XY_PID_KP * ey +
                    DESCEND_XY_PID_KI * descend_pid_iy_ +
                    DESCEND_XY_PID_KD * dey;

            vx_ff = clampValue(vx_ff, -DESCEND_XY_PID_VEL_MAX, DESCEND_XY_PID_VEL_MAX);
            vy_ff = clampValue(vy_ff, -DESCEND_XY_PID_VEL_MAX, DESCEND_XY_PID_VEL_MAX);

            
            vz_ff = smooth_z_.v;

            descend_pid_prev_ex_ = ex;
            descend_pid_prev_ey_ = ey;

            // ROS_INFO_THROTTLE(
            //     1,
            //     "[DESCEND XY PID FF] err=(%.3f, %.3f), v_ff=(%.3f, %.3f, %.3f)",
            //     ex, ey, vx_ff, vy_ff, vz_ff
            // );
        }

        target_raw.position.x = sp_x_;
        target_raw.position.y = sp_y_;
        target_raw.position.z = sp_z_;

        target_raw.velocity.x = vx_ff;
        target_raw.velocity.y = vy_ff;
        target_raw.velocity.z = vz_ff;

        target_raw.acceleration_or_force.x = 0.0;
        target_raw.acceleration_or_force.y = 0.0;
        target_raw.acceleration_or_force.z = 0.0;

        if (yaw_recorded_) {
            target_raw.yaw = initial_yaw_;
        } else {
            target_raw.yaw = 0.0;
        }

        cmd_pose_pub_.publish(target_raw);
    } 

    double getDistanceXY(double t_x, double t_y) {
        return std::sqrt(std::pow(t_x - current_pose_.pose.position.x, 2) + 
                         std::pow(t_y - current_pose_.pose.position.y, 2));
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "logistics_uav_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    LogisticsMissionController controller;
    ros::waitForShutdown();
    return 0;
}
