

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace {

constexpr double CONTROL_DT = 0.05;  // 20 Hz

constexpr int HOME_FILTER_DELAY_FRAMES = 200;
constexpr int HOME_SAMPLE_COUNT = 50;
constexpr int TAKEOFF_HOLD_TICKS = 20;
constexpr int WAYPOINT_HOLD_TICKS = 30;
constexpr int RETURN_HOLD_TICKS = 20;

constexpr double DEFAULT_CRUISE_HEIGHT = 0.8;
constexpr double LAND_TARGET_REL_Z = -0.20;

constexpr double POSITION_TOLERANCE = 0.10;

constexpr double MAX_XY_VEL = 0.20;
constexpr double MAX_Z_VEL = 0.40;
constexpr double MAX_XY_ACC = 0.40;
constexpr double MAX_Z_ACC = 0.20;

constexpr double LAND_MAX_Z_VEL = 0.30;
constexpr double MODE_REQUEST_INTERVAL = 5.0;

}  // namespace

enum class MissionState {
    WAIT_HOME,
    WAIT_OFFBOARD_ARM,
    TAKEOFF,
    FLY_WAYPOINTS,
    RETURN_HOME,
    LAND,
    FINISH
};

struct AxisSmoother {
    double p = 0.0;
    double v = 0.0;
    double max_v = 0.0;
    double max_a = 0.0;

    AxisSmoother(double velocity_limit, double acceleration_limit)
        : max_v(velocity_limit), max_a(acceleration_limit) {}

    void reset(double current_position) {
        p = current_position;
        v = 0.0;
    }

    void update(double target_position, double dt, double kp = 0.6) {
        const double error = target_position - p;

        double desired_velocity = kp * error;
        desired_velocity = std::max(
            -max_v,
            std::min(max_v, desired_velocity)
        );

        double delta_velocity = desired_velocity - v;
        const double max_delta_velocity = max_a * dt;
        delta_velocity = std::max(
            -max_delta_velocity,
            std::min(max_delta_velocity, delta_velocity)
        );

        v += delta_velocity;
        p += v * dt;
    }
};

class MultiWaypointController {
public:
    MultiWaypointController()
        : nh_("~"),
          smooth_x_(MAX_XY_VEL, MAX_XY_ACC),
          smooth_y_(MAX_XY_VEL, MAX_XY_ACC),
          smooth_z_(MAX_Z_VEL, MAX_Z_ACC) {
        initializeWaypoints();
        initializeROS();

        ros::Duration(1.0).sleep();

        control_timer_ = nh_.createTimer(
            ros::Duration(CONTROL_DT),
            &MultiWaypointController::controlLoop,
            this
        );

        ROS_INFO("==================================================");
        ROS_INFO("Simple multi-waypoint controller started.");
        ROS_INFO("Waypoint coordinates are relative to the home point.");
        ROS_INFO("Loaded %zu waypoint(s).", waypoints_.size());
        printWaypoints();
        ROS_INFO("==================================================");
    }

private:
    ros::NodeHandle nh_;
    ros::Timer control_timer_;

    ros::Subscriber state_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber emergency_sub_;

    ros::Publisher setpoint_pub_;

    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    mavros_msgs::State mavros_state_;
    geometry_msgs::PoseStamped current_pose_;

    MissionState state_ = MissionState::WAIT_HOME;

    std::vector<geometry_msgs::Point> waypoints_;
    std::size_t waypoint_index_ = 0;

    bool pose_received_ = false;
    bool home_recorded_ = false;
    bool smoother_initialized_ = false;

    double home_x_ = 0.0;
    double home_y_ = 0.0;
    double home_z_ = 0.0;
    double home_yaw_ = 0.0;

    int home_filter_delay_count_ = 0;
    int home_sample_count_ = 0;
    double home_sum_x_ = 0.0;
    double home_sum_y_ = 0.0;
    double home_sum_z_ = 0.0;
    double home_sum_sin_yaw_ = 0.0;
    double home_sum_cos_yaw_ = 0.0;

    int state_ticks_ = 0;
    int hover_ticks_ = 0;

    ros::Time last_mode_request_time_;

    AxisSmoother smooth_x_;
    AxisSmoother smooth_y_;
    AxisSmoother smooth_z_;

    void initializeWaypoints() {
        /*
         * Modify the waypoint list here.
         *
         * Each point is relative to the recorded takeoff point:
         *   {x, y, z}
         *
         * Example below:
         *   1. Fly 1 m in +X at 1.2 m height
         *   2. Fly to (1, 1) at 1.2 m height
         *   3. Fly to (0, 1) at 1.2 m height
         *
         * Returning to (0, 0) is handled automatically by RETURN_HOME,
         * so it does not need to be added as the final waypoint.
         */
        geometry_msgs::Point point;

        point.x = 1.5;
        point.y = 0.0;
        point.z = 0.8;
        waypoints_.push_back(point);

        point.x = 1.5;
        point.y = 1.0;
        point.z = 0.8;
        waypoints_.push_back(point);

        point.x = 0;
        point.y = 1.0;
        point.z = 0.8;
        waypoints_.push_back(point);
    }

    void initializeROS() {
        state_sub_ = nh_.subscribe<mavros_msgs::State>(
            "/mavros/state",
            10,
            &MultiWaypointController::stateCallback,
            this
        );

        pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
            "/mavros/local_position/pose",
            10,
            &MultiWaypointController::poseCallback,
            this
        );

        emergency_sub_ = nh_.subscribe<std_msgs::Bool>(
            "/mission/rtl",
            10,
            &MultiWaypointController::emergencyCallback,
            this
        );

        setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
            "/mavros/setpoint_raw/local",
            10
        );

        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
            "/mavros/cmd/arming"
        );

        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
            "/mavros/set_mode"
        );

        last_mode_request_time_ = ros::Time::now();
    }

    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        mavros_state_ = *msg;
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_pose_ = *msg;
        pose_received_ = true;

        if (home_recorded_) {
            return;
        }

        const double x = msg->pose.position.x;
        const double y = msg->pose.position.y;
        const double z = msg->pose.position.z;

        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            ROS_WARN_THROTTLE(1.0, "Invalid local-position sample. Ignoring it.");
            return;
        }

        if (z <= -0.5) {
            ROS_WARN_THROTTLE(
                1.0,
                "Local Z is not valid for home filtering yet: z=%.3f",
                z
            );
            return;
        }

        ++home_filter_delay_count_;
        if (home_filter_delay_count_ < HOME_FILTER_DELAY_FRAMES) {
            ROS_INFO_THROTTLE(
                1.0,
                "Waiting before home sampling: %d/%d pose frames",
                home_filter_delay_count_,
                HOME_FILTER_DELAY_FRAMES
            );
            return;
        }

        tf::Quaternion quaternion(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
        );

        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

        home_sum_x_ += x;
        home_sum_y_ += y;
        home_sum_z_ += z;
        home_sum_sin_yaw_ += std::sin(yaw);
        home_sum_cos_yaw_ += std::cos(yaw);
        ++home_sample_count_;

        ROS_INFO_THROTTLE(
            1.0,
            "Collecting home samples: %d/%d",
            home_sample_count_,
            HOME_SAMPLE_COUNT
        );

        if (home_sample_count_ >= HOME_SAMPLE_COUNT) {
            home_x_ = home_sum_x_ / static_cast<double>(home_sample_count_);
            home_y_ = home_sum_y_ / static_cast<double>(home_sample_count_);
            home_z_ = home_sum_z_ / static_cast<double>(home_sample_count_);
            home_yaw_ = std::atan2(home_sum_sin_yaw_, home_sum_cos_yaw_);

            home_recorded_ = true;

            ROS_WARN(
                "Home locked: x=%.3f, y=%.3f, z=%.3f, yaw=%.3f rad",
                home_x_,
                home_y_,
                home_z_,
                home_yaw_
            );
        }
    }

    void emergencyCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (!msg->data) {
            return;
        }

        switch (state_) {
            case MissionState::TAKEOFF:
            case MissionState::FLY_WAYPOINTS:
                ROS_WARN("Emergency RTL requested. Returning home.");
                switchState(MissionState::RETURN_HOME);
                break;

            case MissionState::RETURN_HOME:
            case MissionState::LAND:
            case MissionState::FINISH:
                ROS_WARN("RTL request received, but vehicle is already returning/landing.");
                break;

            default:
                ROS_WARN("RTL request ignored because mission flight has not started.");
                break;
        }
    }

    void controlLoop(const ros::TimerEvent&) {
        if (!mavros_state_.connected) {
            ROS_WARN_THROTTLE(1.0, "Waiting for FCU connection...");
            return;
        }

        if (!pose_received_) {
            ROS_WARN_THROTTLE(1.0, "Waiting for /mavros/local_position/pose...");
            return;
        }

        double target_x = current_pose_.pose.position.x;
        double target_y = current_pose_.pose.position.y;
        double target_z = current_pose_.pose.position.z;

        switch (state_) {
            case MissionState::WAIT_HOME:
                target_x = current_pose_.pose.position.x;
                target_y = current_pose_.pose.position.y;
                target_z = current_pose_.pose.position.z;

                if (home_recorded_) {
                    ROS_INFO(
                        "Home initialization complete. Holding home and waiting "
                        "before requesting OFFBOARD."
                    );
                    switchState(MissionState::WAIT_OFFBOARD_ARM);
                }
                break;

            case MissionState::WAIT_OFFBOARD_ARM:
                target_x = home_x_;
                target_y = home_y_;
                target_z = home_z_;

                requestOffboardAndArm();

                if (mavros_state_.mode == "OFFBOARD" && mavros_state_.armed) {
                    ROS_WARN("OFFBOARD active and vehicle armed. Starting takeoff.");
                    switchState(MissionState::TAKEOFF);
                }
                break;

            case MissionState::TAKEOFF:
                target_x = home_x_;
                target_y = home_y_;
                target_z = home_z_ + missionCruiseHeight();

                if (hasReached(target_x, target_y, target_z)) {
                    ++hover_ticks_;
                    if (hover_ticks_ > TAKEOFF_HOLD_TICKS) {
                        waypoint_index_ = 0;

                        if (waypoints_.empty()) {
                            ROS_WARN("No waypoints configured. Returning home.");
                            switchState(MissionState::RETURN_HOME);
                        } else {
                            ROS_INFO(
                                "Takeoff complete. Flying to waypoint 1/%zu.",
                                waypoints_.size()
                            );
                            switchState(MissionState::FLY_WAYPOINTS);
                        }
                    }
                } else {
                    hover_ticks_ = 0;
                }
                break;

            case MissionState::FLY_WAYPOINTS: {
                if (waypoint_index_ >= waypoints_.size()) {
                    ROS_INFO("All waypoints completed. Returning home.");
                    switchState(MissionState::RETURN_HOME);
                    break;
                }

                const geometry_msgs::Point& waypoint = waypoints_[waypoint_index_];

                target_x = home_x_ + waypoint.x;
                target_y = home_y_ + waypoint.y;
                target_z = home_z_ + waypoint.z;

                ROS_INFO_THROTTLE(
                    1.0,
                    "Flying waypoint %zu/%zu | target_rel=(%.2f, %.2f, %.2f) "
                    "| error_xy=%.2f, error_z=%.2f",
                    waypoint_index_ + 1,
                    waypoints_.size(),
                    waypoint.x,
                    waypoint.y,
                    waypoint.z,
                    distanceXY(target_x, target_y),
                    std::abs(target_z - current_pose_.pose.position.z)
                );

                if (hasReached(target_x, target_y, target_z)) {
                    ++hover_ticks_;

                    if (hover_ticks_ > WAYPOINT_HOLD_TICKS) {
                        ROS_INFO(
                            "Waypoint %zu/%zu completed.",
                            waypoint_index_ + 1,
                            waypoints_.size()
                        );

                        ++waypoint_index_;
                        hover_ticks_ = 0;

                        if (waypoint_index_ >= waypoints_.size()) {
                            ROS_INFO("All waypoints completed. Returning home.");
                            switchState(MissionState::RETURN_HOME);
                        } else {
                            ROS_INFO(
                                "Flying to waypoint %zu/%zu.",
                                waypoint_index_ + 1,
                                waypoints_.size()
                            );
                        }
                    }
                } else {
                    hover_ticks_ = 0;
                }
                break;
            }

            case MissionState::RETURN_HOME:
                target_x = home_x_;
                target_y = home_y_;
                target_z = home_z_ + missionCruiseHeight();

                ROS_INFO_THROTTLE(
                    1.0,
                    "Returning home | error_xy=%.2f, error_z=%.2f",
                    distanceXY(target_x, target_y),
                    std::abs(target_z - current_pose_.pose.position.z)
                );

                if (hasReached(target_x, target_y, target_z)) {
                    ++hover_ticks_;

                    if (hover_ticks_ > RETURN_HOLD_TICKS) {
                        ROS_INFO("Home reached. Starting controlled descent.");
                        switchState(MissionState::LAND);
                    }
                } else {
                    hover_ticks_ = 0;
                }
                break;

            case MissionState::LAND:
                target_x = home_x_;
                target_y = home_y_;
                target_z = home_z_ + LAND_TARGET_REL_Z;

                if ((current_pose_.pose.position.z - home_z_) < 0.05) {
                    ++hover_ticks_;

                    if (hover_ticks_ > 1) {
                        requestAutoLand();
                    }
                } else {
                    hover_ticks_ = 0;
                }
                break;

            case MissionState::FINISH:
                target_x = home_x_;
                target_y = home_y_;
                target_z = home_z_;

                if (!mavros_state_.armed) {
                    ROS_INFO_THROTTLE(2.0, "Mission finished. Vehicle disarmed.");
                }
                break;
        }

        if (!smoother_initialized_ &&
            home_recorded_ &&
            state_ == MissionState::TAKEOFF) {
            smooth_x_.reset(current_pose_.pose.position.x);
            smooth_y_.reset(current_pose_.pose.position.y);
            smooth_z_.reset(current_pose_.pose.position.z);
            smoother_initialized_ = true;
        }

        updateAndPublishSetpoint(target_x, target_y, target_z);
    }

    void updateAndPublishSetpoint(
        double target_x,
        double target_y,
        double target_z
    ) {
        if (!smoother_initialized_) {
            publishRawSetpoint(
                current_pose_.pose.position.x,
                current_pose_.pose.position.y,
                current_pose_.pose.position.z,
                0.0,
                0.0,
                0.0
            );
            return;
        }

        smooth_x_.max_v = MAX_XY_VEL;
        smooth_y_.max_v = MAX_XY_VEL;
        smooth_x_.max_a = MAX_XY_ACC;
        smooth_y_.max_a = MAX_XY_ACC;

        smooth_z_.max_a = MAX_Z_ACC;
        smooth_z_.max_v =
            (state_ == MissionState::LAND) ? LAND_MAX_Z_VEL : MAX_Z_VEL;

        smooth_x_.update(target_x, CONTROL_DT);
        smooth_y_.update(target_y, CONTROL_DT);
        smooth_z_.update(target_z, CONTROL_DT);

        publishRawSetpoint(
            smooth_x_.p,
            smooth_y_.p,
            smooth_z_.p,
            smooth_x_.v,
            smooth_y_.v,
            smooth_z_.v
        );
    }

    void publishRawSetpoint(
        double position_x,
        double position_y,
        double position_z,
        double velocity_x,
        double velocity_y,
        double velocity_z
    ) {
        mavros_msgs::PositionTarget setpoint;
        setpoint.header.stamp = ros::Time::now();
        setpoint.header.frame_id = "map";

        setpoint.coordinate_frame =
            mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

        setpoint.type_mask =
            mavros_msgs::PositionTarget::IGNORE_AFX |
            mavros_msgs::PositionTarget::IGNORE_AFY |
            mavros_msgs::PositionTarget::IGNORE_AFZ |
            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        setpoint.position.x = position_x;
        setpoint.position.y = position_y;
        setpoint.position.z = position_z;

        setpoint.velocity.x = velocity_x;
        setpoint.velocity.y = velocity_y;
        setpoint.velocity.z = velocity_z;

        setpoint.acceleration_or_force.x = 0.0;
        setpoint.acceleration_or_force.y = 0.0;
        setpoint.acceleration_or_force.z = 0.0;

        setpoint.yaw = home_recorded_ ? home_yaw_ : 0.0;
        setpoint.yaw_rate = 0.0;

        setpoint_pub_.publish(setpoint);
    }

    void requestOffboardAndArm() {
        const ros::Time now = ros::Time::now();

        if ((now - last_mode_request_time_).toSec() < MODE_REQUEST_INTERVAL) {
            return;
        }

        last_mode_request_time_ = now;

        if (mavros_state_.mode != "OFFBOARD") {
            mavros_msgs::SetMode mode_request;
            mode_request.request.custom_mode = "OFFBOARD";

            if (set_mode_client_.call(mode_request) &&
                mode_request.response.mode_sent) {
                ROS_INFO("OFFBOARD mode request sent successfully.");
            } else {
                ROS_WARN("OFFBOARD mode request failed.");
            }
            return;
        }

        if (!mavros_state_.armed) {
            mavros_msgs::CommandBool arm_request;
            arm_request.request.value = true;

            if (arming_client_.call(arm_request) &&
                arm_request.response.success) {
                ROS_INFO("Arming request accepted.");
            } else {
                ROS_WARN("Arming request failed.");
            }
        }
    }

    void requestAutoLand() {
        mavros_msgs::SetMode land_request;
        land_request.request.custom_mode = "AUTO.LAND";

        if (set_mode_client_.call(land_request) &&
            land_request.response.mode_sent) {
            ROS_WARN("AUTO.LAND accepted. Waiting for automatic disarm.");
            switchState(MissionState::FINISH);
        } else {
            ROS_WARN("AUTO.LAND request failed. Continuing to hold descent setpoint.");
        }
    }

    double missionCruiseHeight() const {
        if (waypoints_.empty()) {
            return DEFAULT_CRUISE_HEIGHT;
        }
        return waypoints_.front().z;
    }

    bool hasReached(double target_x, double target_y, double target_z) const {
        return distanceXY(target_x, target_y) < POSITION_TOLERANCE &&
               std::abs(target_z - current_pose_.pose.position.z) <
                   POSITION_TOLERANCE;
    }

    double distanceXY(double target_x, double target_y) const {
        const double dx = target_x - current_pose_.pose.position.x;
        const double dy = target_y - current_pose_.pose.position.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    void switchState(MissionState new_state) {
        state_ = new_state;
        state_ticks_ = 0;
        hover_ticks_ = 0;
        last_mode_request_time_ = ros::Time::now();

        ROS_INFO("State changed to: %s", stateName(new_state));
    }

    const char* stateName(MissionState state) const {
        switch (state) {
            case MissionState::WAIT_HOME:
                return "WAIT_HOME";
            case MissionState::WAIT_OFFBOARD_ARM:
                return "WAIT_OFFBOARD_ARM";
            case MissionState::TAKEOFF:
                return "TAKEOFF";
            case MissionState::FLY_WAYPOINTS:
                return "FLY_WAYPOINTS";
            case MissionState::RETURN_HOME:
                return "RETURN_HOME";
            case MissionState::LAND:
                return "LAND";
            case MissionState::FINISH:
                return "FINISH";
        }

        return "UNKNOWN";
    }

    void printWaypoints() const {
        for (std::size_t i = 0; i < waypoints_.size(); ++i) {
            ROS_INFO(
                "Waypoint %zu: relative (x=%.2f, y=%.2f, z=%.2f)",
                i + 1,
                waypoints_[i].x,
                waypoints_[i].y,
                waypoints_[i].z
            );
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_waypoint_uav_node");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    MultiWaypointController controller;

    ros::waitForShutdown();
    return 0;
}
