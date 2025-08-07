#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include <sstream>
#include <mutex>
#include <thread>
#include <ctime>

class DataLogger
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber gss_sub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber imu_sub_;

    std::mutex buffer_mutex_;
    std::vector<std::vector<std::string>> buffer_;

    double gss_linear_x_, gss_linear_y_, gss_linear_z_;
    double gss_angular_x_, gss_angular_y_, gss_angular_z_;
    double lat_, lon_, alt_;
    double ax_, ay_, az_;
    double roll_, pitch_, yaw_;
    double r_rate_, p_rate_, y_rate_;

    double initial_time_;
    std::string log_file_;
    ros::Rate rate_;

public:
    DataLogger() : rate_(100)
    {
        gss_sub_ = nh_.subscribe("/fsds/gss", 10, &DataLogger::gssCallback, this);
        gps_sub_ = nh_.subscribe("/fsds/gps", 10, &DataLogger::gpsCallback, this);
        imu_sub_ = nh_.subscribe("/fsds/imu", 10, &DataLogger::imuCallback, this);

        initial_time_ = ros::Time::now().toSec();

        std::time_t t = std::time(nullptr);
        std::tm *now = std::localtime(&t);
        std::ostringstream filename;
        filename << "/home/user/fsds_ws/src/tutorial/log/data_log_" 
                 << std::put_time(now, "%m%d%H%M") << ".csv";
        log_file_ = filename.str();

        std::ofstream file(log_file_);
        file << "time[sec],ax[m/s^2],ay[m/s^2],az[m/s^2],roll_rate[deg/s],pitch_rate[deg/s],yaw_rate[deg/s],"
             << "roll[deg],pitch[deg],yaw[deg],lat[deg],lon[deg],alt[m],"
             << "gss_linear_x[m/s],gss_linear_y[m/s],gss_linear_z[m/s],"
             << "gss_angular_x[rad/s],gss_angular_y[rad/s],gss_angular_z[rad/s]\n";
        file.close();

        std::thread(&DataLogger::writeDataLog, this).detach();
        ROS_INFO("DATA LOGGING");
    }

    void gssCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg)
    {
        gss_linear_x_ = msg->twist.twist.linear.x;
        gss_linear_y_ = msg->twist.twist.linear.y;
        gss_linear_z_ = msg->twist.twist.linear.z;

        gss_angular_x_ = msg->twist.twist.angular.x;
        gss_angular_y_ = msg->twist.twist.angular.y;
        gss_angular_z_ = msg->twist.twist.angular.z;
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        lat_ = msg->latitude;
        lon_ = msg->longitude;
        alt_ = msg->altitude;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        ax_ = msg->linear_acceleration.x;
        ay_ = msg->linear_acceleration.y;
        az_ = msg->linear_acceleration.z;

        tf::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        roll_ = roll * 180.0 / M_PI;
        pitch_ = pitch * 180.0 / M_PI;
        yaw_ = yaw * 180.0 / M_PI;

        r_rate_ = msg->angular_velocity.x;
        p_rate_ = msg->angular_velocity.y;
        y_rate_ = msg->angular_velocity.z;
    }

    void writeDataLog()
    {
        ros::Rate loop_rate(10); // Write every 0.1s
        while (ros::ok())
        {
            logData();
            loop_rate.sleep();
        }
    }

    void logData()
    {
        double t = ros::Time::now().toSec() - initial_time_;
        std::vector<std::string> data_row;
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(2) << t;
        data_row.push_back(ss.str());

        auto add_data = [&](double value, int precision = 5) {
            std::ostringstream s;
            s << std::fixed << std::setprecision(precision) << value;
            data_row.push_back(s.str());
        };

        add_data(ax_);
        add_data(ay_);
        add_data(az_);
        add_data(r_rate_);
        add_data(p_rate_);
        add_data(y_rate_);
        add_data(roll_);
        add_data(pitch_);
        add_data(yaw_);
        add_data(lat_, 8);
        add_data(lon_, 8);
        add_data(alt_, 3);
        add_data(gss_linear_x_);
        add_data(gss_linear_y_);
        add_data(gss_linear_z_);
        add_data(gss_angular_x_);
        add_data(gss_angular_y_);
        add_data(gss_angular_z_);

        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            buffer_.push_back(data_row);
        }

        flushBuffer();
    }

    void flushBuffer()
    {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        if (!buffer_.empty())
        {
            std::ofstream file(log_file_, std::ios::app);
            for (const auto &row : buffer_)
            {
                for (size_t i = 0; i < row.size(); ++i)
                {
                    file << row[i];
                    if (i != row.size() - 1)
                        file << ",";
                }
                file << "\n";
            }
            buffer_.clear();
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Data_Logger");
    DataLogger logger;
    ros::spin();
    return 0;
}
