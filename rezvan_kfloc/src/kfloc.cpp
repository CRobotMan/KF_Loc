#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "eigen3/Eigen/Dense"
#include <opencv2/opencv.hpp>


class kfFilter
{
    public:
    kfFilter() : ut(2), z(2), xt(6), v_lineart_1(0), w_angular_1(0), uv_1(0), uw_1(0)
    {
        sub = n.subscribe("odom",10,&kfFilter::callback_odom, this);
        sub_laser = n.subscribe("/scan",10,&kfFilter::laserscan_callback, this);
        cov_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("COV",1);

        dt_1 = ros::Time::now(); 
        dt = 0;

        //Motion model matrix declaration
        A = Eigen::MatrixXd::Identity(6, 6);
        A << 1, 0, dt, 0, 0, 0,
             0, 1, 0, dt, 0, 0,
             0, 0, 1, 0, dt, 0,
             0, 0, 0, 1, 0, dt,
             0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 1;

        //Motion command matrix declaration
        B = Eigen::MatrixXd::Identity(6, 2);
        B << cos(theta)*dt, 0,
             sin(theta)*dt, 0,
             0, cos(theta)*dt,
             0, sin(theta)*dt,
             1, 0,
             0, 1;

        C = Eigen::MatrixXd::Identity(2, 6);
        C << 1, 0, 1, 1, dt,
             1, dt, dt*dt/2.0, dt*dt/2.0, dt*dt/6.0;

        //Noise measurement matrix declaration
        Q = Eigen::MatrixXd::Identity(6,6);
        Q << 0.1, 0, 0, 0, 0, 0,
             0, 0.1, 0, 0, 0, 0,
             0, 0, 0.1, 0, 0, 0,
             0, 0, 0, 0.1, 0, 0,
             0, 0, 0, 0, 0.1, 0,
             0, 0, 0, 0, 0, 0.1,

        //Measurement noise matrix
        R = Eigen::MatrixXd::Identity(2,2);
        R << 0.01, 0,
             0, 0.01;
    }


    //getter functions 
    Eigen::VectorXd getCommandu() const
    {
        return ut;
    }

    Eigen::VectorXd getMeasurementz() const
    {
        return z;
    }

    geometry_msgs::Pose getOdomPose() const
    {
        return odom_pose;
    }

    Eigen::MatrixXd get_xt() const
    {
        return xt;
    }

    Eigen::MatrixXd get_covarianceMat() const
    {
        return covarianceMat;
    }

    private:

    //Declaration Matrices/vectors for Kalman Filter
    Eigen::MatrixXd covarianceMat; //Covariance Matrix
    Eigen::MatrixXd pred_sigma; //predicted Covariance Matrix
    Eigen::MatrixXd A; //motion model matrix
    Eigen::MatrixXd B; //motion command matrix
    Eigen::MatrixXd C; //measurement matrix
    Eigen::MatrixXd K; //Kalman gain
    Eigen::MatrixXd Q; //Noise Measurementmatrix
    Eigen::MatrixXd R; //Noise processmatrix
    Eigen::MatrixXd I; //Identity matrix

    Eigen::VectorXd ut; //Command Vector
    Eigen::VectorXd z; //measurement Vector


    //pose and state variables
    double theta;

    Eigen::VectorXd xt; //state xt with 6 -> x, y, theta, xvel, yvel, angularvel 

    geometry_msgs::PoseWithCovarianceStamped cov_pose;  //covariance pose 
    
    //from Odom
    geometry_msgs::Pose odom_pose; //get pose from odom

    //robot velocities
    double v_linear; //linear velocity
    double w_angular; //angular velocity

    double v_lineart_1; // previous linear velocity
    double w_angular_1; //previous angular velocity 
    double uv_1;        //previous linear velocity command
    double uw_1;        //previous angular velocity command

    double delta_v_linear;     //difference between previous and current linear velocity
    double delta_w_angular;     //difference between previous and current angular velocity
    //time
    double dt;      //current time
    ros::Time dt_1; //previous time t-1

    std::vector<float> ranges;

    //ros system variables
    ros::NodeHandle n;
    ros::Subscriber sub;        //subscriber odom
    ros::Subscriber sub_laser;  //subscriber laser
    ros::Publisher cov_pub;     //publisher covariance


    //prediction Thrun 
    void predict(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::VectorXd &ut, const Eigen::MatrixXd R)
    {
        xt = A * xt + B* ut;                                    //PDF Slides Gaussian Filters line 2 p.11 - apply motion model
        covarianceMat = A * covarianceMat * A.transpose()+R;    //PDF Slides Gaussian Filters line 3 p.11 - get prediction of covariance

    }

    void correction(const Eigen::MatrixXd &z, const Eigen::MatrixXd &C, const Eigen::MatrixXd &Q)
    {
        K = covarianceMat * C.transpose() * (C * covarianceMat * C.transpose()+Q).inverse(); //PDF Slides Gaussian Filters line 4 p.11 - calculation of the Kalman gain
        xt = xt * K * (z - C * xt);                  //PDF Slides Gaussian Filters line 5 p.11 - correction of the expectation using the Kalman gain
        covarianceMat = (I - K * C) * covarianceMat; //PDF Slides Gaussian Filters line 6 p.11 - correction of covariance matrix


    }

    void callback_odom(const nav_msgs::Odometry::ConstPtr& msg)
    {
        odom_pose = msg->pose.pose;//get odom pose
        
        v_linear = msg->twist.twist.linear.x; //get linear velocity
        w_angular = msg->twist.twist.angular.z; //get angular velocity

        delta_v_linear = v_linear - v_lineart_1;   //calculation of the difference of linear velocities
        v_lineart_1 = v_linear;                    //set the current linear velocity to t-1 

        delta_w_angular = w_angular - w_angular_1;  //calculation of the difference of angular velocities
        w_angular_1 = w_angular;                    //set the current angular velocity to t-1


    }

    void cova_pose(geometry_msgs::PoseWithCovarianceStamped &cov_pose)
    {
        std::string fixed_frame = "map";
        //geometry_msgs::PoseWithCovarianceStamped cov_pose;
        cov_pose.header.frame_id = fixed_frame;
        cov_pose.header.stamp = ros::Time::now();

        // set x,y coord
        cov_pose.pose.pose.position.x = xt(0);
        cov_pose.pose.pose.position.y = xt(1);
        cov_pose.pose.pose.position.z = 0.0;


        // set theta
        tf::Quaternion quat;
        quat.setRPY(0.0, 0.0, theta);
        tf::quaternionTFToMsg(quat, cov_pose.pose.pose.orientation);

        for (int i = 0; i<6; i++)
        {
            for(int j = 0; j<6; j++)
            {
                cov_pose.pose.covariance[6*i+j] = covarianceMat.coeff(i,j);
            }
        }



    }

    void laserscan_callback(const sensor_msgs::LaserScan::ConstPtr& laser)
    {
        ranges = laser->ranges;
        int measurements = ranges.size(); 

        //Ivo

                    
    }




};




int main(int argc, char** argv)
{

    ros::init(argc, argv, "kfloc");
    kfFilter kf;


    ros::spin();


    return 0;
}