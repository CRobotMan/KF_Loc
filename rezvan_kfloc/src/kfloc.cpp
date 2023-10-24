#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "eigen3/Eigen/Dense"
//#include <opencv2/opencv.hpp>

//: ut(2), z(2), xt(6), v_lineart_1(0), w_angular_1(0), uv_1(0), uw_1(0)


class kfFilter
{
    public:
    kfFilter() : ut(2), z(3), xt(6), v_lineart_1(0), w_angular_1(0), uv_1(0), uw_1(0)
    {
        // ut(2); 
        // z(2); 
        // xt(6);
        // v_lineart_1=0; 
        // w_angular_1=0; 
        // uv_1=0; 
        // uw_1=0;
        
        //ROS_INFO("Hier ist ein Konstruktor!");
        ros::NodeHandle n;
        sub = n.subscribe("odom",10,&kfFilter::callback_odom, this);
        sub_laser = n.subscribe("scan",10,&kfFilter::laserscan_callback, this);
        cov_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/covariance",10);
        pubkpose = n.advertise<geometry_msgs::PoseWithCovarianceStamped> ("/kfpose", 10);

        prevt_1 = ros::Time::now(); 
        dt = 0;

        //Motion model matrix declaration
        A = Eigen::MatrixXd::Identity(6, 6);
        A << 1, 0, 0, dt, 0, 0,
             0, 1, 0, 0, dt, 0,
             0, 0, 1, 0, 0, dt,
             0, 0, 0, 1, 0, 0,
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
        C << 1, 0, 0, 1, dt, 0,
             1, dt, 0, dt*dt/2.0, dt*dt/2.0, dt*dt/6.0;

        //Noise measurement matrix declaration
        Q = Eigen::MatrixXd::Identity(6,6);
        Q << 0.1, 0, 0, 0, 0, 0,
             0, 0.1, 0, 0, 0, 0,
             0, 0, 0.1, 0, 0, 0,
             0, 0, 0, 0.1, 0, 0,
             0, 0, 0, 0, 0.1, 0,
             0, 0, 0, 0, 0, 0.1;

        //CovarianceMatrix
        covarianceMat = Eigen::MatrixXd::Identity(6,6);
        covarianceMat << 0.1, 0, 0, 0, 0, 0,
                        0, 0.1, 0, 0, 0, 0,
                        0, 0, 0.1, 0, 0, 0,
                        0, 0, 0, 0.1, 0, 0,
                        0, 0, 0, 0, 0.1, 0,
                        0, 0, 0, 0, 0, 0.1;

        //Measurement noise matrix
        R = Eigen::MatrixXd::Identity(2,2);
        R << 0.01, 0,
             0, 0.01;
        
        I = Eigen::MatrixXd::Identity(6,6);

        xt = Eigen::VectorXd::Zero(6);
        xt << 0.5,0.5,0,0,0,0;

        z = Eigen::VectorXd::Zero(2);
        z << 0, 0;
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


    //for scan
    int measurements;
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
    ros::Time prevt_1; //previous time t-1

    std::vector<float> ranges;

    //ros system variables
    
    ros::Subscriber sub;        //subscriber odom
    ros::Subscriber sub_laser;  //subscriber laser
    ros::Publisher cov_pub;     //publisher covariance
    ros::Publisher pubkpose;    

    //landmark positions
    double landmarkx = -3;
    double landmarky = 0;

    public:
    //prediction Thrun 
    void predict(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::VectorXd &ut)
    {
        // std::cout<<xt.rows()<<" "<<xt.cols()<<std::endl;
        // std::cout<<B<<std::endl;
         
         
        // std::cout<<B.rows()<<" "<< B.cols()<< std::endl;
        // std::cout<<ut.rows()<<" "<<ut.cols()<<std::endl;
        

        xt = A * xt + B* ut;                                    //PDF Slides Gaussian Filters line 2 p.11 - apply motion model
        covarianceMat = A * covarianceMat * A.transpose();    //PDF Slides Gaussian Filters line 3 p.11 - get prediction of covariance
        //std::cout << "hier wird predict aufgerufen!"<< std::endl;
    }


    void correction(const Eigen::MatrixXd &z, const Eigen::MatrixXd &C, const Eigen::MatrixXd &Q)
    {
        updateTime();
        //Q fehlt
        
        
        K = covarianceMat * C.transpose() * (C * covarianceMat * C.transpose()).inverse(); //PDF Slides Gaussian Filters line 4 p.11 - calculation of the Kalman gain
        xt = xt + K * (z - C * xt);                  //PDF Slides Gaussian Filters line 5 p.11 - correction of the expectation using the Kalman gain
        covarianceMat = (I - K * C) * covarianceMat; //PDF Slides Gaussian Filters line 6 p.11 - correction of covariance matrix

        /*for(int i=0; i<1; i++)
        {
            if()
        }*/
        covar_pose(cov_pose);
        cov_pub.publish(cov_pose);
    }
    //Matrizen anpassen
    void updateMatrices()
    {
        A << 1, 0, 0, dt, 0, 0,
             0, 1, 0, 0, dt, 0,
             0, 0, 1, 0, 0, dt,
             0, 0, 0, 0.5, 0, 0,
             0, 0, 0, 0, 0.5, 0,
             0, 0, 0, 0, 0, 0.5;

        double tmp1 = (0.5*cos(xt[2]));
        double tmp2 = (0.5*sin(xt[2]));

        B << 0, 0,
             0, 0,
             0, 0,
             tmp1, 0,
             tmp2, 0,
             0, 0.5;

        //bad values
         C << 1, 0, 0, 1, 1, dt,
              1, dt, 0, dt*dt/2.0, dt*dt/2.0, dt*dt/6.0;

    }

    void updateTime()
    {
        ros::Time timeNow = ros::Time::now();
        ros::Duration timediff = timeNow - prevt_1;
        dt = timediff.toSec();
        updateMatrices();

    }

    void callback_odom(const nav_msgs::Odometry::ConstPtr& msg)
    {
        
        updateTime();

        odom_pose = msg->pose.pose;//get odom pose
        
        v_linear = msg->twist.twist.linear.x; //get linear velocity
        w_angular = msg->twist.twist.angular.z; //get angular velocity

        delta_v_linear = v_linear - v_lineart_1;   //calculation of the difference of linear velocities
        v_lineart_1 = v_linear;                    //set the current linear velocity to t-1 

        delta_w_angular = w_angular - w_angular_1;  //calculation of the difference of angular velocities
        w_angular_1 = w_angular;                    //set the current angular velocity to t-1

        ut[0] = v_linear;
        ut[1] = w_angular;
        //std::cout << "hier wird odom aufgerufen!"<< std::endl;
        std::cout<<ut.transpose()<<std::endl;
        predict(A, B, ut);
    }

    void covar_pose(geometry_msgs::PoseWithCovarianceStamped &pose)
    {
        std::string fixed_frame = "map";
    //geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.frame_id = fixed_frame;
    pose.header.stamp = ros::Time::now();

    // set x,y coord
    pose.pose.pose.position.x = xt[0];
    pose.pose.pose.position.y = xt[1];
    pose.pose.pose.position.z = 0.0;

    //pose.pose.covariance[0]=covarianceMat(0,0); 
    //pose.pose.covariance[7]=covarianceMat(1,1);
    //pose.pose.covariance[35]=covarianceMat(4,4);

    // set theta
    tf::Quaternion quat;
    quat.setRPY(0.0, 0.0, xt[2]);
    tf::quaternionTFToMsg(quat, pose.pose.pose.orientation);

    for (int i=0; i<6; ++i)
      {
        for (int j=0; j<6; ++j)
        {    
          pose.pose.covariance[6*i+j] = covarianceMat.coeff(i,j);
        }
      }
    }

    // void cova_pose(const Eigen::VectorXd &state, const Eigen::MatrixXd &covariance)
    // {

    //     geometry_msgs::PoseWithCovarianceStamped cov_msg;
    //     cov_msg.header.stamp = ros::Time::now();
    //     cov_msg.header.frame_id = "base_link";
    //     cov_msg.pose.pose.position.x= state[0];
    //     cov_msg.pose.pose.position.y=state[1];

    //     tf2::Quaternion q;
    //     q.setRPY(0, 0, state[2]);
    //     cov_msg.pose.pose.orientation.x=q.x();
    //     cov_msg.pose.pose.orientation.y=q.y();
    //     cov_msg.pose.pose.orientation.z=q.z();
    //     cov_msg.pose.pose.orientation.w=q.w();

    //     for (int i =0; i< 36;i++)
    //         {
                
    //             cov_msg.pose.covariance[i] = 0;
                
    //         }
            
    //         //schreibe nur auf die x,y und orientrungs positionen in der covarinaz
    //         cov_msg.pose.covariance[0]=covariance(0,0); 
    //         cov_msg.pose.covariance[7]=covariance(1,1);
    //         cov_msg.pose.covariance[35]=covariance(4,4);

    //         cov_pub.publish(cov_msg);


    // }

    void laserscan_callback(const sensor_msgs::LaserScan::ConstPtr& laser)
    {
        //std::cout<<"hier ist der laser callback!"<<std::endl;
        ranges = laser->ranges;
        measurements = ranges.size(); 
        correction(z, C, Q);
    }



};


int main(int argc, char** argv)
{

    ros::init(argc, argv, "kfloc");
    // ros::NodeHandle n;
    kfFilter kf;
    // ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("odom",1,&kfFilter::callback_odom, &kf);
    // ros::Subscriber sub_laser = n.subscribe("scan",1,&kfFilter::laserscan_callback, &kf);
    // ros::Publisher cov_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/covariance",1);
    // ros::Publisher pubkpose = n.advertise<geometry_msgs::PoseWithCovarianceStamped> ("/kfpose", 1);
    


    ros::spin();


    return 0;
}