#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>

#include <string>
#include <numeric>

#include <ublox_msgs/NavPVT.h>

#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/Geoid.hpp>

#include <boost/circular_buffer.hpp>

static sensor_msgs::NavSatFix llh;
static geometry_msgs::PoseWithCovarianceStamped pose_cov;
static geometry_msgs::PoseStamped pose;
static geometry_msgs::Quaternion quat;
static int zone = 0;
static double utm_x,utm_y;
static bool northup = true;
static int mgrs_precision = 9;//9→0.1mm 
static std::string mgrs_code;
static ros::Publisher pub;
static ros::Publisher pub_cov;
static ros::Publisher stat_pub;
static std_msgs::Bool gnss_stat_msg;
double alt_above_sea = 0;
std::string input_msg = "fix";//or nmea
std::string output_msg = "mgrs";//or plane or UTM
std::string ubx_msg_name = "ublox";
static bool use_ublox_gps = false;
int buff_epoch = 1;
bool yaw_calc_enable = true;

int heading =0;

static geometry_msgs::PoseStamped prev_pose;

const double initial_pose_cov[3] = {10000.0,10000.0,40000.0};

boost::circular_buffer<double> position_buffer_x(buff_epoch);
boost::circular_buffer<double> position_buffer_y(buff_epoch);
boost::circular_buffer<double> position_buffer_z(buff_epoch);


const int GZD_ID_size = 5;//size of header like "53SPU"

void llh_callback(const sensor_msgs::NavSatFix::ConstPtr& msg){
    llh.latitude = msg->latitude;
    llh.longitude = msg->longitude;
    llh.altitude = msg->altitude;//height from ellipsoid
    
    // Experimental:for height conversion
    // GeographicLib::Geoid egm2008("egm2008-1");//change model if necessary

    //quality check
    if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX){
        gnss_stat_msg.data = false;
        return;
    }else{
        gnss_stat_msg.data = true;  
    }

    try{
        GeographicLib::UTMUPS::Forward(llh.latitude, llh.longitude, zone, northup, utm_x, utm_y);//llh to UTM
        // alt_above_sea = egm2008.ConvertHeight(llh.latitude,llh.longitude,llh.altitude,GeographicLib::Geoid::ELLIPSOIDTOGEOID);//height from geoid
    }
    catch(const GeographicLib::GeographicErr err){
        ROS_ERROR_STREAM("Failed to convert from LLH to UTM" << err.what());
        return;
    }
    

    //for test 
    // std::cout << std::setprecision(12) << "llh.lat " << llh.latitude << " llh.lon " << llh.longitude <<"\n";
    // std::cout << std::setprecision(12) << "utm_x " << utm_x << " utm_y " << utm_y <<"\n";
    // std::cout << "mgrs_code " << mgrs_code << "\n";
    // std::cout  << "mgrs zone:" << mgrs_code.substr(0,GZD_ID_size) << " mgrs_north:" << mgrs_code.substr(GZD_ID_size,mgrs_precision) << " mgrs_east:" << mgrs_code.substr(GZD_ID_size + mgrs_precision,mgrs_precision) <<"\n";
    // std::cout  <<  std::setprecision(mgrs_precision)  << " mgrs_north_double:" << std::stod(mgrs_code.substr(GZD_ID_size,mgrs_precision)) << " mgrs_east_double:" << std::stod(mgrs_code.substr(GZD_ID_size + mgrs_precision,mgrs_precision)) <<"\n";    
    // std::cout << "height above sea level " << alt_above_sea << "\n";
    // std::cout << "buff_epoch:" << buff_epoch <<  std::endl;
    // std::cout << std::setprecision(12) << "utm_x:" << utm_x << " utm_y:" << utm_y << std::endl;

    position_buffer_x.push_front(utm_x);
    position_buffer_y.push_front(utm_y);
    position_buffer_z.push_front(llh.altitude);

    if(position_buffer_x.full() ){
        pose.header.stamp = msg->header.stamp;
        pose.header.frame_id = "map";   
        
        //change to median
        // double mean_x = std::accumulate(position_buffer_x.begin(),position_buffer_x.end(),0.0) / buff_epoch;
        // double mean_y = std::accumulate(position_buffer_y.begin(),position_buffer_y.end(),0.0) / buff_epoch;    
        // double mean_z = std::accumulate(position_buffer_z.begin(),position_buffer_z.end(),0.0) / buff_epoch;

        boost::circular_buffer<double> sort_buffer_x(buff_epoch);
        boost::circular_buffer<double> sort_buffer_y(buff_epoch);
        boost::circular_buffer<double> sort_buffer_z(buff_epoch);
        sort_buffer_x = position_buffer_x;
        sort_buffer_y = position_buffer_y;
        sort_buffer_z = position_buffer_z;
        
        std::sort(sort_buffer_x.begin(),sort_buffer_x.end());
        std::sort(sort_buffer_y.begin(),sort_buffer_y.end());
        std::sort(sort_buffer_z.begin(),sort_buffer_z.end());
        double median[3];//x,y,z
        size_t median_index = sort_buffer_x.size() / 2; 

        // std::cout << "median_index:" << median_index <<  std::endl;
        // std::cout << "sort_buffer_x.size:" << sort_buffer_x.size() <<  std::endl;

        if(sort_buffer_x.size() % 2){
            median[0] = sort_buffer_x.at(median_index);
            median[1] = sort_buffer_y.at(median_index);
            median[2] = sort_buffer_z.at(median_index);
        }else{ 
            median[0] = (sort_buffer_x.at(median_index) + sort_buffer_x.at(median_index - 1)) / 2;
            median[1] = (sort_buffer_y.at(median_index) + sort_buffer_y.at(median_index - 1)) / 2;
            median[2] = (sort_buffer_z.at(median_index) + sort_buffer_z.at(median_index - 1)) / 2;
        }

        if(output_msg == "mgrs"){
            try{
                GeographicLib::MGRS::Forward(zone, northup, median[0], median[1], llh.latitude, mgrs_precision, mgrs_code);
            }catch(const GeographicLib::GeographicErr err){
                ROS_ERROR_STREAM("Failed to convert from UTM to MGRS" <<err.what());
                return;
            }
            
            pose.pose.position.x = std::stod(mgrs_code.substr(GZD_ID_size,mgrs_precision)) / 10000.0 ;//set unit as [m]
            pose.pose.position.y = std::stod(mgrs_code.substr(GZD_ID_size + mgrs_precision,mgrs_precision)) / 10000.0 ;//set unit as [m]
            pose.pose.position.z = median[2]; 
        }else if(output_msg == "plane"){
            //later

        }else{//utm
            pose.pose.position.x = median[0];
            pose.pose.position.y = median[1];
            pose.pose.position.z = median[2];
        }
         
        double position_diff[2] = {pose.pose.position.y - prev_pose.pose.position.y,pose.pose.position.x - prev_pose.pose.position.x};
        //for fix2tfpose/nmea2tfpose compatibility
        // if(sqrt(position_diff[0] * position_diff[0] + position_diff[1] * position_diff[1]) >0.2 ){
        //     yaw_calc_enable = true;
        // }
        double yaw;
        if(yaw_calc_enable){
            //something bugged for direction
            double yaw = atan2(position_diff[0] , position_diff[1]);
            int heading_conv;
            if(use_ublox_gps){
                //convert heading[0(North)~360] to yaw[-M_PI(West)~M_PI]
                if(heading >= 0 && heading <= 27000000){
                    heading_conv = 9000000 - heading;
                }else{
                    heading_conv = 45000000 - heading;
                }
                yaw = (heading_conv * 1e-5) * M_PI / 180.0;
            }
            // std::cout << "heading:" << heading << " heading_conv:" << heading_conv * 1e-5 << std::endl;
            quat = tf::createQuaternionMsgFromYaw(yaw);
            // double yaw_ubx = tf::getYaw(quat);
            // std::cout << "covert yaw:" << yaw_ubx << std::endl;

            pose.pose.orientation = quat;
        }else{
            pose.pose.orientation.w = 1.0;
        }
        




        // rotation from 2point  ( use **RMC/VTG if available) 
        pub.publish(pose);
        prev_pose = pose;
        stat_pub.publish(gnss_stat_msg);

        //add tf for keeping compatibility fix2tfpose/nmea2tfpose
        static tf::TransformBroadcaster tf_br;
        tf::Transform transform;
        tf::Quaternion tf_quat;
        transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
        tf_quat.setRPY(0.0, 0.0, yaw);
        transform.setRotation(tf_quat);
        tf_br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map","gps"));

        pose_cov.header = pose.header;
        pose_cov.pose.pose = pose.pose;        
        //covariance from navsatfix for temporary(this use HDOP but bad HDOP don't always means bad accuracy)
        // need to read other message for more precision
        double msg_pose_cov[3];
        if(msg->position_covariance_type > sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN){
            msg_pose_cov[0] = msg->position_covariance[0]; //East
            msg_pose_cov[1] = msg->position_covariance[4]; //North
            msg_pose_cov[2] = msg->position_covariance[8];//Up
        }else{
            msg_pose_cov[0] = initial_pose_cov[0];//worst vaule
            msg_pose_cov[1] = initial_pose_cov[1];
            msg_pose_cov[2] = initial_pose_cov[2];
        }
        pose_cov.pose.covariance[6*0 + 0] = msg_pose_cov[0];
        pose_cov.pose.covariance[6*1 + 1] = msg_pose_cov[1];
        pose_cov.pose.covariance[6*2 + 2] = msg_pose_cov[2];
        
        pub_cov.publish(pose_cov);
    }    
    // ROS_INFO("sub %d",heading);

}

void heading_callback(const ublox_msgs::NavPVT::ConstPtr& msg){
    heading = msg->heading;

    // ROS_INFO("subc %f",heading);
}



int main(int argc, char** argv){
    ros::init(argc, argv,"gnss2tfpose");

    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    n_private.getParam("/gnss2tfpose/input_msg",input_msg);
    n_private.getParam("/gnss2tfpose/output_msg",output_msg);  
    n_private.getParam("/gnss2tfpose/buff_epoch",buff_epoch);
    n_private.getParam("/gnss2tfpose/use_ublox_gps",use_ublox_gps);
    n_private.getParam("/gnss2tfpose/ubx_msg_name",ubx_msg_name);

    position_buffer_x.set_capacity(buff_epoch);
    position_buffer_y.set_capacity(buff_epoch);
    position_buffer_z.set_capacity(buff_epoch);

    ros::Subscriber llh = n.subscribe("/fix", 1000, llh_callback);
    ros::Subscriber heading_sub;
    if(use_ublox_gps){
        //subscribe ublox_msgs
        std::string ubx_msg_fix = ubx_msg_name +  "/fix";
        std::string ubx_msg_heading = ubx_msg_name + "/navpvt";
        ROS_INFO("sub %s",ubx_msg_name.c_str());
        llh = n.subscribe(ubx_msg_fix, 1000, llh_callback);
        heading_sub = n.subscribe(ubx_msg_heading, 1000, heading_callback);
    }

    pub = n.advertise<geometry_msgs::PoseStamped>("/gnss_pose",1000);
    pub_cov = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/gnss_pose_cov",1000);
    stat_pub = n.advertise<std_msgs::Bool>("/fix",1000);


    ros::spin();

    return 0;


}
