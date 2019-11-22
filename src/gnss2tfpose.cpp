#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>

#include <string>
#include <numeric>


#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/Geoid.hpp>

#include <boost/circular_buffer.hpp>

static sensor_msgs::NavSatFix llh;
static geometry_msgs::PoseWithCovarianceStamped pose_cov;
static geometry_msgs::PoseStamped pose;
static int zone = 0;
static double utm_x,utm_y;
static bool northup = true;
static int mgrs_precision = 9;//9→0.1mm 
static std::string mgrs_code;
static ros::Publisher pub;
static ros::Publisher pub_cov;
double alt_above_sea = 0;
std::string input_msg = "fix";//or nmea
std::string output_msg = "mgrs";//or plane or UTM
int buff_epoch = 1;
bool yaw_calc_enable = true;

static geometry_msgs::PoseStamped prev_pose;


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
        return;
    }
    
    try{
        GeographicLib::UTMUPS::Forward(llh.latitude, llh.longitude, zone, northup, utm_x, utm_y);//llh to UTM
        // alt_above_sea = egm2008.ConvertHeight(llh.latitude,llh.longitude,llh.altitude,GeographicLib::Geoid::ELLIPSOIDTOGEOID);//height from geoid
    }
    catch(const GeographicLib::GeographicErr err){
        ROS_ERROR_STREAM(err.what());
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
                ROS_ERROR_STREAM(err.what());
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

        if(yaw_calc_enable){
            //something bugged for direction
            double yaw = atan2(pose.pose.position.y - prev_pose.pose.position.y , pose.pose.position.x - prev_pose.pose.position.x );
            std::cout << "yaw:" << yaw << std::endl;
            geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(yaw);// yaw->quat is OK
            pose.pose.orientation = quat;
        }else{
            pose.pose.orientation.w = 1.0;
        }
        


        // rotation from 2point  ( use **RMC/VTG if can) 
        pub.publish(pose);
        prev_pose = pose;
        
        //add tf for keeping compatibility fix2tfpose/nmea2tfpose

        //add covariance(temporary)
        pose_cov.header = pose.header;
        pose_cov.pose.pose = pose.pose;
        //just copy&paste
        pose_cov.pose.covariance[6*0 + 0] = 1;
        pose_cov.pose.covariance[6*1 + 1] = 1;
        pose_cov.pose.covariance[6*2 + 2] = 1;


    }    
}



int main(int argc, char** argv){
    ros::init(argc, argv,"gnss2tfpose");

    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    n_private.getParam("/gnss2tfpose/input_msg",input_msg);
    n_private.getParam("/gnss2tfpose/output_msg",output_msg);  
    n_private.getParam("/gnss2tfpose/buff_epoch",buff_epoch);

    position_buffer_x.set_capacity(buff_epoch);
    position_buffer_y.set_capacity(buff_epoch);
    position_buffer_z.set_capacity(buff_epoch);

    ros::Subscriber llh = n.subscribe("/fix", 1000, llh_callback);

    pub = n.advertise<geometry_msgs::PoseStamped>("/gnss_pose",1000);
    pub_cov = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/gnss_pose_cov",1000);


    ros::spin();

    return 0;


}
