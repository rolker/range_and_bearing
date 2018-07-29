#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float32.h"
#include "project11/mutex_protected_bag_writer.h"
#include <regex>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "project11/gz4d_geo.h"

ros::Publisher rangePub;
ros::Publisher bearingPub;

double lat1, lon1, lat2, lon2;
ros::Time ts1, ts2;

MutexProtectedBagWriter log_bag;

void navSatFix1Callback(const sensor_msgs::NavSatFix::ConstPtr& message)
{
    ts1 = message->header.stamp;
    lat1 = message->latitude;
    lon1 = message->longitude;
}

void navSatFix2Callback(const sensor_msgs::NavSatFix::ConstPtr& message)
{
    ts2 = message->header.stamp;
    lat2 = message->latitude;
    lon2 = message->longitude;
}

void sendRangeAndBearing(const ros::TimerEvent event)
{
    if(event.current_real-ts1 < ros::Duration(2.0) && event.current_real-ts2 < ros::Duration(2.0))
    {
        ros::Time now = ros::Time::now();
        gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> gr(lat1,lon1,0.0);
        gz4d::geo::LocalENU<> geoReference = gz4d::geo::LocalENU<>(gr);
        gz4d::Point<double> position = geoReference.toLocal(gz4d::geo::Point<double,gz4d::geo::WGS84::ECEF>(gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon>(lat2,lon2,0.0)));
        double range = gz4d::norm(position);
        std_msgs::Float32 rangeMsg;
        rangeMsg.data = range;
        rangePub.publish(rangeMsg);
        log_bag.write("/range",now,rangeMsg);
        
        gz4d::Point<double> northRef(0.0,1.0,0.0);
        double n = norm(northRef)*norm(position);
        gz4d::Angle<double,gz4d::Radian> angle  = acos(northRef.dot(position)/n);
        gz4d::Angle<double,gz4d::Degree> angleDeg(angle);
        double bearing = angleDeg.value();
        if(position[0] < 0.0)
            bearing = 360.0-angleDeg.value();
//        std::cerr << angle.value() << " radians, in degrees: " << angleDeg.value() << " bearing: " << bearing << std::endl; 
        
        std_msgs::Float32 bearingMsg;
        bearingMsg.data = bearing;
        bearingPub.publish(bearingMsg);
        log_bag.write("/bearing",now,bearingMsg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_to_twiststamped");
    ros::NodeHandle n;
    
    boost::posix_time::ptime now = ros::WallTime::now().toBoost();
    std::string iso_now = std::regex_replace(boost::posix_time::to_iso_extended_string(now),std::regex(":"),"-");
    std::string log_filename = "nodes/range_and_bearing-"+iso_now+".bag";
    log_bag.open(log_filename, rosbag::bagmode::Write);

    ros::Subscriber input1 = n.subscribe("/input1",10,&navSatFix1Callback);
    ros::Subscriber input2 = n.subscribe("/input2",10,&navSatFix2Callback);
    
    rangePub = n.advertise<std_msgs::Float32>("/range",1);
    bearingPub = n.advertise<std_msgs::Float32>("/bearing",1);

    ros::Timer timer = n.createTimer(ros::Duration(1.0),sendRangeAndBearing);
    ros::spin();
    
    return 0;
    
}
