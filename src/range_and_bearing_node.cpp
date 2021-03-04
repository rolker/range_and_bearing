#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float32.h"
//#include "project11/mutex_protected_bag_writer.h"
#include <regex>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "project11/utils.h"

ros::Publisher rangePub;
ros::Publisher bearingPub;

double lat1, lon1, lat2, lon2;
ros::Time ts1, ts2;

namespace p11 = project11;

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
        p11::LatLongDegrees gr(lat1,lon1,0.0);
        p11::ENUFrame geoReference(gr);
        p11::Point position = geoReference.toLocal(p11::LatLongDegrees(lat2,lon2,0.0));
        double range = gz4d::norm(position);
        std_msgs::Float32 rangeMsg;
        rangeMsg.data = range;
        rangePub.publish(rangeMsg);
        
        p11::Point northRef(0.0,1.0,0.0);
        double n = norm(northRef)*norm(position);
        p11::AngleRadiansPositive angle  = acos(northRef.dot(position)/n);
        double bearing = static_cast<double>(p11::AngleDegrees(angle));
        
        std_msgs::Float32 bearingMsg;
        bearingMsg.data = bearing;
        bearingPub.publish(bearingMsg);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "range_and_bearing");
    ros::NodeHandle n;
    
    ros::Subscriber input1 = n.subscribe("input1",10,&navSatFix1Callback);
    ros::Subscriber input2 = n.subscribe("input2",10,&navSatFix2Callback);
    
    rangePub = n.advertise<std_msgs::Float32>("range",1);
    bearingPub = n.advertise<std_msgs::Float32>("bearing",1);

    ros::Timer timer = n.createTimer(ros::Duration(1.0),sendRangeAndBearing);
    ros::spin();
    
    return 0;
}
