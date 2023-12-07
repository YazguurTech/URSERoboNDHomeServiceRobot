#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

visualization_msgs::Marker GetMarker(double px, double py)
{
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = px;
    marker.pose.position.y = py;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 0.4x0.4x0.4 here means 0.4m on a side
    marker.scale.x = 0.4;
    marker.scale.y = 0.4;
    marker.scale.z = 0.4;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    return marker;
}

enum State {
  PICKUP,
  CARRYING,
  DROPOFF
};

double threshold = 0.2;

double current_pos[2] = { 0.0, 0.0 };
double pickup_zone[2] = { 0.0, 0.0 }; // { 6.25, 4.75 }; 
double dropoff_zone[2] = { 0.0, 0.0 };// { -5.8, -3.5 };

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_pos[0] = msg->pose.pose.position.y;
  current_pos[1] = -msg->pose.pose.position.x;
}

bool reachedDestination(double current[2], double dest[2])
{
  double distance = sqrt(pow(dest[0] - current[0], 2) + pow(dest[1] - current[1], 2));

  ROS_INFO("Current coordinates: (%.2f, %.2f)", current[0], current[1]);
  ROS_INFO("Destination coordinates: (%.2f, %.2f)", dest[0], dest[1]);
  ROS_INFO("Distance between points: %.2f", distance);

  return (distance < threshold);
}

std::string stateToString(State value) {
  switch (value) {
    case PICKUP: return "PICKUP";
    case CARRYING: return "CARRYING";
    case DROPOFF: return "DROPOFF";
    default: return "UNKNOWN";
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub1 = n.subscribe("odom", 10, odom_callback);

  double pickup_location_x, pickup_location_y;
  double dropoff_location_x, dropoff_location_y;
  n.getParam("/pickup_location/px", pickup_location_x);
  n.getParam("/pickup_location/py", pickup_location_y);
  n.getParam("/dropoff_location/px", dropoff_location_x);
  n.getParam("/dropoff_location/py", dropoff_location_y);

  visualization_msgs::Marker markerToPickUp = GetMarker(pickup_location_x, pickup_location_y);
  visualization_msgs::Marker markerToDropOff = GetMarker(dropoff_location_x, dropoff_location_y);
  State currentState = PICKUP; 
  bool isPickUpMarkerPublished = false;
  pickup_zone[0] = (pickup_location_x);
  pickup_zone[1] = (pickup_location_y);

  dropoff_zone[0] = (dropoff_location_x);
  dropoff_zone[1] = (dropoff_location_y);

  while (ros::ok())
  {
    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    
    // Process callbacks
    ros::spinOnce();

    ROS_INFO("Current state: %s", stateToString(currentState).c_str());

    if(currentState == PICKUP)
    {
      if(reachedDestination(current_pos, pickup_zone))
      {
        markerToPickUp.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(markerToPickUp);

        ros::Duration(5).sleep();

        currentState = CARRYING;
      }
      else if(!isPickUpMarkerPublished)
      {
        ROS_INFO("Publishing the pick-up marker.");
        marker_pub.publish(markerToPickUp);
        isPickUpMarkerPublished = true;
      }
    }
    else if(currentState == CARRYING)
    {
      if(reachedDestination(current_pos, dropoff_zone))
      {       
        currentState = DROPOFF;
      }
    }
    else 
    {
        ROS_INFO("Publishing the drop-off marker.");
        marker_pub.publish(markerToDropOff);
        ros::Duration(5).sleep();
        return 0;
    }

    r.sleep();
  }
}