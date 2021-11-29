#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <fstream>
#include <fstream>
#include <ios>

/** @brief : Gets the map produced by the gazebo_ros_2Dmap plugin and converts it to a txt file 
 * callMap() calls the service that produces the map
 * The most recent map is received using getMap() and stored as a class attribute.
 * Then, the function OccGridToTxt() converts it to txt and saves it at required path. 
 * */

class Map {
public:
    Map() {
        callMap();                                                      // Called service to get map
        getGrid_ = nh_.subscribe("/map2d", 10, &Map::getMap, this);     // Subscriber, will be sent to callback which sends it to the function for processing it to .txt
    }
    void OccGridToTxt();
    void callMap();
    void getMap(const nav_msgs::OccupancyGrid& gridMap);

protected:
    ros::NodeHandle nh_;
    ros::Subscriber getGrid_;
    nav_msgs::OccupancyGrid gazeboMap_;

};

void Map::callMap() 
{
  ros::ServiceClient client = nh_.serviceClient<std_srvs::Empty>("/gazebo_2Dmap_plugin/generate_map");
  std_srvs::Empty srv;

  if (client.call(srv))
    ROS_INFO("Map service call successful");

  else
    ROS_ERROR("Failed to call service for getting map, maybe the gazebo world is not complete yet?");
}

void Map::getMap(const nav_msgs::OccupancyGrid& gridMap)
{
    callMap();
    gazeboMap_ = gridMap;
    if(!gazeboMap_.data.empty())                          // Checks to see if the map is empty
        OccGridToTxt();
}

void Map::OccGridToTxt()
{
    std::ofstream myfile("/home/jibran/catkin_ws/src/MotionPlanningWithVFH/planning/scripts/map_no_sides.txt", std::ios::trunc);

    for(int i{}; i < gazeboMap_.data.size(); i++)
    {
        if((i % gazeboMap_.info.height) == 0)          // Since the occgrid stores all points as 1 long 1D array, we have to add rows according to the data index
            myfile << std::endl;                   // For eg: If the map is 4 x 5 , every 5th point will have to be added to a new row.

        else if(gazeboMap_.data.at(i) > 0)         // Data with greater than 0 (obstacle) is added as 1 since our map is binary.
            myfile << 1 << " ";
        
        else
            myfile << 0 << " ";                         // 0 is 0
    }
    ROS_INFO_STREAM("Map of size " << gazeboMap_.data.size() << " copied into a text file" );
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "MapCreator");
    Map mapObject;
    ros::spinOnce();

    return 0;
}

