#include <ros/sonar_ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sonar_node");

//    std::vector<std::string> sonar_names;

//    sonar_names.push_back("/bosch_uss5/ultrasound_rr_sag");
//    sonar_names.push_back("/bosch_uss5/ultrasound_rr_lat");

    auto sr = SonarOccupancyMapROS();
    sr.spin();
    return 0;

}
