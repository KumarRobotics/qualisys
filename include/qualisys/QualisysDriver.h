#ifndef QUALISYS_DRIVER_H
#define QUALISYS_DRIVER_H

#include <sstream>
#include <cmath>
#include <string>

// Including ROS
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <qualisys/Subject.h>
#include <qualisys/RTProtocol.h>


namespace qualisys{

class QualisysDriver{

  public:
    /*
     * @brief Constructor
     * @param nh Ros node
     */
    QualisysDriver(const ros::NodeHandle& n);

    /*
     * @brief Destructor
     */
    ~QualisysDriver() {
      disconnect();
    }

    /*
     * @brief init Initialize the object
     * @return True if successfully initialized
     */
    bool init();

    /*
     * @brief run Start acquiring data from the server
     */
    void run();

    /*
     * @brief disconnect Disconnect to the server
     * @Note The function is called automatically when the
     *  destructor is called.
     */
    void disconnect();

  private:
    // Disable the copy constructor and assign operator
    QualisysDriver(const QualisysDriver& );
    QualisysDriver& operator=(const QualisysDriver& );

    // Initialize publishers
    void checkPublishers(const int& body_count);

    // Handle data packet
    void handlePacketData(CRTPacket* prt_packet);

    // Unit converter
    static double deg2rad;

    // Address and port of the server
    std::string server_address;
    int base_port;

    // Protocol to connect to the server
    CRTProtocol port_protocol;

    // Ros related
    ros::NodeHandle nh;

    // Publishers
    std::map<std::string, ros::Publisher> subject_publishers;
    std::map<std::string, ros::Publisher> pose_publishers;
    tf::TransformBroadcaster tf_publisher;

    // If publish tf msgs
    bool publish_tf;


};
}


#endif
