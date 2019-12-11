#include <qualisys/QualisysDriver.h>
#include <algorithm>

using namespace std;

namespace qualisys{

double QualisysDriver::deg2rad = M_PI / 180.0;

QualisysDriver::QualisysDriver(const ros::NodeHandle& n):
  nh(n),
  publish_tf(false){
  return;
}

bool QualisysDriver::init() {
  // The base port (as entered in QTM, TCP/IP port number, in the RT output tab
  // of the workspace options
  nh.param("server_address", server_address, string("192.168.254.1"));
  nh.param("server_base_port", base_port, 22222);
  nh.param("publish_tf", publish_tf, false);

  // Connecting to the server
  ROS_INFO_STREAM("Connecting to the Qualisys Motion Tracking system specified at: "
      << server_address << ":" << base_port);

  if(!port_protocol.Connect((char *)server_address.data(), base_port, 0, 1, 7)) {
    ROS_FATAL_STREAM("Could not find the Qualisys Motion Tracking system at: "
        << server_address << ":" << base_port);
    return false;
  }
  ROS_INFO_STREAM("Connected to " << server_address << ":" << base_port);

  // Get 6DOF settings
  port_protocol.Read6DOFSettings();

  return true;
}

void QualisysDriver::disconnect() {
  ROS_INFO_STREAM("Disconnected with the server "
      << server_address << ":" << base_port);
  port_protocol.StreamFramesStop();
  port_protocol.Disconnect();
  return;
}

void QualisysDriver::checkPublishers(const int& body_count) {
  map<string, bool> subject_indicator;

  for (auto it = subject_publishers.begin();
      it != subject_publishers.end(); ++it)
    subject_indicator[it->first] = false;

  // Check publishers for each body
  for(int i = 0; i < body_count; ++i) {
    //std::stringstream name;
    //name << port_protocol.Get6DOFBodyName(i);
    string name(port_protocol.Get6DOFBodyName(i));

    // Create a publisher for the rigid body
    // if it does not have one.
    if (subject_publishers.find(name) ==
          subject_publishers.end()) {
      subject_publishers[name] =
        nh.advertise<qualisys::Subject>(name, 10);
      pose_publishers[name] =
        nh.advertise<geometry_msgs::PoseStamped>(name + "/pose", 10);
	}

    subject_indicator[name] = true;
  }

  for (auto it = subject_indicator.begin();
      it != subject_indicator.end(); ++it) {
    if (it->second == false) {
      subject_publishers.erase(it->first);
      pose_publishers.erase(it->first);
    }
  }

  return;
}

void QualisysDriver::handlePacketData(CRTPacket* prt_packet) {

  // Number of rigid bodies
  int body_count = prt_packet->Get6DOFEulerBodyCount();

  // Check the publishers for the rigid bodies
  checkPublishers(body_count);

  // Publish data for each rigid body
  for(int i = 0; i < body_count; ++i) {
    float x, y, z, roll, pitch, yaw;
    prt_packet->Get6DOFEulerBody(i, x, y, z, roll, pitch, yaw);

    if(isnan(x) || isnan(y) || isnan(z) ||
        isnan(roll) || isnan(pitch) || isnan(yaw)) {
      ROS_WARN_STREAM_THROTTLE(3, "Rigid-body " << i + 1 << "/"
          << body_count << " not detected");
      continue;
    }

    // ROTATION: GLOBAL (FIXED) X Y Z (R P Y)
    //std::stringstream name;
    //name << port_protocol.Get6DOFBodyName(i);
    string subject_name(port_protocol.Get6DOFBodyName(i));

    // Qualisys sometimes flips 180 degrees around the x axis
    if(roll > 90)
      roll -= 180;
    else if(roll < -90)
      roll += 180;

    // Send transform
    tf::StampedTransform stamped_transform = tf::StampedTransform(
        tf::Transform(
            tf::createQuaternionFromRPY(
                roll * deg2rad, pitch * deg2rad, yaw * deg2rad),
            tf::Vector3(x, y, z) / 1000.),
        ros::Time::now(), "qualisys", subject_name);
    if (publish_tf)
      tf_publisher.sendTransform(stamped_transform);

    // Send Subject msg
    geometry_msgs::TransformStamped geom_stamped_transform;
    tf::transformStampedTFToMsg(stamped_transform,
        geom_stamped_transform);

    qualisys::Subject subject_msg;
    subject_msg.header =
      geom_stamped_transform.header;
    subject_msg.name = subject_name;
    subject_msg.position.x =
        geom_stamped_transform.transform.translation.x;
    subject_msg.position.y =
        geom_stamped_transform.transform.translation.y;
    subject_msg.position.z =
        geom_stamped_transform.transform.translation.z;
    subject_msg.orientation =
        geom_stamped_transform.transform.rotation;
    subject_publishers[subject_name].publish(subject_msg);

	geometry_msgs::PoseStamped pose_msg;
	pose_msg.header = geom_stamped_transform.header;
	pose_msg.pose.position.x = geom_stamped_transform.transform.translation.x;
	pose_msg.pose.position.y = geom_stamped_transform.transform.translation.y;
	pose_msg.pose.position.z = geom_stamped_transform.transform.translation.z;
	pose_msg.pose.orientation.x = geom_stamped_transform.transform.rotation.x;
	pose_msg.pose.orientation.y = geom_stamped_transform.transform.rotation.y;
	pose_msg.pose.orientation.z = geom_stamped_transform.transform.rotation.z;
	pose_msg.pose.orientation.w = geom_stamped_transform.transform.rotation.w;
    pose_publishers[subject_name].publish(pose_msg);
  }

  return;
}

void QualisysDriver::run() {

  CRTPacket* prt_packet = port_protocol.GetRTPacket();
  CRTPacket::EPacketType e_type;
  port_protocol.GetCurrentFrame(CRTProtocol::Component6dEuler);

  if(port_protocol.ReceiveRTPacket(e_type, true)) {

    switch(e_type) {
      // Case 1 - sHeader.nType 0 indicates an error
      case CRTPacket::PacketError:
        ROS_ERROR_STREAM_THROTTLE(
            1, "Error when streaming frames: "
            << port_protocol.GetRTPacket()->GetErrorString());
        break;

      // Case 2 - No more data
      case CRTPacket::PacketNoMoreData:
        ROS_WARN_STREAM_THROTTLE(1, "No more data");
        break;

      // Case 3 - Data received
      case CRTPacket::PacketData:
        handlePacketData(prt_packet);
        break;

      default:
        ROS_ERROR_THROTTLE(1, "Unknown CRTPacket case");
    }
  }

  return;
}

}

