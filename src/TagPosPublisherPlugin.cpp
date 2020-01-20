#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Collision.hh>
#include "gazebo/physics/PhysicsTypes.hh"
#include <gazebo/common/common.hh>
#include <gazebo/sensors/Noise.hh>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <gtec_msgs/GenericRanging.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>


namespace gazebo
{
class TagPosPublisherPlugin : public ModelPlugin
{


public: TagPosPublisherPlugin() :
    ModelPlugin(),
    sequence(0)
  {
    this->updatePeriod = common::Time(0.0);
  }

public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                       << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    ROS_INFO("TagPosPublisher Plugin running");

    this->model = _parent;
    this->world = _parent->GetWorld();
    this->zOffset = 0.0;

    if (_sdf->HasElement("z_offset"))
    {
      this->zOffset= _sdf->Get<double>("z_offset");
    } 

    if (_sdf->HasElement("update_rate"))
    {
      this->SetUpdateRate(_sdf->Get<double>("update_rate"));
    } 

    if (_sdf->HasElement("tag_link"))
    {
      std::string tag_link = _sdf->Get<std::string>("tag_link");
      this->tagLink = _parent->GetLink(tag_link);
    }

    this->lastUpdateTime = common::Time(0.0);
    
    ros::NodeHandle n;
    this->gtecRealPos = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/gtec/gazebo/pos", 1000);
    this->gtecRealPosOdom = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/gtec/gazebo/pos/odom", 1000);
    
    this->updateConnection =
      event::Events::ConnectWorldUpdateBegin(boost::bind(&TagPosPublisherPlugin::OnUpdate, this, _1));
  }

public: void OnUpdate(const common::UpdateInfo &_info)
  {
    common::Time simTime = _info.simTime;
    common::Time elapsed = simTime - this->lastUpdateTime;

    if (elapsed >= this->updatePeriod)
    {
      this->lastUpdateTime = _info.simTime;
      ignition::math::Pose3d tagPose = this->tagLink->WorldPose();

      geometry_msgs::PoseWithCovarianceStamped pose;
      pose.pose.pose.position.x = tagPose.Pos().X();
      pose.pose.pose.position.y = tagPose.Pos().Y();
      pose.pose.pose.position.z = tagPose.Pos().Z() + this->zOffset;
      pose.pose.pose.orientation.x = tagPose.Rot().X();
      pose.pose.pose.orientation.y = tagPose.Rot().Y();
      pose.pose.pose.orientation.z = tagPose.Rot().Z();
      pose.pose.pose.orientation.w = tagPose.Rot().W();
      pose.header.frame_id = "world";
      pose.header.stamp = ros::Time::now();

      this->gtecRealPos.publish(pose);


      geometry_msgs::PoseWithCovarianceStamped poseOdom;
      poseOdom.pose.pose.position.x = 0;
      poseOdom.pose.pose.position.y = 0;
      poseOdom.pose.pose.position.z = 0;
      poseOdom.pose.pose.orientation.x = 0;
      poseOdom.pose.pose.orientation.y = 0;
      poseOdom.pose.pose.orientation.z = 0;
      poseOdom.pose.pose.orientation.w = 1;
      poseOdom.header.frame_id = "world";
      poseOdom.header.stamp = ros::Time::now();

      this->gtecRealPosOdom.publish(poseOdom);

     // tf::TransformBroadcaster broadcaster;
      
      /*broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(tagPose.Rot().X(), tagPose.Rot().Y(), tagPose.Rot().Z(), tagPose.Rot().W()), tf::Vector3(tagPose.Pos().X(),tagPose.Pos().Y(),tagPose.Pos().Z())),
        pose.header.stamp,"map", "odom"));*/

        /*   broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0,0,0)),
        ros::Time::now(),"map", "odom"));*/


      this->sequence++;
    }
  }

public: void SetUpdateRate(double _rate)
  {
    if (_rate > 0.0)
    {
      this->updatePeriod = 1.0 / _rate;
    }
    else
    {
      this->updatePeriod = 0.0;
    }
  }

 public:  void Reset() override {
    ROS_INFO("RESET IN TagPosPublisherPlugin \n");
    this->lastUpdateTime = common::Time(0.0);
  }


private: physics::ModelPtr model;
private: physics::WorldPtr world;
private: event::ConnectionPtr updateConnection;
private: common::Time updatePeriod;
private: common::Time lastUpdateTime;
private: physics::LinkPtr tagLink;
private: unsigned char sequence;
private: ros::Publisher gtecRealPos;
private: ros::Publisher gtecRealPosOdom;
private: double zOffset;

};

GZ_REGISTER_MODEL_PLUGIN(TagPosPublisherPlugin)
}

