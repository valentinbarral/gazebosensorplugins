/*
MIT License

Copyright (c) 2020 Group of Electronic Technology and Communications. University of A Coruna.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/sensors/Noise.hh>
#include <gazebo/sensors/MagnetometerSensor.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


namespace gazebo
{
class MagneticDistortionPlugin : public SensorPlugin
{
public: MagneticDistortionPlugin() :
    fieldStrength(0.0)
  {
  }

public: std::string GetTopic() const
  {
    std::string topicName = "~/" + this->parentSensor->ParentName() + "/"
                            + this->parentSensor->Name()
                            + "/interfered";
    boost::replace_all(topicName, "::", "/");
    return topicName;
  }

public: void Load(sensors::SensorPtr sensor, sdf::ElementPtr _sdf)
  {
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                       << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    this->fieldStrength = _sdf->Get<double>("field_strength");
    if (this->fieldStrength == 0.0)
    {
      this->fieldStrength = 1e-6;
    }

    this->world = physics::get_world(sensor->WorldName());
    physics::EntityPtr entity = this->world->EntityByName(sensor->ParentName());

    this->link = boost::dynamic_pointer_cast<physics::Link>(entity);
    this->parentSensor = std::dynamic_pointer_cast<sensors::MagnetometerSensor>(sensor);
    // TODO: prefijo configurable
    this->interfererPrefix = "interferer_";
    this->node = transport::NodePtr(new transport::Node());
#if GAZEBO_MAJOR_VERSION < 8
    this->node->Init(this->world->GetName());
#else
    this->node->Init(this->world->Name());
#endif
    this->magPub = this->node->Advertise<msgs::Magnetometer>(this->GetTopic(), 50);
    this->connection = this->parentSensor->ConnectUpdated(std::bind(&MagneticDistortionPlugin::OnUpdate, this, this->parentSensor));
  }

public: void OnUpdate(sensors::SensorPtr)
  {
    ignition::math::Vector3d magneticField = this->parentSensor->MagneticField();
    ignition::math::Pose3d sensorPose = this->link->WorldPose();
    physics::Model_V models = this->world->Models();

    ignition::math::Vector3d interference;
    for (physics::Model_V::iterator iter = models.begin(); iter != models.end(); ++iter)
    {
      if ((*iter)->GetName().find(this->interfererPrefix) == 0)
      {
        physics::ModelPtr interferer = *iter;
        ignition::math::Pose3d interfererPose = interferer->WorldPose();
        double distance = sensorPose.Pos().Distance(interfererPose.Pos());
        ignition::math::Vector3d interfererDirection = interfererPose.Pos() - sensorPose.Pos();
        interference += interfererDirection / (distance * distance);
      }
    }
    // TODO: Intensidad de campo ajustable
    interference *= this->fieldStrength;
    interference = sensorPose.Rot().RotateVectorReverse(interference);

    ignition::math::Vector3d finalField = magneticField + interference;
    msgs::Magnetometer magMsg;
    msgs::Set(magMsg.mutable_field_tesla(), finalField);
    msgs::Set(magMsg.mutable_time(), this->world->SimTime());
    if (this->magPub)
    {
      this->magPub->Publish(magMsg);
    }

    //magneticField = sensorPose.Rot().RotateVector(magneticField);
    //ROS_INFO("finalField sin rotar: (%f, %f, %f)", finalField.X(), finalField.Y(), finalField.Z());
    //finalField = sensorPose.Rot().RotateVector(finalField);
    //ROS_INFO("finalField rotado: (%f, %f, %f)", finalField.X(), finalField.Y(), finalField.Z());
    //ROS_INFO("magneticField: (%f, %f, %f)", magneticField.X(), magneticField.Y(), magneticField.Z());
    //ROS_INFO("Interference: (%f, %f, %f)", interference.X(), interference.Y(), interference.Z());
  }

private: std::string interfererPrefix;
private: sensors::MagnetometerSensorPtr parentSensor;
private: event::ConnectionPtr connection;
private: physics::WorldPtr world;
private: physics::LinkPtr link;
private: transport::NodePtr node;
private: transport::PublisherPtr magPub;
private: double fieldStrength;
};
GZ_REGISTER_SENSOR_PLUGIN(MagneticDistortionPlugin)
}

