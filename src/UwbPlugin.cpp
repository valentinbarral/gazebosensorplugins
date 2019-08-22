/*
MIT License

Copyright (c) 2018 Group of Electronic Technology and Communications. University of A Coruña.

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


namespace gazebo
{
class UwbPlugin : public ModelPlugin
{
  double rxPowerVsBias[23][3] = {
    { -44, -0.230, -0.130 },
    { -54, -0.230, -0.125 },
    { -56, -0.220, -0.120 },
    { -60, -0.210, -0.115 },
    { -61, -0.198, -0.110 },
    { -63, -0.187, -0.105 },
    { -65, -0.179, -0.100 },
    { -67, -0.163, -0.093 },
    { -69, -0.143, -0.082 },
    { -71, -0.127, -0.069 },
    { -73, -0.109, -0.051 },
    { -75, -0.084, -0.027 },
    { -77, -0.059, 0 },
    { -79, -0.031, 0.021 },
    { -81, 0, 0.035 },
    { -83, 0.036, 0.042 },
    { -85, 0.065, 0.049 },
    { -87, 0.084, 0.062 },
    { -89, 0.097, 0.071 },
    { -91, 0.106, 0.076 },
    { -93, 0.110, 0.081 },
    { -95, 0.120, 0.086 },
    { -97, 0.130, 0.091 }
  };

  double rxPowerEstimatedPrf16[39][2] = {
    { -103, -103 },
    { -102, -101.5 },
    { -101, -100 },
    { -100, -99 },
    { -99, -98 },
    { -98, -97.8 },
    { -97, -97.2 },
    { -96, -96.3 },
    { -95, -95.2 },
    { -94, -94.3 },
    { -93, -93.2 },
    { -92, -92.5 },
    { -91, -91.5 },
    { -90, -90.5 },
    { -89, -89.5 },
    { -88, -89 },
    { -87, -88 },
    { -86, -87.8 },
    { -85, -87.2 },
    { -84, -87 },
    { -83, -86.5 },
    { -82, -86 },
    { -81, -85.8 },
    { -80, -85.5 },
    { -79, -85.2 },
    { -78, -85 },
    { -77, -84.8 },
    { -76, -84.2 },
    { -75, -84 },
    { -74, -83.8 },
    { -73, -83.5 },
    { -72, -83 },
    { -71, -82.8 },
    { -70, -82.6 },
    { -69, -82.4 },
    { -68, -82.1 },
    { -67, -82 },
    { -66, -81.3 },
    { -50, -81.3 }
  };

  double rxPowerEstimatedPrf64[40][2] = {
    { -120, -120 },
    { -103, -103 },
    { -102, -102.2 },
    { -101, -102 },
    { -100, -100.5 },
    { -99, -99 },
    { -98, -97.5 },
    { -97, -97 },
    { -96, -96 },
    { -95, -95 },
    { -94, -94 },
    { -93, -93 },
    { -92, -92 },
    { -91, -91 },
    { -90, -90 },
    { -89, -89.5 },
    { -88, -89 },
    { -87, -88 },
    { -86, -87 },
    { -85, -86 },
    { -84, -85.2 },
    { -83, -84.5 },
    { -82, -83.5 },
    { -81, -83 },
    { -80, -82.2 },
    { -79, -81.8 },
    { -78, -81.2 },
    { -77, -81 },
    { -76, -80.8 },
    { -75, -80.5 },
    { -74, -80.1 },
    { -73, -80 },
    { -72, -79.9 },
    { -71, -79.7 },
    { -70, -79.5 },
    { -69, -79.3 },
    { -68, -79.2 },
    { -67, -79 },
    { -66, -78.9 },
    { -50, -78.9 }
  };

double normalDis[8][2] = {
  {0.80 , 1.281552},
  {0.90 , 1.644854},
  {0.95 , 1.959964},
  {0.98 , 2.326348},
  {0.99 , 2.575829},
  {0.995 , 2.807034},
  {0.998 , 3.090232},
  {0.999 , 3.290527}
};

public: UwbPlugin() :
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

    ROS_INFO("UWB Plugin running");

    this->model = _parent;
    this->world = _parent->GetWorld();
    this->SetUpdateRate(_sdf->Get<double>("update_rate"));
    this->power = _sdf->Get<double>("power");
    this->txGain = _sdf->Get<double>("tx_gain");
    this->rxGain = _sdf->Get<double>("rx_gain");
    this->frequency = _sdf->Get<double>("frequency");

    if (_sdf->HasElement("tag_link"))
    {
      std::string tag_link = _sdf->Get<std::string>("tag_link");
      this->tagLink = _parent->GetLink(tag_link);
    }

    if (_sdf->HasElement("anchor_prefix"))
    {
      this->anchorPrefix = _sdf->Get<std::string>("anchor_prefix");
    }
    else
    {
      this->anchorPrefix = "uwb_anchor";
    }
    if (this->frequency == 0.0)
    {
      this->frequency = 2.5e9;
    }

    this->hasHideRangings = false;

    if (_sdf->HasElement("hide_rangings"))
    {
      this->hideRangingsPer = _sdf->Get<double>("hide_rangings");
      this->hasHideRangings = true;
      this->hideNoiseModel = sensors::NoiseFactory::NewNoiseModel(_sdf->GetElement("hide_noise"));
    }

    this->interfererPrefix = "interferer_";

    this->lastUpdateTime = common::Time(0.0);
    this->noiseModel = sensors::NoiseFactory::NewNoiseModel(_sdf->GetElement("noise"));
    

  if (_sdf->HasElement("anchors_noise"))
    {
    sensors::NoisePtr anchorsNoiseModel = sensors::NoiseFactory::NewNoiseModel(_sdf->GetElement("anchors_noise"));
    
    //TODO: get num anchors
    for (int i = 0; i < 6; ++i)
    {
      ignition::math::Vector3d anchorNoiseValue(anchorsNoiseModel->Apply(0),anchorsNoiseModel->Apply(0),anchorsNoiseModel->Apply(0));
      this->anchorNoises[i].Set(anchorNoiseValue.X(), anchorNoiseValue.Y(),anchorNoiseValue.Z());

      ROS_INFO("UWB Plugin Anchor: %d Noise (%f, %f, %f)", i, anchorNoiseValue.X(),anchorNoiseValue.Y(),anchorNoiseValue.Z());
    }
  }

    ros::NodeHandle n;
    this->gtecUwbPub = n.advertise<gtec_msgs::GenericRanging>("/gtec/gazebo/ranging", 1000);
    this->gtecAnchors = n.advertise<visualization_msgs::MarkerArray>("/gtec/gazebo/anchors", 1000);
    this->gtecMagneticInterferences = n.advertise<visualization_msgs::MarkerArray>("/gtec/gazebo/magnetic/interferences", 1000);
    this->testRay = boost::dynamic_pointer_cast<physics::RayShape>(
                      this->world->Physics()->CreateShape("ray", physics::CollisionPtr()));
    
    this->updateConnection =
      event::Events::ConnectWorldUpdateBegin(boost::bind(&UwbPlugin::OnUpdate, this, _1));
  }

public: void OnUpdate(const common::UpdateInfo &_info)
  {
    common::Time simTime = _info.simTime;
    common::Time elapsed = simTime - this->lastUpdateTime;
    double powerVar = 1.1297;
    if (elapsed >= this->updatePeriod)
    {
      this->lastUpdateTime = _info.simTime;
      ignition::math::Pose3d tagPose = this->tagLink->WorldPose();
      visualization_msgs::MarkerArray markerArray;
      visualization_msgs::MarkerArray interferencesArray;
      int interferenceId = 0;

      physics::Model_V models = this->world->Models();
      for (physics::Model_V::iterator iter = models.begin(); iter != models.end(); ++iter)
      {
        if ((*iter)->GetName().find(this->anchorPrefix) == 0) {
          physics::ModelPtr anchor = *iter;
          std::string aidStr = anchor->GetName().substr(this->anchorPrefix.length());
          int aid = std::stoi(aidStr);
          ignition::math::Pose3d anchorPose = anchor->WorldPose();

          //We add the anchor noise
          ignition::math::Pose3d anchorPoseWithNoise = anchor->WorldPose();
          ignition::math::Vector3d anchorNoise = this->anchorNoises[aid];
          ignition::math::Vector3d anchorPosition = anchorPoseWithNoise.Pos();
          anchorPoseWithNoise.Set(anchorPosition+anchorNoise, anchorPoseWithNoise.Rot());

        //ROS_INFO("UWB Plugin AnchorWithNoise: %d Noise (%f, %f, %f)", aid, anchorPoseWithNoise.Pos().X(),anchorPoseWithNoise.Pos().Y(),anchorPoseWithNoise.Pos().Z());
 

          // Comprobar obstáculos
          double distanceToObstacle;
          std::string obstacleName;
          double penalty = 0.0;

         // this->testRay->SetPoints(tagPose.Pos() + 0.5 * (anchorPose.Pos() - tagPose.Pos()), anchorPose.Pos());


          //this->testRay->SetPoints(tagPose.Pos() + 0.5 * (anchorPose.Pos() - tagPose.Pos()), anchorPose.Pos());

          ignition::math::Vector3d directionToAnchor = (anchorPoseWithNoise.Pos() - tagPose.Pos()).Normalize();
          this->testRay->SetPoints(tagPose.Pos() + directionToAnchor, anchorPoseWithNoise.Pos());

          this->testRay->GetIntersection(distanceToObstacle, obstacleName);
          if (obstacleName != "")
          {
            // penalty = distanceToObstacle;
          }

          double distance = tagPose.Pos().Distance(anchorPoseWithNoise.Pos()) + penalty;
          double distanceNoise = ApplyNoiseModel(distance);
          double rxPower = RxPowerEstimated(RxPower(distance))
                           + std::abs(ignition::math::Rand::DblNormal(0.0, powerVar));

          gtec_msgs::GenericRanging ranging_msg;

          ranging_msg.anchorId = aid;
          ranging_msg.tagId = 0;
          ranging_msg.range = distanceNoise * 1000;
          ranging_msg.seq = this->sequence;
          ranging_msg.rxPower = rxPower; // Realmente se espera en otro formato...

          if (this->hasHideRangings){
            double disVal = 4;
            for (int i = 0; i < 8; ++i)
            {
              if (hideRangingsPer==normalDis[i][0]){
                disVal = normalDis[i][1];
                break;
              }
            }

            double testVal = hideNoiseModel->Apply(0);
            if (testVal<disVal){
               this->gtecUwbPub.publish(ranging_msg);
            }

          } else {
            this->gtecUwbPub.publish(ranging_msg);
          }
          

          visualization_msgs::Marker marker;
          marker.header.frame_id = "map";
          marker.header.stamp = ros::Time();
          marker.id = aid;
          marker.type = visualization_msgs::Marker::CYLINDER;
          marker.action = visualization_msgs::Marker::ADD;
          marker.pose.position.x = anchorPose.Pos().X();
          marker.pose.position.y = anchorPose.Pos().Y();
          marker.pose.position.z = anchorPose.Pos().Z();
          marker.pose.orientation.x = anchorPose.Rot().X();
          marker.pose.orientation.y = anchorPose.Rot().Y();
          marker.pose.orientation.z = anchorPose.Rot().Z();
          marker.pose.orientation.w = anchorPose.Rot().W();
          marker.scale.x = 0.2;
          marker.scale.y = 0.2;
          marker.scale.z = 0.5;
          marker.color.a = 1.0;

          if (aid == 0) {
            marker.color.r = 0.6;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
          } else if (aid == 1) {
            marker.color.r = 0.6;
            marker.color.g = 0.2;
            marker.color.b = 0.0;
          } else if (aid == 2) {
            marker.color.r = 0.6;
            marker.color.g = 0.5;
            marker.color.b = 1.0;
          } else if (aid == 3) {
            marker.color.r = 0.6;
            marker.color.g = 0.8;
            marker.color.b = 0.0;
          } else if (aid == 4) {
            marker.color.b = 0.6;
            marker.color.g = 0.0;
            marker.color.r = 0.0;
          } else if (aid == 5) {
            marker.color.b = 0.6;
            marker.color.g = 0.2;
            marker.color.r = 0.0;
          } else if (aid == 6) {
            marker.color.b = 0.6;
            marker.color.g = 0.5;
            marker.color.r = 1.0;
          } else if (aid == 7) {
            marker.color.b = 0.6;
            marker.color.g = 0.8;
            marker.color.r = 0.0;
          }

          markerArray.markers.push_back(marker);
          //ROS_INFO("Anchor dectected at %f (%f dB, %f m)", distanceNoise, rxPower, distance);
        } else if ((*iter)->GetName().find(this->interfererPrefix) == 0) {

          physics::ModelPtr interferer = *iter;
          ignition::math::Pose3d interfererPose = interferer->WorldPose();

          visualization_msgs::Marker marker;
          marker.header.frame_id = "map";
          marker.header.stamp = ros::Time();
          marker.id = interferenceId;
          interferenceId+=1;
          marker.type = visualization_msgs::Marker::CUBE;
          marker.action = visualization_msgs::Marker::ADD;
          marker.pose.position.x = interfererPose.Pos().X();
          marker.pose.position.y = interfererPose.Pos().Y();
          marker.pose.position.z = interfererPose.Pos().Z();
          marker.pose.orientation.x = interfererPose.Rot().X();
          marker.pose.orientation.y = interfererPose.Rot().Y();
          marker.pose.orientation.z = interfererPose.Rot().Z();
          marker.pose.orientation.w = interfererPose.Rot().W();
          marker.scale.x = 0.2;
          marker.scale.y = 0.2;
          marker.scale.z = 0.2;
          marker.color.a = 1.0;
          marker.color.b = 0.1;
          marker.color.g = 0.9;
          marker.color.r = 1.0;

          interferencesArray.markers.push_back(marker);

        }

      }

      this->gtecAnchors.publish(markerArray);
      this->gtecMagneticInterferences.publish(interferencesArray);
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
    ROS_INFO("RESET IN UwbPlugin \n");
    this->lastUpdateTime = common::Time(0.0);
  }

private: double ApplyNoiseModel(double distance)
  {
    double distanceNoise = noiseModel->Apply(distance);
    double rxPower = RxPower(distance);

    distanceNoise += RxPowerToBias(rxPower);
  }

private: double RxPowerToBias(double rxPower)
  {
    if (rxPower > rxPowerVsBias[0][0])
    {
      return rxPowerVsBias[0][1];
    }
    int i = 0;
    while (rxPowerVsBias[i][0] > rxPower && i < 23)
    {
      i++;
    }
    if (rxPowerVsBias[i][0] > rxPower)
    {
      i--;
    }

    return rxPowerVsBias[i][1];
  }

private: double RxPowerEstimated(double realRxPower)
  {
    if (realRxPower < rxPowerEstimatedPrf16[0][0])
    {
      return realRxPower;
    }
    int i = 0;
    while (rxPowerEstimatedPrf16[i][0] < realRxPower && i < 39)
    {
      i++;
    }
    if (rxPowerEstimatedPrf16[i][0] < realRxPower)
    {
      i--;
    }

    return rxPowerEstimatedPrf16[i][1];
  }

private: double RxPower(double distance)
  {
    double rxPower = this->power + this->txGain + this->rxGain - 20 * log10(distance)
                     - 20 * log10(4 * M_PI * this->frequency / common::SpeedOfLight);
    return rxPower;
  }

private: physics::ModelPtr model;
private: physics::WorldPtr world;
private: physics::RayShapePtr testRay;
private: event::ConnectionPtr updateConnection;
private: common::Time updatePeriod;
private: common::Time lastUpdateTime;
private: sensors::NoisePtr noiseModel;
private: sensors::NoisePtr hideNoiseModel;
private: double power;
private: double txGain;
private: double rxGain;
private: double frequency;
private: std::string anchorPrefix;
private: std::string interfererPrefix;
private: physics::LinkPtr tagLink;
private: ros::Publisher gtecUwbPub;
private: ros::Publisher gtecAnchors;
private: ros::Publisher gtecMagneticInterferences;
private: unsigned char sequence;
private: ignition::math::Vector3d anchorNoises[10];
private: bool hasHideRangings;
private: double hideRangingsPer;
};

GZ_REGISTER_MODEL_PLUGIN(UwbPlugin)
}

