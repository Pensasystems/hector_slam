//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include "HectorMappingRos.h"

#include "map/GridMap.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include "sensor_msgs/PointCloud2.h"

#include "HectorDrawings.h"
#include "HectorDebugInfoProvider.h"
#include "HectorMapMutex.h"

#ifndef TF_SCALAR_H
  typedef btScalar tfScalar;
#endif

HectorMappingRos::HectorMappingRos()
  : debugInfoProvider(0)
  , hectorDrawings(0)
  , lastGetMapUpdateIndex(-100)
  , nodePaused_(true)
  , positionHold_(false)
  ,forward_way_(false)
  ,flag_test_(false)
  ,yaw_new_(0)
  , tfB_(0)
  , map__publish_thread_(0)
  ,yend_(0)
  ,xend_(0)
  ,current_pose_y_(0)
  ,previous_pose_y_(0)
  ,angle_y_(0.0872665)
  ,residual_x_(0)
  ,residual_y_(0)
  // ,vislamControl(false)
  , initial_pose_set_(false)
{
  ros::NodeHandle private_nh_("~");

  std::string mapTopic_ = "map";

  private_nh_.param("pub_drawings", p_pub_drawings, false);
  private_nh_.param("pub_debug_output", p_pub_debug_output_, false);
  private_nh_.param("pub_map_odom_transform", p_pub_map_odom_transform_,true);
  private_nh_.param("pub_odometry", p_pub_odometry_,false);
  private_nh_.param("advertise_map_service", p_advertise_map_service_,true);
  private_nh_.param("scan_subscriber_queue_size", p_scan_subscriber_queue_size_, 5);

  private_nh_.param("map_resolution", p_map_resolution_, 0.025);
  private_nh_.param("map_size", p_map_size_, 1024);
  private_nh_.param("map_start_x", p_map_start_x_, 0.5);
  private_nh_.param("map_start_y", p_map_start_y_, 0.5);
  private_nh_.param("map_multi_res_levels", p_map_multi_res_levels_, 3);

  private_nh_.param("update_factor_free", p_update_factor_free_, 0.4);
  private_nh_.param("update_factor_occupied", p_update_factor_occupied_, 0.9);

  private_nh_.param("map_update_distance_thresh", p_map_update_distance_threshold_, 0.4);
  private_nh_.param("map_update_angle_thresh", p_map_update_angle_threshold_, 0.9);

  private_nh_.param("scan_topic", p_scan_topic_, std::string("scan"));
  private_nh_.param("sys_msg_topic", p_sys_msg_topic_, std::string("syscommand"));
  private_nh_.param("pose_update_topic", p_pose_update_topic_, std::string("poseupdate"));

  private_nh_.param("use_tf_scan_transformation", p_use_tf_scan_transformation_,true);
  private_nh_.param("use_tf_pose_start_estimate", p_use_tf_pose_start_estimate_,false);
  private_nh_.param("map_with_known_poses", p_map_with_known_poses_, false);

  private_nh_.param("base_frame", p_base_frame_, std::string("base_link"));
  private_nh_.param("map_frame", p_map_frame_, std::string("map"));
  private_nh_.param("odom_frame", p_odom_frame_, std::string("odom"));

  private_nh_.param("pub_map_scanmatch_transform", p_pub_map_scanmatch_transform_,true);
  private_nh_.param("tf_map_scanmatch_transform_frame_name", p_tf_map_scanmatch_transform_frame_name_, std::string("scanmatcher_frame"));

  private_nh_.param("output_timing", p_timing_output_,false);

  private_nh_.param("map_pub_period", p_map_pub_period_, 2.0);

  double tmp = 0.0;
  private_nh_.param("laser_min_dist", tmp, 0.4);
  p_sqr_laser_min_dist_ = static_cast<float>(tmp*tmp);

  private_nh_.param("laser_max_dist", tmp, 30.0);
  p_sqr_laser_max_dist_ = static_cast<float>(tmp*tmp);

  private_nh_.param("laser_z_min_value", tmp, -1.0);
  p_laser_z_min_value_ = static_cast<float>(tmp);

  private_nh_.param("laser_z_max_value", tmp, 1.0);
  p_laser_z_max_value_ = static_cast<float>(tmp);

  if (p_pub_drawings)
  {
    ROS_INFO("HectorSM publishing debug drawings");
    hectorDrawings = new HectorDrawings();
  }

  // holdServiceServer_ = node_.advertiseService("position_hold", &HectorMappingRos::holdCallback, this);
  // pauseServiceServer_ = node_.advertiseService("pause", &HectorMappingRos::pauseCallback, this);

  holdServiceServer_ = node_.advertiseService("position_hold", &HectorMappingRos::holdCallback, this);

  pauseServiceServer_ = node_.advertiseService("pause", &HectorMappingRos::pauseCallback, this);

  mavrosPoseSub_ = node_.subscribe("/mavros/local_position/pose", 1,&HectorMappingRos::mavrosPoseCB, this);

  vislamOdomSub_ = node_.subscribe("/vislam/odometry", 1,&HectorMappingRos::vislamOdomCB, this);

  setpointSub_ = node_.subscribe("/mavros/setpoint_position/local", 1,&HectorMappingRos::setpointCB, this);

  mavrosPublisher_ = node_.advertise<nav_msgs::Odometry>("mavros/test", 50);
  // pauseServiceClient_ = node_.serviceClient<std_srvs::SetBool>("vilamwithCov_pause");	


  
  if(p_pub_debug_output_)
  {
    ROS_INFO("HectorSM publishing debug info");
    debugInfoProvider = new HectorDebugInfoProvider();
  }

  if(p_pub_odometry_)
  {
    odometryPublisher_ = node_.advertise<nav_msgs::Odometry>("posewithcov", 50);
  }

  slamProcessor = new hectorslam::HectorSlamProcessor(static_cast<float>(p_map_resolution_), p_map_size_, p_map_size_, Eigen::Vector2f(p_map_start_x_, p_map_start_y_), p_map_multi_res_levels_, hectorDrawings, debugInfoProvider);
  slamProcessor->setUpdateFactorFree(p_update_factor_free_);
  slamProcessor->setUpdateFactorOccupied(p_update_factor_occupied_);
  slamProcessor->setMapUpdateMinDistDiff(p_map_update_distance_threshold_);
  slamProcessor->setMapUpdateMinAngleDiff(p_map_update_angle_threshold_);

  int mapLevels = slamProcessor->getMapLevels();
  mapLevels = 1;

  for (int i = 0; i < mapLevels; ++i)
  {
    mapPubContainer.push_back(MapPublisherContainer());
    slamProcessor->addMapMutex(i, new HectorMapMutex());

    std::string mapTopicStr(mapTopic_);

    if (i != 0)
    {
      mapTopicStr.append("_" + boost::lexical_cast<std::string>(i));
    }

    std::string mapMetaTopicStr(mapTopicStr);
    mapMetaTopicStr.append("_metadata");

    MapPublisherContainer& tmp = mapPubContainer[i];
    tmp.mapPublisher_ = node_.advertise<nav_msgs::OccupancyGrid>(mapTopicStr, 1, true);
    tmp.mapMetadataPublisher_ = node_.advertise<nav_msgs::MapMetaData>(mapMetaTopicStr, 1, true);

    if ( (i == 0) && p_advertise_map_service_)
    {
      tmp.dynamicMapServiceServer_ = node_.advertiseService("dynamic_map", &HectorMappingRos::mapCallback, this);
    }

    setServiceGetMapData(tmp.map_, slamProcessor->getGridMap(i));

    if ( i== 0){
      mapPubContainer[i].mapMetadataPublisher_.publish(mapPubContainer[i].map_.map.info);
    }
  }

  ROS_INFO("HectorSM p_base_frame_: %s", p_base_frame_.c_str());
  ROS_INFO("HectorSM p_map_frame_: %s", p_map_frame_.c_str());
  ROS_INFO("HectorSM p_odom_frame_: %s", p_odom_frame_.c_str());
  ROS_INFO("HectorSM p_scan_topic_: %s", p_scan_topic_.c_str());
  ROS_INFO("HectorSM p_use_tf_scan_transformation_: %s", p_use_tf_scan_transformation_ ? ("true") : ("false"));
  ROS_INFO("HectorSM p_pub_map_odom_transform_: %s", p_pub_map_odom_transform_ ? ("true") : ("false"));
  ROS_INFO("HectorSM p_scan_subscriber_queue_size_: %d", p_scan_subscriber_queue_size_);
  ROS_INFO("HectorSM p_map_pub_period_: %f", p_map_pub_period_);
  ROS_INFO("HectorSM p_update_factor_free_: %f", p_update_factor_free_);
  ROS_INFO("HectorSM p_update_factor_occupied_: %f", p_update_factor_occupied_);
  ROS_INFO("HectorSM p_map_update_distance_threshold_: %f ", p_map_update_distance_threshold_);
  ROS_INFO("HectorSM p_map_update_angle_threshold_: %f", p_map_update_angle_threshold_);
  ROS_INFO("HectorSM p_laser_z_min_value_: %f", p_laser_z_min_value_);
  ROS_INFO("HectorSM p_laser_z_max_value_: %f", p_laser_z_max_value_);

  scanSubscriber_ = node_.subscribe(p_scan_topic_, p_scan_subscriber_queue_size_, &HectorMappingRos::scanCallback, this);
  sysMsgSubscriber_ = node_.subscribe(p_sys_msg_topic_, 1, &HectorMappingRos::sysMsgCallback, this);

  poseUpdatePublisher_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>(p_pose_update_topic_, 1, false);
  posePublisher_ = node_.advertise<geometry_msgs::PoseStamped>("slam_out_pose", 1, false);

  scan_point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud>("slam_cloud",1,false);

  tfB_ = new tf::TransformBroadcaster();
  ROS_ASSERT(tfB_);

  /*
  bool p_use_static_map_ = false;

  if (p_use_static_map_){
    mapSubscriber_ = node_.subscribe(mapTopic_, 1, &HectorMappingRos::staticMapCallback, this);
  }
  */

  initial_pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "initialpose", 2);
  initial_pose_filter_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(*initial_pose_sub_, tf_, p_map_frame_, 2);
  initial_pose_filter_->registerCallback(boost::bind(&HectorMappingRos::initialPoseCallback, this, _1));


  map__publish_thread_ = new boost::thread(boost::bind(&HectorMappingRos::publishMapLoop, this, p_map_pub_period_));

  map_to_odom_.setIdentity();

  lastMapPublishTime = ros::Time(0,0);

  positionHoldTimer_ = private_nh_.createTimer(ros::Duration(0.2),
											   &HectorMappingRos::publishHeldPosition,
											   this,
											   false,
											   false);
}

HectorMappingRos::~HectorMappingRos()
{
  delete slamProcessor;

  if (hectorDrawings)
    delete hectorDrawings;

  if (debugInfoProvider)
    delete debugInfoProvider;

  if (tfB_)
    delete tfB_;

  if(map__publish_thread_)
    delete map__publish_thread_;
}

void HectorMappingRos::publishHeldPosition(const ros::TimerEvent& e)
{
  ROS_INFO(" stop.............................................Start");
  //if (nodePaused_){
	  lastOdomMsg_.header.stamp = ros::Time::now();
	  lastScanMatchTf_.stamp_ = ros::Time::now();
	  lastMapToOdomTf_.stamp_ = ros::Time::now();
	  // odometryPublisher_.publish(lastOdomMsg_);
	  // tfB_->sendTransform( lastScanMatchTf_);
	  // tfB_->sendTransform( lastMapToOdomTf_ );
    if (nodePaused_ || positionHold_) {
      lastOdomMsg_.pose.pose.position.x += residual_x_;
      lastOdomMsg_.pose.pose.position.y += residual_y_;

      std::cout<< "update last msg"<<std::cout;


      std::cout<< residual_x_<<std::cout;
      std::cout<< residual_y_<<std::cout;
      yend_=lastOdomMsg_.pose.pose.position.y;
      xend_=lastOdomMsg_.pose.pose.position.x;
    }

    // lastOdomMsg_.pose.pose.position.x = currentPose_.pose.position.x;
	  // lastOdomMsg_.pose.pose.position.y = currentPose_.pose.position.y;
    // lastOdomMsg_.pose.pose.orientation.z = currentPose_.pose.orientation.z;
    // lastOdomMsg_.pose.pose.orientation.w = currentPose_.pose.orientation.w;
    // lastOdomMsg_.pose.covariance [0] = .000003;
    // lastOdomMsg_.pose.covariance[7] =  .000001;
    // lastOdomMsg_.pose.covariance[14] = .0000032;
    // lastOdomMsg_.pose.covariance[21] = .000002;
    // lastOdomMsg_.pose.covariance[28] = .0000014;
    // lastOdomMsg_.pose.covariance[35] = .0000021;

    lastOdomMsg_.pose.covariance [0] = vislamOdom_.pose.covariance [0];
    lastOdomMsg_.pose.covariance [7] =  vislamOdom_.pose.covariance [7];
    lastOdomMsg_.pose.covariance [14] = vislamOdom_.pose.covariance [14];
    lastOdomMsg_.pose.covariance [21] = vislamOdom_.pose.covariance [21];
    lastOdomMsg_.pose.covariance [28] = vislamOdom_.pose.covariance [28];
    lastOdomMsg_.pose.covariance [35] = vislamOdom_.pose.covariance [35];
    lastOdomMsg_.child_frame_id = p_base_frame_;
    lastOdomMsg_.header.frame_id= p_map_frame_;
    // lastOdomMsg_.header.stamp = ros::Time::now();
	  // lastScanMatchTf_.stamp_ = ros::Time::now();
	  // lastMapToOdomTf_.stamp_ = ros::Time::now();
    odometryPublisher_.publish(lastOdomMsg_);
    ROS_INFO(" stop...............................................");


 // }
}

void HectorMappingRos::scanCallback(const sensor_msgs::LaserScan& scan)
{
  if (nodePaused_ || positionHold_) {
    // vislamControl(true); 
      double qx=0;
      double qy=0;
      double qz=0;
      double qw=0;
      qx= currentPose_.pose.orientation.x;
      qy= currentPose_.pose.orientation.y;
      qz= currentPose_.pose.orientation.z;
      qw= currentPose_.pose.orientation.w;
      
    // initial_pose_.pose.position.x=currentPose_.pose.position.x;
    //initial_pose_.pose.position.y=currentPose_.pose.position.x;
        double siny_cosp = 2 * (qw * qz + qx * qy);
       double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
       //double yaw_new_ = std::atan2(siny_cosp, cosy_cosp);
       //double yaw_new=tf::getYaw(getRotation(qx,qy,qz,qw));
       
    initial_pose_ = Eigen::Vector3f(currentPose_.pose.position.x, currentPose_.pose.position.y,yaw_new_);
    initial_pose_set_ = true;
    ROS_INFO(" I am initiated");
    nav_msgs::Odometry lastOdomMsg_;
    if (!flag_test_){
      lastOdomMsg_.pose.pose.position.x = vislamOdom_.pose.pose.position.x;
      lastOdomMsg_.pose.pose.position.y = vislamOdom_.pose.pose.position.y;
      lastOdomMsg_.pose.pose.orientation.z = vislamOdom_.pose.pose.orientation.z;
      lastOdomMsg_.pose.pose.orientation.w = vislamOdom_.pose.pose.orientation.w;
      ROS_INFO(" I am initiated with vislam");
   
    // else if (flag_test_){
    //   lastOdomMsg_.pose.pose.position.x = currentPose_.pose.position.x;
    //   lastOdomMsg_.pose.pose.position.y = currentPose_.pose.position.y;
    //   lastOdomMsg_.pose.pose.orientation.z = currentPose_.pose.orientation.z;
    //   lastOdomMsg_.pose.pose.orientation.w = currentPose_.pose.orientation.w;
    //   ROS_INFO(" I am initiated with Mavros");
    // }
    lastOdomMsg_.pose.covariance [0] = vislamOdom_.pose.covariance [0];
    lastOdomMsg_.pose.covariance [7] =  vislamOdom_.pose.covariance [7];
    lastOdomMsg_.pose.covariance [14] = vislamOdom_.pose.covariance [14];
    lastOdomMsg_.pose.covariance [21] = vislamOdom_.pose.covariance [21];
    lastOdomMsg_.pose.covariance [28] = vislamOdom_.pose.covariance [28];
    lastOdomMsg_.pose.covariance [35] = vislamOdom_.pose.covariance [35];
    lastOdomMsg_.child_frame_id = p_base_frame_;
    lastOdomMsg_.header.frame_id= p_map_frame_;
    lastOdomMsg_.header.stamp = ros::Time::now();
	  lastScanMatchTf_.stamp_ = ros::Time::now();
	  lastMapToOdomTf_.stamp_ = ros::Time::now();

    odometryPublisher_.publish(lastOdomMsg_);
    }
    ROS_INFO(" stop...............................................");


    return;
  }
	
  if (hectorDrawings)
  {
    hectorDrawings->setTime(scan.header.stamp);
  }

  ros::WallTime startTime = ros::WallTime::now();

  if (!p_use_tf_scan_transformation_)
  {
    if (rosLaserScanToDataContainer(scan, laserScanContainer,slamProcessor->getScaleToMap()))
    {
      slamProcessor->update(laserScanContainer,slamProcessor->getLastScanMatchPose());
    }
  }
  else
  {
    ros::Duration dur (0.5);

    if (tf_.waitForTransform(p_base_frame_,scan.header.frame_id, scan.header.stamp,dur))
    {
      tf::StampedTransform laserTransform;
      tf_.lookupTransform(p_base_frame_,scan.header.frame_id, scan.header.stamp, laserTransform);

      //projector_.transformLaserScanToPointCloud(p_base_frame_ ,scan, pointCloud,tf_);
      projector_.projectLaser(scan, laser_point_cloud_,30.0);

      if (scan_point_cloud_publisher_.getNumSubscribers() > 0){
        scan_point_cloud_publisher_.publish(laser_point_cloud_);
      }

      Eigen::Vector3f startEstimate(Eigen::Vector3f::Zero());

      if(rosPointCloudToDataContainer(laser_point_cloud_, laserTransform, laserScanContainer, slamProcessor->getScaleToMap()))
      {
        if (initial_pose_set_){
          initial_pose_set_ = false;
          startEstimate = initial_pose_;
        }else if (p_use_tf_pose_start_estimate_){

          try
          {
            tf::StampedTransform stamped_pose;

            tf_.waitForTransform(p_map_frame_,p_base_frame_, scan.header.stamp, ros::Duration(0.5));
            tf_.lookupTransform(p_map_frame_, p_base_frame_,  scan.header.stamp, stamped_pose);

            tfScalar yaw, pitch, roll;
            stamped_pose.getBasis().getEulerYPR(yaw, pitch, roll);

            startEstimate = Eigen::Vector3f(stamped_pose.getOrigin().getX(),stamped_pose.getOrigin().getY(), yaw);
          }
          catch(tf::TransformException e)
          {
            ROS_ERROR("Transform from %s to %s failed\n", p_map_frame_.c_str(), p_base_frame_.c_str());
            startEstimate = slamProcessor->getLastScanMatchPose();
          }

        }else{
          startEstimate = slamProcessor->getLastScanMatchPose();
        }


        if (p_map_with_known_poses_){
          slamProcessor->update(laserScanContainer, startEstimate, true);
        }else{
          slamProcessor->update(laserScanContainer, startEstimate);
        }
      }

    }else{
      ROS_INFO("lookupTransform %s to %s timed out. Could not transform laser scan into base_frame.", p_base_frame_.c_str(), scan.header.frame_id.c_str());
      return;
    }
  }

  if (p_timing_output_)
  {
    ros::WallDuration duration = ros::WallTime::now() - startTime;
    ROS_INFO("HectorSLAM Iter took: %f milliseconds", duration.toSec()*1000.0f );
  }

  //If we're just building a map with known poses, we're finished now. Code below this point publishes the localization results.
  if (p_map_with_known_poses_)
  {
    return;
  }

  poseInfoContainer_.update(slamProcessor->getLastScanMatchPose(), slamProcessor->getLastScanMatchCovariance(), scan.header.stamp, p_map_frame_);

  poseUpdatePublisher_.publish(poseInfoContainer_.getPoseWithCovarianceStamped());
  posePublisher_.publish(poseInfoContainer_.getPoseStamped());

  if(p_pub_odometry_)
  {
    nav_msgs::Odometry tmp;
    tmp.pose = poseInfoContainer_.getPoseWithCovarianceStamped().pose;
    // tmp.pose.pose.position.x=(((tmp.pose.pose.position.x))*cos(angle_y_))-(((tmp.pose.pose.position.y)-(yend_))*sin(angle_y_));
    // tmp.pose.pose.position.y=(((tmp.pose.pose.position.x)-(xend_))*sin(angle_y_))+(((tmp.pose.pose.position.y))*cos(angle_y_));
    tmp.header = poseInfoContainer_.getPoseWithCovarianceStamped().header;
    tmp.child_frame_id = p_base_frame_;
        for(int16_t i=0;i<6;i++){
        for(int16_t j=0; j<6; j++){
              tmp.pose.covariance[ i*6 + j ] =  0.0000001*tmp.pose.covariance[ i*6 + j ];
        }
    }
    ROS_INFO("Before condition");
    //Ros_INFO(currentPose_.pose.position.x);
    //std::cout<<fabs(currentPose_.pose.position.x)<< std::endl;
    //std::cout<<fabs(currentPose_.pose.position.y)<< std::endl;


    //if ( (1.5 < (fabs(currentPose_.pose.position.y))) || (flag_test_)) {
      //flag_test_==true;
      //ROS_INFO("1.5 < (fabs(currentPose_.pose.position.x)) |  1.5 < (fabs(currentPose_.pose.position.x))");
      // if ( 2.05 > (fabs(currentPose_.pose.position.x)) ||  2.05 > (fabs(currentPose_.pose.position.y))){
      //   ROS_INFO("( 2.05 > (fabs(currentPose_.pose.position.x)) |  2.05 > (fabs(currentPose_.pose.position.x)))");
        /// check fo x
        if ((fabs(lastOdomMsg_.pose.pose.position.x-tmp.pose.pose.position.x)) > 0.10 ){
          
          tmp.pose.pose.position.x = lastOdomMsg_.pose.pose.position.x+(tanh((lastOdomMsg_.pose.pose.position.x-tmp.pose.pose.position.x)))*0.05;
        }
        else
        {
          tmp.pose.pose.position.x=tmp.pose.pose.position.x;
        }//end check for x
        // ///check for y
        if ((fabs(lastOdomMsg_.pose.pose.position.y-tmp.pose.pose.position.y)) > 0.10) {
          
          tmp.pose.pose.position.y = lastOdomMsg_.pose.pose.position.x + (tanh((lastOdomMsg_.pose.pose.position.y-tmp.pose.pose.position.y)))*0.05;
        }
        else
        {
          tmp.pose.pose.position.y=tmp.pose.pose.position.y;
        }//end check for y
        // odometryPublisher_.publish(tmp);
	      // lastOdomMsg_ = tmp;
      // }
      
      flag_test_=true;
      odometryPublisher_.publish(tmp);
	    lastOdomMsg_ = tmp;
      ///pause
      //vislamControl(false);
      //forward_way_(false);
    //   if ( 2.1< (fabs(currentPose_.pose.position.x)) ||  2.1 < (fabs(currentPose_.pose.position.y)) ) {
    //     // forward_way_=true;
    //     //flag_test_==false;
    //     vislamControl(false);

    // }
    // }

    // else if (forward_way_){
    //   //vislamControl(true);
    //   ROS_INFO("condition is set");

    // }
    // odometryPublisher_.publish(tmp);
	  // lastOdomMsg_ = tmp;
  }

  if (p_pub_map_odom_transform_)
  {
    tf::StampedTransform odom_to_base;

    try
    {
      tf_.waitForTransform(p_odom_frame_, p_base_frame_, scan.header.stamp, ros::Duration(0.5));
      tf_.lookupTransform(p_odom_frame_, p_base_frame_, scan.header.stamp, odom_to_base);
    }
    catch(tf::TransformException e)
    {
      ROS_ERROR("Transform failed during publishing of map_odom transform: %s",e.what());
      odom_to_base.setIdentity();
    }
    map_to_odom_ = tf::Transform(poseInfoContainer_.getTfTransform() * odom_to_base.inverse());
	lastMapToOdomTf_ = tf::StampedTransform(map_to_odom_, scan.header.stamp, p_map_frame_, p_odom_frame_);
    tfB_->sendTransform( lastMapToOdomTf_ );
  }

  if (p_pub_map_scanmatch_transform_){
	  lastScanMatchTf_ = tf::StampedTransform(poseInfoContainer_.getTfTransform(), scan.header.stamp, p_map_frame_, p_tf_map_scanmatch_transform_frame_name_);
	  tfB_->sendTransform( lastScanMatchTf_); 
  }
}

void HectorMappingRos::sysMsgCallback(const std_msgs::String& string)
{

  std::cout<<"try to reset"<<std::endl;
  ROS_INFO("HectorSM sysMsgCallback, msg contents: %s", string.data.c_str());

  if (string.data == "reset")
  {
    ROS_INFO("HectorSM reset");
    slamProcessor->reset();
  }
}

bool HectorMappingRos::holdCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& rsp)
{
	// Perhaps easier to do this from the user side. And there might be a case where we want this behavior.
	//if (nodePaused_)
	//{
	//	rsp.success = false;
	//	return false;
	//}
	
	if (positionHold_ != req.data)
	{
		positionHold_ = req.data;
		rsp.success = true;

		if (positionHold_)
		{
			positionHoldTimer_.setPeriod(ros::Duration(0.2), true);
			positionHoldTimer_.start();
		}
		else
		{
			positionHoldTimer_.stop();
		}
	}
	return true;
}
    



  void HectorMappingRos::mavrosPoseCB (const geometry_msgs::PoseStampedConstPtr& msg)
  {
    currentPose_= *msg;
    currentPose_.header.stamp = ros::Time::now();
	  // currentPose_.stamp_ = ros::Time::now();
	  // currentPose_.stamp_ = ros::Time::now();
      //ROS_INFO("........");

      if (  (1.4 <= (fabs(currentPose_.pose.position.y) ))) {
        if (   (1.6 > (fabs(currentPose_.pose.position.y))) ){
          
          ROS_INFO("Flag set");
          // std::cout<<flag_test_<<std::endl;
          std::cout<<fabs(currentPose_.pose.position.x)<< std::endl;
          std::cout<<fabs(currentPose_.pose.position.y)<< std::endl;




        }
        
      }

    tf::Pose pose;
    tf::poseMsgToTF(msg->pose, pose);
    yaw_new_=tf::getYaw(pose.getRotation());
  }


  void HectorMappingRos::setpointCB (const geometry_msgs::PoseStampedPtr& msg)
  {
    
    current_pose_y_=(msg->pose.position.y);

    if (fabs(current_pose_y_) > fabs(previous_pose_y_)){
      angle_y_= angle_y_;
    }
    else if(fabs(current_pose_y_) < fabs(previous_pose_y_)){
      angle_y_ = -1 * angle_y_;
    }
    

    previous_pose_y_=current_pose_y_;



  

  
        
  }

  void HectorMappingRos::vislamOdomCB (const nav_msgs::OdometryPtr& msg)
  {

    residual_x_=vislamOdom_.pose.pose.position.x-(msg->pose.pose.position.x);
    residual_y_=vislamOdom_.pose.pose.position.y-(msg->pose.pose.position.y);
    vislamOdom_= *msg;
    vislamOdom_.header.stamp = ros::Time::now();

	  // currentPose_.stamp_ = ros::Time::now();
	  // currentPose_.stamp_ = ros::Time::now();
      //ROS_INFO("........");

     mavrosPublisher_.publish(vislamOdom_);  
        
  }




    // if (   (0< (fabs(currentPose_.pose.position.y))) ){

    //   nodePaused_ = false;
    // }

  

bool HectorMappingRos::pauseCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& rsp)
{
	if (nodePaused_ != req.data)
	{
		nodePaused_ = req.data;
		//if (nodePaused_) slamProcessor->reset();
		rsp.success = true;
	}

  if (nodePaused_){
    slamProcessor->reset();
  }
	return true;
}

bool HectorMappingRos::mapCallback(nav_msgs::GetMap::Request  &req,
                                   nav_msgs::GetMap::Response &res)
{
  ROS_INFO("HectorSM Map service called");
  res = mapPubContainer[0].map_;
  return true;
}

void HectorMappingRos::publishMap(MapPublisherContainer& mapPublisher, const hectorslam::GridMap& gridMap, ros::Time timestamp, MapLockerInterface* mapMutex)
{
  nav_msgs::GetMap::Response& map_ (mapPublisher.map_);

  //only update map if it changed
  if (lastGetMapUpdateIndex != gridMap.getUpdateIndex())
  {

    int sizeX = gridMap.getSizeX();
    int sizeY = gridMap.getSizeY();

    int size = sizeX * sizeY;

    std::vector<int8_t>& data = map_.map.data;

    //std::vector contents are guaranteed to be contiguous, use memset to set all to unknown to save time in loop
    memset(&data[0], -1, sizeof(int8_t) * size);

    if (mapMutex)
    {
      mapMutex->lockMap();
    }

    for(int i=0; i < size; ++i)
    {
      if(gridMap.isFree(i))
      {
        data[i] = 0;
      }
      else if (gridMap.isOccupied(i))
      {
        data[i] = 100;
      }
    }

    lastGetMapUpdateIndex = gridMap.getUpdateIndex();

    if (mapMutex)
    {
      mapMutex->unlockMap();
    }
  }

  map_.map.header.stamp = timestamp;

  mapPublisher.mapPublisher_.publish(map_.map);
}

bool HectorMappingRos::rosLaserScanToDataContainer(const sensor_msgs::LaserScan& scan, hectorslam::DataContainer& dataContainer, float scaleToMap)
{
  size_t size = scan.ranges.size();

  float angle = scan.angle_min;

  dataContainer.clear();

  dataContainer.setOrigo(Eigen::Vector2f::Zero());

  float maxRangeForContainer = scan.range_max - 0.1f;

  for (size_t i = 0; i < size; ++i)
  {
    float dist = scan.ranges[i];

    if ( (dist > scan.range_min) && (dist < maxRangeForContainer))
    {
      dist *= scaleToMap;
      dataContainer.add(Eigen::Vector2f(cos(angle) * dist, sin(angle) * dist));
    }

    angle += scan.angle_increment;
  }

  return true;
}

bool HectorMappingRos::rosPointCloudToDataContainer(const sensor_msgs::PointCloud& pointCloud, const tf::StampedTransform& laserTransform, hectorslam::DataContainer& dataContainer, float scaleToMap)
{
  size_t size = pointCloud.points.size();
  //ROS_INFO("size: %d", size);

  dataContainer.clear();

  tf::Vector3 laserPos (laserTransform.getOrigin());
  dataContainer.setOrigo(Eigen::Vector2f(laserPos.x(), laserPos.y())*scaleToMap);

  for (size_t i = 0; i < size; ++i)
  {

    const geometry_msgs::Point32& currPoint(pointCloud.points[i]);

    float dist_sqr = currPoint.x*currPoint.x + currPoint.y* currPoint.y;

    if ( (dist_sqr > p_sqr_laser_min_dist_) && (dist_sqr < p_sqr_laser_max_dist_) ){

      if ( (currPoint.x < 0.0f) && (dist_sqr < 0.50f)){
        continue;
      }

      tf::Vector3 pointPosBaseFrame(laserTransform * tf::Vector3(currPoint.x, currPoint.y, currPoint.z));

      float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos.z();

      if (pointPosLaserFrameZ > p_laser_z_min_value_ && pointPosLaserFrameZ < p_laser_z_max_value_)
      {
        dataContainer.add(Eigen::Vector2f(pointPosBaseFrame.x(),pointPosBaseFrame.y())*scaleToMap);
      }
    }
  }

  return true;
}

void HectorMappingRos::setServiceGetMapData(nav_msgs::GetMap::Response& map_, const hectorslam::GridMap& gridMap)
{
  Eigen::Vector2f mapOrigin (gridMap.getWorldCoords(Eigen::Vector2f::Zero()));
  mapOrigin.array() -= gridMap.getCellLength()*0.5f;

  map_.map.info.origin.position.x = mapOrigin.x();
  map_.map.info.origin.position.y = mapOrigin.y();
  map_.map.info.origin.orientation.w = currentPose_.pose.orientation.w;

  

  map_.map.info.resolution = gridMap.getCellLength();

  map_.map.info.width = gridMap.getSizeX();
  map_.map.info.height = gridMap.getSizeY();

  map_.map.header.frame_id = p_map_frame_;
  map_.map.data.resize(map_.map.info.width * map_.map.info.height);
}

/*
void HectorMappingRos::setStaticMapData(const nav_msgs::OccupancyGrid& map)
{
  float cell_length = map.info.resolution;
  Eigen::Vector2f mapOrigin (map.info.origin.position.x + cell_length*0.5f,
                             map.info.origin.position.y + cell_length*0.5f);

  int map_size_x = map.info.width;
  int map_size_y = map.info.height;

  slamProcessor = new hectorslam::HectorSlamProcessor(cell_length, map_size_x, map_size_y, Eigen::Vector2f(0.0f, 0.0f), 1, hectorDrawings, debugInfoProvider);
}
*/
// void HectorMappingRos::vislamControl(bool pauseVislam)
// {

// 	std_srvs::SetBool msg;
// 	msg.request.data = pauseVislam;
// 	std::string output = "Hector SM: position hold " + std::string(msg.request.data ? "start" : "end");
// 	if (!pauseServiceClient_.call(msg)) ROS_ERROR_STREAM(output.c_str() << " error");
// 	else ROS_DEBUG_STREAM(output);


// }

void HectorMappingRos::publishMapLoop(double map_pub_period)
{
  ros::Rate r(1.0 / map_pub_period);
  while(ros::ok())
  {
    //ros::WallTime t1 = ros::WallTime::now();
    ros::Time mapTime (ros::Time::now());
    //publishMap(mapPubContainer[2],slamProcessor->getGridMap(2), mapTime);
    //publishMap(mapPubContainer[1],slamProcessor->getGridMap(1), mapTime);
    publishMap(mapPubContainer[0],slamProcessor->getGridMap(0), mapTime, slamProcessor->getMapMutex(0));

    //ros::WallDuration t2 = ros::WallTime::now() - t1;

    //std::cout << "time s: " << t2.toSec();
    //ROS_INFO("HectorSM ms: %4.2f", t2.toSec()*1000.0f);

    r.sleep();
  }
}

void HectorMappingRos::staticMapCallback(const nav_msgs::OccupancyGrid& map)
{

}

void HectorMappingRos::initialPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  // Only accept an initial pose when the node is paused. Otherwise, this becomes an online pose reset.
  if (!nodePaused_ && !positionHold_) return;
  
  tf::Pose pose;
  tf::poseMsgToTF(msg->pose, pose);
  initial_pose_ = Eigen::Vector3f(msg->pose.position.x, msg->pose.position.y, tf::getYaw(pose.getRotation()));
  ROS_INFO("Setting initial pose with world coords x: %f y: %f yaw: %f", initial_pose_[0], initial_pose_[1], initial_pose_[2]);
  initial_pose_set_ = true;

  if (positionHold_)
  {
	  // Note: this does not change the transforms!!!
	  lastOdomMsg_.pose.pose.orientation = msg->pose.orientation;
  }
}

