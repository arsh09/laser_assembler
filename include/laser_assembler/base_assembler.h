/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

//! \author Vijay Pradeep

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "message_filters/subscriber.h"

#include <deque>

// Service
#include "laser_assembler/AssembleScans.h"
#include "laser_assembler/AssembleScans2.h"

#include "boost/thread.hpp"
#include "math.h"

// PCL 
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include "pcl/filters/voxel_grid.h" //for downsampling the point cloud
#include "pcl/kdtree/kdtree_flann.h" //for the kdtree
#include "pcl/registration/transforms.h" //for the transformation function
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>

namespace laser_assembler
{

bool pclSort (pcl::PointXYZINormal i, pcl::PointXYZINormal j)
{
  return (i.x < j.x);
}

bool pclUnique (pcl::PointXYZINormal i, pcl::PointXYZINormal j)
{
  double x_diff = fabs(i.x - j.x);
  double y_diff = fabs(i.y - j.y);
  double z_diff = fabs(i.z - j.z);
  if(x_diff < 0.0001 && y_diff < 0.0001 && z_diff < 0.0001 )
    return true;
  else
    return false;
}

/**
 * \brief Maintains a history of point clouds and generates an aggregate point cloud upon request
 */
template<class T>
class BaseAssembler
{
public:
  BaseAssembler(const std::string& max_size_param_name);
  ~BaseAssembler() ;

  /**
   * \brief Tells the assembler to start listening to input data
   * It is possible that a derived class needs to initialize and configure
   * various things before actually beginning to process scans. Calling start
   * create subcribes to the input stream, thus allowing the scanCallback and
   * ConvertToCloud to be called. Start should be called only once.
   */
  void start(const std::string& in_topic_name);
  void start();


  /** \brief Returns the number of points in the current scan
   * \param scan The scan for for which we want to know the number of points
   * \return the number of points in scan
   */
  virtual unsigned int GetPointsInScan(const T& scan) = 0 ;

  /** \brief Converts the current scan into a cloud in the specified fixed frame
   *
   * Note: Once implemented, ConvertToCloud should NOT catch TF exceptions. These exceptions are caught by
   * BaseAssembler, and will be counted for diagnostic information
   * \param fixed_frame_id The name of the frame in which we want cloud_out to be in
   * \param scan_in The scan that we want to convert
   * \param cloud_out The result of transforming scan_in into a cloud in frame fixed_frame_id
   */
  virtual void ConvertToCloud(const std::string& fixed_frame_id, const T& scan_in, sensor_msgs::PointCloud& cloud_out) = 0 ;

protected:
  tf::TransformListener* tf_ ;
  tf::MessageFilter<T>* tf_filter_;

  ros::NodeHandle private_ns_;
  ros::NodeHandle n_;

private:
  // ROS Input/Ouptut Handling
  ros::ServiceServer build_cloud_server_;
  ros::ServiceServer assemble_scans_server_;
  ros::ServiceServer build_cloud_server2_;
  ros::ServiceServer assemble_scans_server2_;
  ros::ServiceServer merge_scans_server2_;
  message_filters::Subscriber<T> scan_sub_;
  message_filters::Connection tf_filter_connection_;

  //! \brief Callback function for every time we receive a new scan
  //void scansCallback(const tf::MessageNotifier<T>::MessagePtr& scan_ptr, const T& testA)
  virtual void msgCallback(const boost::shared_ptr<const T>& scan_ptr) ;

  //! \brief Service Callback function called whenever we need to build a cloud
  bool buildCloud(AssembleScans::Request& req, AssembleScans::Response& resp) ;
  bool assembleScans(AssembleScans::Request& req, AssembleScans::Response& resp) ;
  bool buildCloud2(AssembleScans2::Request& req, AssembleScans2::Response& resp) ;
  bool assembleScans2(AssembleScans2::Request& req, AssembleScans2::Response& resp) ;

  // use PCL for ICP and downsampling
  bool mergeScanICP(AssembleScans2::Request& req, AssembleScans2::Response& resp) ;
  bool mergeScans2(AssembleScans2::Request& req, AssembleScans2::Response& resp) ;

  //! \brief Stores history of scans
  std::deque<sensor_msgs::PointCloud> scan_hist_ ;
  boost::mutex scan_hist_mutex_ ;

  //! \brief The number points currently in the scan history
  unsigned int total_pts_ ;

  //! \brief The max number of scans to store in the scan history
  unsigned int max_scans_ ;

  //! \brief The frame to transform data into upon receipt
  std::string fixed_frame_ ;

  //! \brief Specify how much to downsample the data. A value of 1 preserves all the data. 3 would keep 1/3 of the data.
  unsigned int downsample_factor_ ;


  // pointcloud merger 
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree_;  // for kdtree
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud2_current_, pointcloud2_merged_, pointcloud2_transformed_;
  pcl::PointCloud<pcl::PointXYZRGB> convertFromMsgToPointCloud(const sensor_msgs::PointCloud& pointcloud_msg);
  bool getOverlapTransformation(void);
  void convertToMesh(void);
} ;

template <class T>
BaseAssembler<T>::BaseAssembler(const std::string& max_size_param_name) : private_ns_("~")
{
  // **** Initialize TransformListener ****
  double tf_cache_time_secs ;
  private_ns_.param("tf_cache_time_secs", tf_cache_time_secs, 10.0) ;
  if (tf_cache_time_secs < 0)
    ROS_ERROR("Parameter tf_cache_time_secs<0 (%f)", tf_cache_time_secs) ;

  tf_ = new tf::TransformListener(n_, ros::Duration(tf_cache_time_secs));
  ROS_INFO("TF Cache Time: %f Seconds", tf_cache_time_secs) ;

  // ***** Set max_scans *****
  const int default_max_scans = 400 ;
  int tmp_max_scans ;
  private_ns_.param(max_size_param_name, tmp_max_scans, default_max_scans);
  if (tmp_max_scans < 0)
  {
    ROS_ERROR("Parameter max_scans<0 (%i)", tmp_max_scans) ;
    tmp_max_scans = default_max_scans ;
  }
  max_scans_ = tmp_max_scans ;
  ROS_INFO("Max Scans in History: %u", max_scans_) ;
  total_pts_ = 0 ;    // We're always going to start with no points in our history

  // ***** Set fixed_frame *****
  private_ns_.param("fixed_frame", fixed_frame_, std::string("ERROR_NO_NAME"));
  ROS_INFO("Fixed Frame: %s", fixed_frame_.c_str()) ;
  if (fixed_frame_ == "ERROR_NO_NAME")
    ROS_ERROR("Need to set parameter fixed_frame") ;

  // ***** Set downsample_factor *****
  int tmp_downsample_factor ;
  private_ns_.param("downsample_factor", tmp_downsample_factor, 1);
  if (tmp_downsample_factor < 1)
  {
    ROS_ERROR("Parameter downsample_factor<1: %i", tmp_downsample_factor) ;
    tmp_downsample_factor = 1 ;
  }
  downsample_factor_ = tmp_downsample_factor ;
  if (downsample_factor_ != 1)
    ROS_WARN("Downsample set to [%u]. Note that this is an unreleased/unstable feature", downsample_factor_);

  // ***** Start Services *****
  build_cloud_server_    = n_.advertiseService("build_cloud",    &BaseAssembler<T>::buildCloud,    this);
  assemble_scans_server_ = n_.advertiseService("assemble_scans", &BaseAssembler<T>::assembleScans, this);
  build_cloud_server2_    = n_.advertiseService("build_cloud2",    &BaseAssembler<T>::buildCloud2,    this);
  assemble_scans_server2_ = n_.advertiseService("assemble_scans2", &BaseAssembler<T>::assembleScans2, this);
  merge_scans_server2_ =   n_.advertiseService("merge_scans2",  &BaseAssembler<T>::mergeScans2, this);


  // ***** Start Listening to Data *****
  // (Well, don't start listening just yet. Keep this as null until we actually start listening, when start() is called)
  tf_filter_ = NULL;

}

template <class T>
void BaseAssembler<T>::start(const std::string& in_topic_name)
{
  ROS_DEBUG("Called start(string). Starting to listen on message_filter::Subscriber the input stream");
  if (tf_filter_)
    ROS_ERROR("assembler::start() was called twice!. This is bad, and could leak memory") ;
  else
  {
    scan_sub_.subscribe(n_, in_topic_name, 10);
    tf_filter_ = new tf::MessageFilter<T>(scan_sub_, *tf_, fixed_frame_, 10);
    tf_filter_->registerCallback( boost::bind(&BaseAssembler<T>::msgCallback, this, boost::placeholders::_1) );
  }
}

template <class T>
void BaseAssembler<T>::start()
{
  ROS_DEBUG("Called start(). Starting tf::MessageFilter, but not initializing Subscriber");
  if (tf_filter_)
    ROS_ERROR("assembler::start() was called twice!. This is bad, and could leak memory") ;
  else
  {
    scan_sub_.subscribe(n_, "bogus", 10);
    tf_filter_ = new tf::MessageFilter<T>(scan_sub_, *tf_, fixed_frame_, 10);
    tf_filter_->registerCallback( boost::bind(&BaseAssembler<T>::msgCallback, this, boost::placeholders::_1) );
  }
}

template <class T>
BaseAssembler<T>::~BaseAssembler()
{
  if (tf_filter_)
    delete tf_filter_;

  delete tf_ ;
}

template <class T>
void BaseAssembler<T>::msgCallback(const boost::shared_ptr<const T>& scan_ptr)
{
  ROS_DEBUG("starting msgCallback");
  const T scan = *scan_ptr ;

  sensor_msgs::PointCloud cur_cloud ;

  // Convert the scan data into a universally known datatype: PointCloud
  try
  {
    ConvertToCloud(fixed_frame_, scan, cur_cloud) ;              // Convert scan into a point cloud
  }
  catch(tf::TransformException& ex)
  {
    ROS_WARN("Transform Exception %s", ex.what()) ;
    return ;
  }

  // Add the current scan (now of type PointCloud) into our history of scans
  scan_hist_mutex_.lock() ;
  if (scan_hist_.size() == max_scans_)                           // Is our deque full?
  {
    total_pts_ -= scan_hist_.front().points.size () ;            // We're removing an elem, so this reduces our total point count
    scan_hist_.pop_front() ;                                     // The front of the deque has the oldest elem, so we can get rid of it
  }
  scan_hist_.push_back(cur_cloud) ;                              // Add the newest scan to the back of the deque
  total_pts_ += cur_cloud.points.size () ;                       // Add the new scan to the running total of points

  //printf("Scans: %4u  Points: %10u\n", scan_hist_.size(), total_pts_) ;

  scan_hist_mutex_.unlock() ;
  ROS_DEBUG("done with msgCallback");
}

template <class T>
bool BaseAssembler<T>::buildCloud(AssembleScans::Request& req, AssembleScans::Response& resp)
{
  ROS_WARN("Service 'build_cloud' is deprecated. Call 'assemble_scans' instead");
  return assembleScans(req, resp);
}

template <class T>
bool BaseAssembler<T>::assembleScans(AssembleScans::Request& req, AssembleScans::Response& resp)
{

  scan_hist_mutex_.lock() ;
  // Determine where in our history we actually are
  unsigned int i = 0 ;

  // Find the beginning of the request. Probably should be a search
  while ( i < scan_hist_.size() &&                                                    // Don't go past end of deque
          scan_hist_[i].header.stamp < req.begin )                                    // Keep stepping until we've exceeded the start time
  {
    i++ ;
  }
  unsigned int start_index = i ;

  unsigned int req_pts = 0 ;                                                          // Keep a total of the points in the current request
  // Find the end of the request
  while ( i < scan_hist_.size() &&                                                    // Don't go past end of deque
          scan_hist_[i].header.stamp < req.end )                                      // Don't go past the end-time of the request
  {
    req_pts += (scan_hist_[i].points.size ()+downsample_factor_-1)/downsample_factor_ ;
    i += downsample_factor_ ;
  }
  unsigned int past_end_index = i ;

  if (start_index == past_end_index)
  {
    resp.cloud.header.frame_id = fixed_frame_ ;
    resp.cloud.header.stamp = req.end ;
    resp.cloud.points.resize (0) ;
    resp.cloud.channels.resize (0) ;
  }
  else
  {
    // Note: We are assuming that channel information is consistent across multiple scans. If not, then bad things (segfaulting) will happen
    // Allocate space for the cloud
    resp.cloud.points.resize (req_pts);
    const unsigned int num_channels = scan_hist_[start_index].channels.size ();
    resp.cloud.channels.resize (num_channels) ;
    for (i = 0; i<num_channels; i++)
    {
      resp.cloud.channels[i].name = scan_hist_[start_index].channels[i].name ;
      resp.cloud.channels[i].values.resize (req_pts) ;
    }
    //resp.cloud.header.stamp = req.end ;
    resp.cloud.header.frame_id = fixed_frame_ ;
    unsigned int cloud_count = 0 ;
    for (i=start_index; i<past_end_index; i+=downsample_factor_)
    {

      // Sanity check: Each channel should be the same length as the points vector
      for (unsigned int chan_ind = 0; chan_ind < scan_hist_[i].channels.size(); chan_ind++)
      {
        if (scan_hist_[i].points.size () != scan_hist_[i].channels[chan_ind].values.size())
          ROS_FATAL("Trying to add a malformed point cloud. Cloud has %u points, but channel %u has %u elems", (int)scan_hist_[i].points.size (), chan_ind, (int)scan_hist_[i].channels[chan_ind].values.size ());
      }

      for(unsigned int j=0; j<scan_hist_[i].points.size (); j+=downsample_factor_)
      {
        resp.cloud.points[cloud_count].x = scan_hist_[i].points[j].x ;
        resp.cloud.points[cloud_count].y = scan_hist_[i].points[j].y ;
        resp.cloud.points[cloud_count].z = scan_hist_[i].points[j].z ;

        for (unsigned int k=0; k<num_channels; k++)
          resp.cloud.channels[k].values[cloud_count] = scan_hist_[i].channels[k].values[j] ;

        cloud_count++ ;
      }
      resp.cloud.header.stamp = scan_hist_[i].header.stamp;
    }
  }
  scan_hist_mutex_.unlock() ;

  ROS_ERROR_STREAM("Start and end indics are: " << start_index <<  "  " << past_end_index );
  resp.start_index.data = start_index;
  resp.end_index.data =  past_end_index;

  ROS_DEBUG("Point Cloud Results: Aggregated from index %u->%u. BufferSize: %lu. Points in cloud: %u", start_index, past_end_index, scan_hist_.size(), (int)resp.cloud.points.size ()) ;
  return true ;
}

template <class T>
bool BaseAssembler<T>::buildCloud2(AssembleScans2::Request& req, AssembleScans2::Response& resp)
{
  ROS_WARN("Service 'build_cloud' is deprecated. Call 'assemble_scans' instead");
  return assembleScans2(req, resp);
}


template <class T>
bool BaseAssembler<T>::assembleScans2(AssembleScans2::Request& req, AssembleScans2::Response& resp)
{
  AssembleScans::Request tmp_req;
  AssembleScans::Response tmp_res;
  tmp_req.begin = req.begin;
  tmp_req.end = req.end;
  bool ret = assembleScans(tmp_req, tmp_res);

  resp.start_index = tmp_res.start_index;
  resp.end_index =  tmp_res.end_index;

  if ( ret )
  {
    sensor_msgs::convertPointCloudToPointCloud2(tmp_res.cloud, resp.cloud);
  }

  return ret;
}

template<class T>
void BaseAssembler<T>::convertToMesh()
{
  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointcloud2_merged_const_ptr(&pointcloud2_merged_);
  tree->setInputCloud (pointcloud2_merged_const_ptr);
  n.setSearchMethod (tree);
  n.setKSearch (5);
  n.compute (*normals);

  //* normals should not contain the point normals + surface curvatures
  // // Concatenate the XYZRGB and normal fields*
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::concatenateFields (pointcloud2_merged_, *normals, *cloud_with_normals);


  // // Create search tree*
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
  pcl::PolygonMesh triangles;

  // // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.025);

  // // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  pcl::io::saveVTKFile ("/tmp/mesh.vtk", triangles);

}

template <class T>
bool BaseAssembler<T>::getOverlapTransformation(void)
{
  //searching for overlapped points in the point cloud
  // Allocate enough space to hold the results
  int max_nn_overlap_ = 10;
  double radius_overlap_ = 0.05;
  std::vector<int> nn_indices (max_nn_overlap_);
  std::vector<float> nn_dists (max_nn_overlap_);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  size_t count_idx = 0;

  for ( auto it = pointcloud2_current_.begin() ; it != pointcloud2_current_.end() ; ++it)
  {
    kdtree_.radiusSearch(pointcloud2_current_, count_idx, radius_overlap_, nn_indices, nn_dists, max_nn_overlap_);
    if(nn_indices.size() > 0 )
    {
      // inliers->indices.push_back(idx);
      pointcloud2_current_.erase(it);
    }
    count_idx += 1;
  }

  // for(size_t idx = 0 ; idx < pointcloud2_current_.points.size(); idx++ )
  // {
  //   kdtree_.radiusSearch(pointcloud2_current_, idx, radius_overlap_, nn_indices, nn_dists, max_nn_overlap_);
  //   if(nn_indices.size() > 0 )
  //   {
  //     // inliers->indices.push_back(idx);
  //     pointcloud2_current_.erase(i);
  //   }
  // }

  // for(size_t idx = 0 ; idx < pointcloud2_current_.points.size(); idx++ )
  // {
  //   kdtree_.radiusSearch(pointcloud2_current_, idx, radius_overlap_, nn_indices, nn_dists, max_nn_overlap_);
  //   if(nn_indices.size() > 0 )
  //   {
  //     inliers->indices.push_back(idx);
  //   }
  // }
  
  // ROS_ERROR_STREAM("Before -> Current Points: " << pointcloud2_current_.points.size() << " To Removed: " << inliers->indices.size() ) ;
  // pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr filteredptr( &pointcloud2_current_ );
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputPtr (new pcl::PointCloud<pcl::PointXYZRGB>);

  // extract.setInputCloud(filteredptr);
  // extract.setIndices(inliers);
  // extract.setNegative(true);
  // extract.filter(*outputPtr);

  // ROS_ERROR_STREAM("After -> Current Points: " << pointcloud2_current_.points.size() << " Removed: " << outputPtr->points.size()) ;
  
  return true;
}

template <class T>
pcl::PointCloud<pcl::PointXYZRGB> BaseAssembler<T>::convertFromMsgToPointCloud(const sensor_msgs::PointCloud& pointcloud_msg)
{
  // Declaring some variables required in this function
  sensor_msgs::PointCloud2 pointcloud2_msg;
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl_step01;
  pcl::PointCloud<pcl::PointXYZRGB> pointcloud_pcl_step02;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree_ptr_(new pcl::search::KdTree<pcl::PointXYZI>);
  std::vector<int> indices_nan;

  // Converting from PointCloud msg format to PointCloud2 msg format
  sensor_msgs::convertPointCloudToPointCloud2(pointcloud_msg, pointcloud2_msg);

  //This is done because there is a bug in PCL.
  for(u_int i = 0 ; i < pointcloud2_msg.fields.size(); i++)
  {
    if(pointcloud2_msg.fields[i].name == "intensities")
    {
      pointcloud2_msg.fields[i].name = "intensity";
    }
  } 

  // Converting from PointCloud2 msg format to pcl pointcloud format
  pcl::fromROSMsg(pointcloud2_msg, pointcloud_pcl_step01);
  // remove nan
  pcl::removeNaNFromPointCloud( pointcloud_pcl_step01, pointcloud_pcl_step02, indices_nan );
  pointcloud_pcl_step02.is_dense = false;

  return (pointcloud_pcl_step02);
}

template <class T>
bool BaseAssembler<T>::mergeScanICP(AssembleScans2::Request& req, AssembleScans2::Response& resp)
{

  AssembleScans::Request tmp_req;
  AssembleScans::Response tmp_res;
  tmp_req.begin = req.begin;
  tmp_req.end = req.end;
  bool ret = assembleScans(tmp_req, tmp_res);

  if ( ret && tmp_res.cloud.points.size() > 0)
  {
    pointcloud2_merged_ = convertFromMsgToPointCloud(tmp_res.cloud); 
    pcl::toROSMsg(pointcloud2_merged_, resp.cloud );
  }
  else 
  {
    resp.cloud.fields.resize(0);
    resp.cloud.data.resize(0);
  }

  resp.cloud.header.frame_id = fixed_frame_.c_str();
  resp.cloud.header.stamp = ros::Time::now();
  
  return true ;
}

template <class T>
bool BaseAssembler<T>::mergeScans2(AssembleScans2::Request& req, AssembleScans2::Response& resp)
{
  bool ret = mergeScanICP(req, resp);
  return ret;
}

}
