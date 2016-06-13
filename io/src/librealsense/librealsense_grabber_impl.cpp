/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *  Copyright (c) 2016, Intel Corporation
 *
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 *
 */

#include <boost/lexical_cast.hpp>

#include <pcl/common/io.h>
#include <pcl/io/librealsense_grabber.h>
#include <pcl/io/librealsense/librealsense_grabber_impl.h>
#include <pcl/io/librealsense/librealsense_device_manager.h>

#include <librealsense/rs.hpp>


pcl::io::librealsense::LibRealSenseGrabberImpl::LibRealSenseGrabberImpl (LibRealSenseGrabber* parent, const std::string& device_id)
: p_ (parent)
, is_running_ (false)
, fps_ (0)
{
  if (device_id == "")
  {
    device_id_ = LibRealSenseDeviceManager::getInstance ()->captureDevice (this);
  }
  else if (device_id[0] == '#')
    device_id_ = LibRealSenseDeviceManager::getInstance ()->captureDevice (this, boost::lexical_cast<int> (device_id.substr (1)) - 1);
  else
    device_id_ = LibRealSenseDeviceManager::getInstance ()->captureDevice (this, device_id);

  point_cloud_signal_ = p_->createSignal<sig_cb_librealsense_point_cloud> ();
  point_cloud_rgba_signal_ = p_->createSignal<sig_cb_librealsense_point_cloud_rgba> ();
}

pcl::io::librealsense::LibRealSenseGrabberImpl::~LibRealSenseGrabberImpl () throw ()
{
  if (is_running_) stop ();
  LibRealSenseDeviceManager::getInstance ()->releaseDevice (device_id_);

  p_->disconnect_all_slots<sig_cb_librealsense_point_cloud> ();
  p_->disconnect_all_slots<sig_cb_librealsense_point_cloud_rgba> ();
}

void
pcl::io::librealsense::LibRealSenseGrabberImpl::start ()
{
  need_xyz_ = p_->num_slots<sig_cb_librealsense_point_cloud> () > 0;
  need_xyzrgba_ = p_->num_slots<sig_cb_librealsense_point_cloud_rgba> () > 0;

  if (!is_running_)
  {
    LibRealSenseDeviceManager::getInstance ()->startDevice (device_id_);
    is_running_ = true;
  }
}

void
pcl::io::librealsense::LibRealSenseGrabberImpl::stop ()
{
  if (is_running_)
  {
    LibRealSenseDeviceManager::getInstance ()->stopDevice (device_id_);
    is_running_ = false;
    fps_ = 0;
  }
}

float
pcl::io::librealsense::LibRealSenseGrabberImpl::getFramesPerSecond () const
{
  return fps_;
}

void
pcl::io::librealsense::LibRealSenseGrabberImpl::onDataReceived (const uint16_t* depth_image, const uint8_t* color_image, rs::intrinsics depth_intrin, rs::intrinsics color_intrin, rs::extrinsics depth_to_color, float scale, int fps)
{
  if (fps_ < 1)
  {
    depth_width_ = depth_intrin.width;
    depth_height_ = depth_intrin.height;
    depth_size_ = depth_width_ * depth_height_;
    color_width_ = color_intrin.width;
    color_height_ = color_intrin.height;
    color_size_ = color_width_ * color_height_ * 3;
    depth_data_.resize (depth_size_);
    color_data_.resize (color_size_);
  }

  depth_data_.clear ();
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr xyzrgba_cloud;
  fps_ = fps;
  float max_distance = 6; // get rid of noisy data that is past 6 meters
  static const float nan = std::numeric_limits<float>::quiet_NaN ();

  memcpy (depth_data_.data (), &depth_image[0], depth_size_ * sizeof (uint16_t));

  if (need_xyzrgba_)
  {
    color_data_.clear ();
    memcpy (color_data_.data (), &color_image[0], color_size_ * sizeof (uint8_t));
    xyzrgba_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA> (depth_width_, depth_height_));
    xyzrgba_cloud->is_dense = false;
    if (need_xyz_)
    {
      xyz_cloud.reset (new pcl::PointCloud<pcl::PointXYZ> (depth_width_, depth_height_));
      xyz_cloud->is_dense = false;
    }
    for (int dy = 0; dy < depth_height_; ++dy)
    {
      uint i = dy * depth_width_ - 1;
      for (int dx = 0; dx < depth_width_; ++dx)
      {
        i++;
        // Retrieve the 16-bit depth value and map it into a depth in meters
        uint16_t depth_value = depth_data_[i];
        float depth_in_meters = depth_value * scale;

        // Map from pixel coordinates in the depth image to real world co-ordinates
        rs::float2 depth_pixel = {(float)dx, (float)dy};
        rs::float3 depth_point = depth_intrin.deproject (depth_pixel, depth_in_meters);
        rs::float3 color_point = depth_to_color.transform (depth_point);
        rs::float2 color_pixel = color_intrin.project (color_point);

        const int cx = (int)std::round (color_pixel.x), cy = (int)std::round (color_pixel.y);
        int red = 0, green = 0, blue = 0;
        if (cx < 0 || cy < 0 || cx >= color_width_ || cy >= color_height_)
        {
          red = 255; green = 255; blue = 255;
        }
        else
        {
          int pos = (cy * color_width_ + cx) * 3;
          red =  color_data_[pos];
          green = color_data_[pos + 1];
          blue = color_data_[pos + 2];
        }
        if (depth_value == 0 || depth_point.z > max_distance)
        {
          xyzrgba_cloud->points[i].x = xyzrgba_cloud->points[i].y = xyzrgba_cloud->points[i].z = (float) nan;
          if (need_xyz_)
          {
            xyz_cloud->points[i].x = xyz_cloud->points[i].y = xyz_cloud->points[i].z = (float) nan;
          }
          continue;
        }
        else
        {
          xyzrgba_cloud->points[i].x = depth_point.x;
          xyzrgba_cloud->points[i].y = -depth_point.y;
          xyzrgba_cloud->points[i].z = -depth_point.z;
          xyzrgba_cloud->points[i].r = red;
          xyzrgba_cloud->points[i].g = green;
          xyzrgba_cloud->points[i].b = blue;
          if (need_xyz_)
          {
            xyz_cloud->points[i].x = depth_point.x;
            xyz_cloud->points[i].y = -depth_point.y;
            xyz_cloud->points[i].z = -depth_point.z;
          }
        }
      }
    }
    point_cloud_rgba_signal_->operator () (xyzrgba_cloud);
    if (need_xyz_)
    {
      point_cloud_signal_->operator () (xyz_cloud);
    }
  }
  else if (need_xyz_)
  {
    xyz_cloud.reset (new pcl::PointCloud<pcl::PointXYZ> (depth_width_, depth_height_));
    xyz_cloud->is_dense = false;
    for (int dy = 0; dy < depth_height_; ++dy)
    {
      uint i = dy * depth_width_ - 1;
      for (int dx = 0; dx < depth_width_; ++dx)
      {
        i++;
        // Retrieve the 16-bit depth value and map it into a depth in meters
        uint16_t depth_value = depth_data_[i];
        float depth_in_meters = depth_value * scale;
        rs::float2 depth_pixel = {(float)dx, (float)dy};
        rs::float3 depth_point = depth_intrin.deproject (depth_pixel, depth_in_meters);
        if (depth_value == 0 || depth_point.z > max_distance)
        {
          xyz_cloud->points[i].x = xyz_cloud->points[i].y = xyz_cloud->points[i].z = (float) nan;
          continue;
        }
        else
        {
          xyz_cloud->points[i].x = depth_point.x;
          xyz_cloud->points[i].y = -depth_point.y;
          xyz_cloud->points[i].z = -depth_point.z;
        }
      }
    }
    point_cloud_signal_->operator () (xyz_cloud);
  }
  else
  {
    //do nothing
  }
}
