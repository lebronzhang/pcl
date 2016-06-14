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

#include <pcl/io/io_exception.h>
#include <chrono>
#include <unistd.h>

#include <pcl/io/librealsense/librealsense_grabber_impl.h>
#include <pcl/io/librealsense/librealsense_device_manager.h>

boost::mutex pcl::io::librealsense::LibRealSenseDeviceManager::mutex_;

pcl::io::librealsense::LibRealSenseDeviceManager::LibRealSenseDeviceManager ()
{
}

pcl::io::librealsense::LibRealSenseDeviceManager::~LibRealSenseDeviceManager ()
{
}


std::string
pcl::io::librealsense::LibRealSenseDeviceManager::captureDevice (LibRealSenseGrabberImpl* grabber)
{
  boost::mutex::scoped_lock lock (mutex_);
  if (context_.get_device_count () == 0)
    THROW_IO_EXCEPTION ("no connected devices");
  for(int i = 0; i < context_.get_device_count (); i++)
  {
    if (!isCaptured (context_.get_device (i)->get_serial ()))
      return (captureDevice (grabber, context_.get_device (i)));
  }
  THROW_IO_EXCEPTION ("all connected devices are captured by other grabbers");
  return ("");  // never reached, needed just to silence -Wreturn-type warning
}

std::string
pcl::io::librealsense::LibRealSenseDeviceManager::captureDevice (LibRealSenseGrabberImpl* grabber, size_t index)
{
  boost::mutex::scoped_lock lock (mutex_);
  if (index >= context_.get_device_count ())
    THROW_IO_EXCEPTION ("device with index %i is not connected", index + 1);
  if (isCaptured (context_.get_device (index)->get_serial ()))
    THROW_IO_EXCEPTION ("device with index %i is captured by another grabber", index);
  return (captureDevice (grabber, context_.get_device (index)));
}

std::string
pcl::io::librealsense::LibRealSenseDeviceManager::captureDevice (LibRealSenseGrabberImpl* grabber, const std::string& sn)
{
  boost::mutex::scoped_lock lock (mutex_);
  for (size_t i = 0; i < context_.get_device_count (); ++i)
  {
    if (context_.get_device (i)->get_serial () == sn || context_.get_device (i)->get_serial () == sn + " ")
    {
      if (isCaptured (sn))
        THROW_IO_EXCEPTION ("device with serial number %s is captured by another grabber", sn.c_str ());
      return (captureDevice (grabber, context_.get_device (i)));
    }
  }
  THROW_IO_EXCEPTION ("device with serial number %s is not connected", sn.c_str ());
  return ("");  // never reached, needed just to silence -Wreturn-type warning
}

void
pcl::io::librealsense::LibRealSenseDeviceManager::startDevice (const std::string& sn)
{
  CapturedDevice& dev = captured_devices_[sn];
  boost::thread device_thread = boost::thread (boost::bind (&pcl::io::librealsense::LibRealSenseDeviceManager::run, this, dev, sn));
  thread_close_flag_[sn] = false;
}

void
pcl::io::librealsense::LibRealSenseDeviceManager::stopDevice (const std::string& sn)
{
  CapturedDevice& dev = captured_devices_[sn];
  thread_close_flag_[sn] = true;
  //hang up the thread to make sure the closing thread not been blocked by waitting for frames
  usleep (40000);
  dev.librealsense_device->stop ();
}

void
pcl::io::librealsense::LibRealSenseDeviceManager::releaseDevice (const std::string& sn)
{
  boost::mutex::scoped_lock lock (mutex_);
  captured_devices_.erase (sn);
  thread_close_flag_.erase (sn);
}

std::string
pcl::io::librealsense::LibRealSenseDeviceManager::captureDevice (LibRealSenseGrabberImpl* grabber, rs::device* device)
{
  // This is called from public captureDevice() functions and should already be
  // under scoped lock
  CapturedDevice dev;
  dev.grabber = grabber;
  dev.librealsense_device = device;
  captured_devices_.insert (std::make_pair (device->get_serial (), dev));
  thread_close_flag_.insert (std::make_pair (device->get_serial (), true));
  return (device->get_serial ());
}


void
pcl::io::librealsense::LibRealSenseDeviceManager::run (const CapturedDevice& dev, const std::string& sn)
{
  int frames = 0;
  float time = 0, fps = 0;
  auto t0 = std::chrono::high_resolution_clock::now ();
  dev.librealsense_device->enable_stream (rs::stream::depth, rs::preset::best_quality);
  dev.librealsense_device->enable_stream (rs::stream::color, rs::preset::best_quality);
  dev.librealsense_device->start ();
  while (!thread_close_flag_[sn])
  {
    if (dev.librealsense_device->is_streaming ()) dev.librealsense_device->wait_for_frames ();
    auto t1 = std::chrono::high_resolution_clock::now ();
    time += std::chrono::duration<float> (t1 - t0).count ();
    t0 = t1;
    ++frames;
    if (time > 0.5f)
    {
      fps = frames / time;
      frames = 0;
      time = 0;
    }
    // Retrieve our images
    const uint16_t * depth_image = (const uint16_t *)dev.librealsense_device->get_frame_data (rs::stream::depth);
    const uint8_t * color_image = (const uint8_t *)dev.librealsense_device->get_frame_data (rs::stream::color);
    // Retrieve camera parameters for mapping between depth and color
    rs::intrinsics depth_intrin = dev.librealsense_device->get_stream_intrinsics (rs::stream::depth);
    rs::intrinsics color_intrin = dev.librealsense_device->get_stream_intrinsics (rs::stream::color);
    rs::extrinsics depth_to_color = dev.librealsense_device->get_extrinsics (rs::stream::depth, rs::stream::color);
    float scale = dev.librealsense_device->get_depth_scale ();
    dev.grabber->onDataReceived (depth_image, color_image, depth_intrin, color_intrin, depth_to_color, scale, fps);
  }
}
