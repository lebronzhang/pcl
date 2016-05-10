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

#ifndef PCL_IO_LIBREALSENSE_LIBREALSENSE_GRABBER_IMPL_H_
#define PCL_IO_LIBREALSENSE_LIBREALSENSE_GRABBER_IMPL_H

#include <librealsense/rs.hpp>

#include <pcl/common/time.h>
#include <pcl/io/buffers.h>
#include <pcl/io/librealsense_grabber.h>

namespace pcl
{

  namespace io
  {

    namespace librealsense
    {

      struct LibRealSenseGrabberImpl
      {

        /// Parent grabber
        LibRealSenseGrabber* p_;

        /// Serial number of the device captured by this grabber
        std::string device_id_;

        bool is_running_;

        typedef LibRealSenseGrabber::sig_cb_librealsense_point_cloud sig_cb_librealsense_point_cloud;
        typedef LibRealSenseGrabber::sig_cb_librealsense_point_cloud_rgba sig_cb_librealsense_point_cloud_rgba;

        /// Signal to indicate whether new XYZ cloud is available
        boost::signals2::signal<sig_cb_librealsense_point_cloud>* point_cloud_signal_;
        /// Signal to indicate whether new XYZRGBA cloud is available
        boost::signals2::signal<sig_cb_librealsense_point_cloud_rgba>* point_cloud_rgba_signal_;

        /// Indicates whether there are subscribers for PointXYZ signal. This is
        /// computed and stored on start()
        bool need_xyz_;

        /// Indicates whether there are subscribers for PointXYZRGBA signal. This
        /// is computed and stored on start()
        bool need_xyzrgba_;

        ///frequency;
        int fps_;

        ///Buffer to store depth data
        std::vector<uint16_t> depth_data_;

        ///Buffer to store color data
        std::vector<uint8_t> color_data_;

        int depth_width_;
        int depth_height_;
        int depth_size_;
        int color_width_;
        int color_height_;
        int color_size_;

        LibRealSenseGrabberImpl (LibRealSenseGrabber* parent, const std::string& device_id);

        ~LibRealSenseGrabberImpl () throw ();

        void
        start ();

        void
        stop ();

        float
        getFramesPerSecond () const;

        /** A callback for processing data.
          *
          * It is supposed to be called from the pcl::io::librealsense::LibRealSenseDeviceManager::run thread that
          * is managed by LibRealSenseDeviceManager. */
        void
        onDataReceived (const uint16_t* depth_image, const uint8_t* color_image, rs::intrinsics depth_intrin, rs::intrinsics color_intrin, rs::extrinsics depth_to_color, float scale,int fps);

      };

    }

  }

}

#endif  // PCL_IO_LIBREALSENSE_LIBREALSENSE_GRABBER_IMPL_H_
