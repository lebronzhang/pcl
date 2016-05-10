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

#ifndef PCL_IO_LIBREALSENSE_GRABBER_H_
#define PCL_IO_LIBREALSENSE_GRABBER_H_

#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl
{

  // Forward declaration of a class that contains actual grabber implementation
  namespace io { namespace librealsense { struct LibRealSenseGrabberImpl; } }

  /** Grabber for RealSense devices (e.g. Intel® RealSense™ F200, SR300 and R200 cameras).
    *
    * Requires [Intel librealsense](https://github.com/IntelRealSense/librealsense).
    *
    * \author Lebron Zhang
    * \ingroup io */
  class PCL_EXPORTS LibRealSenseGrabber : public Grabber
  {

    public:

      typedef
        void (sig_cb_librealsense_point_cloud)
          (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&);

      typedef
        void (sig_cb_librealsense_point_cloud_rgba)
          (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&);

      /** Create a grabber for a RealSense device.
        *
        * The grabber "captures" the device, making it impossible for other
        * grabbers to interact with it. The device is "released" when the
        * grabber is destructed.
        *
        * This will throw pcl::IOException if there are no free devices that
        * match the supplied \a device_id.
        *
        * \param[in] device_id device identifier, which might be a serial
        * number, an index (with '#' prefix), or an empty string (to select the
        * first available device)
        */
      LibRealSenseGrabber (const std::string& device_id = "");

      virtual
      ~LibRealSenseGrabber () throw ();

      virtual void
      start ();

      virtual void
      stop ();

      virtual bool
      isRunning () const;

      virtual std::string
      getName () const
      {
        return (std::string ("LibRealSenseGrabber"));
      }

      virtual float
      getFramesPerSecond () const;

      /** Get the serial number of device captured by the grabber. */
      std::string
      getDeviceSerialNumber () const;

    private:

      pcl::io::librealsense::LibRealSenseGrabberImpl* p_;
      friend struct pcl::io::librealsense::LibRealSenseGrabberImpl;

  };

}

#endif  // PCL_IO_LIBREALSENSE_GRABBER_H_
