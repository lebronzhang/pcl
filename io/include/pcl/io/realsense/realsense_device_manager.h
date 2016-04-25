/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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


#ifndef PCL_IO_REALSENSE_DEVICE_MANAGER_H
#define PCL_IO_REALSENSE_DEVICE_MANAGER_H

#include <boost/utility.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <pcl/pcl_exports.h>

#include <librealsense/rs.hpp>

namespace pcl
{
  namespace io
  {
 	namespace realsense
 	{
 	  struct RealSenseGrabberImpl;

 	  class PCL_EXPORTS RealSenseDeviceManager : boost::noncopyable
 	  {
 	    public:

 		  typedef boost::shared_ptr<RealSenseDeviceManager> Ptr;

 		  static Ptr&
 		  getInstance ()
 		  {
 		    static Ptr instance;
 		    if (!instance)	
 		      {
 		        boost::mutex::scoped_lock lock (mutex_);
 			    if (!instance)
 			      {
 			        instance.reset (new RealSenseDeviceManager);
 			      }
 		      }
 		    return (instance);
 		  }

 		  /** Get the number of connected RealSense devices. */
 		  inline size_t
 		  getNumDevices ()
 		  {
 		    return (context_.get_device_count ());
 		  }

		  /** Capture first available device and associate it with a given
		  * grabber instance. */
		  std::string
		  captureDevice (RealSenseGrabberImpl* grabber);

		  /** Capture the device with given index and associate it with a given
		  * grabber instance. */
		  std::string
		  captureDevice (RealSenseGrabberImpl* grabber, size_t index);

		  /** Capture the device with given serial number and associate it with
		  * a given grabber instance. */
		  std::string
		  captureDevice (RealSenseGrabberImpl* grabber, const std::string& sn);

		  /** Release RealSense device with given serial number. */
		  void
		  releaseDevice (const std::string& sn);

		  /** Start data capturing for a given device. */
		  void
		  startDevice (const std::string& sn);

		  /** Stop data capturing for a given device. */
		  void
		  stopDevice (const std::string& sn);

		  ~RealSenseDeviceManager ();

		private:

		  RealSenseDeviceManager ();

		  std::string
		  captureDevice (RealSenseGrabberImpl* grabber, rs::device* device);

		  inline bool
		  isCaptured (const std::string& sn) const
		  {
		    return (captured_devices_.count (sn) != 0);
		  }

		  rs::context context_;

		  static boost::mutex mutex_;

		  struct CapturedDevice
		  {
		    RealSenseGrabberImpl* grabber;
		    rs::device* realsense_device;
		  };

		  void
		  run (const CapturedDevice& dev, const std::string& sn);

		  std::map<std::string, CapturedDevice> captured_devices_;

		  //Thread map which control thread by setting thread flag
		  std::map<std::string, bool> thread_close_flag_;

	  };
	} // namespace real_sense
  } // namespace io
} //namespace pcl

#endif /* PCL_IO_REALSENSE_DEVICE_MANAGER_H */