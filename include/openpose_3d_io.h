#ifndef _OPENPOSE_3D_IO
#define _OPENPOSE_3D_IO

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>

#include <opencv2/core/core.hpp>

#include <openpose_ros/BoundingBox.h>
#include <openpose_ros/OpenPoseHuman.h>
#include <openpose_ros/OpenPoseHumanList.h>
#include <openpose_ros/PointWithProb.h>
#include <openpose_ros/HandKeypoints.h>
#include <openpose_ros/FaceFeatures3D.h>
#include <openpose_ros/BodyFeatures3D.h>
#include <openpose_ros/BodyPart3D.h>

#include <openpose.h>
#include <gflags_options.h>

// OpenPose dependencies
#include <openpose/headers.hpp>

namespace openpose_ros {

    class OpenPose3DIO
    {
        private:
            ros::NodeHandle nh_;
            ros::Publisher openpose_human_list_pub_;
            image_transport::ImageTransport it_;
            image_transport::Subscriber image_sub_;
            cv_bridge::CvImagePtr cv_img_ptr_;
            std_msgs::Header rgb_image_header_;

            OpenPose* openpose_;

        public:
            OpenPose3DIO(OpenPose &openPose);

            ~OpenPose3DIO(){}

            void processPointCloud();

    };
}

#endif
