#ifndef _OPENPOSE_ROS_IO
#define _OPENPOSE_ROS_IO

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
#include <openpose_ros/FaceFeatures.h>
#include <openpose_ros/BodyFeatures.h>
#include <openpose_ros/BodyPart.h>

#include <openpose.h>
#include <gflags_options.h>

// OpenPose dependencies
#include <openpose/headers.hpp>

namespace openpose_ros {

    class OpenPoseROSIO
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
            OpenPoseROSIO(OpenPose &openPose);

            ~OpenPoseROSIO(){}

            void processImage(const sensor_msgs::ImageConstPtr& msg);

            void convertImage(const sensor_msgs::ImageConstPtr& msg);

            std::shared_ptr<std::vector<op::Datum>> createDatum();

            bool display(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr,
                         const openpose_ros::OpenPoseHumanList);

            cv_bridge::CvImagePtr& getCvImagePtr();

            openpose_ros::OpenPoseHumanList getKeypoints(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr);

            void publishKeypoints(const openpose_ros::OpenPoseHumanList);

            template <typename T> void printKeypoints(T poseKeypoints, T faceKeypoints,
                                              T leftHandKeypoints, T rightHandKeypoints);

            template <typename T> void printHeatmaps(T poseHeatMaps, T faceHeatMaps,
                                              T leftHandHeatMaps, T rightHandHeatMaps);

    };
}

#endif
