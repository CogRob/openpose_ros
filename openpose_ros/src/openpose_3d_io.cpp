#include <openpose_3d_io.h>

using namespace openpose_ros;

/*
 * Subscribes to the input video feed and starts processing.
 */
OpenPose3DIO::OpenPose3DIO(OpenPose &openPose): it_(nh_)
{
    // Subscribe to input point cloud; publish human lists as output
    std::string pc_topic;
    std::string output_topic;

    nh_.param("/openpose_ros_3d_node/pc_topic", pc_topic, std::string("/PointCloud2"));
    nh_.param("/openpose_ros_3d_node/output_topic", output_topic, std::string("/openpose_ros/human_list"));

    // Subscribe to point clouds from depth sensor
    //pc_sub_ = it_.subscribe(pc_topic, 1, &OpenPose3DIO::processPointCloud, this);

    //openpose_human_list_pub_ = nh_.advertise<openpose_ros_msgs::OpenPoseHumanList>(output_topic, 10);

    cv_img_ptr_ = nullptr;
    openpose_ = &openPose;
}


/*
 * Callback function for point cloud topic subscriber.
 */
void OpenPose3DIO::processPointCloud()
{

}
