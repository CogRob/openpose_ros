#include <openpose_ros_io.h>

using namespace openpose_ros;

/*
 * Subscribes to the input video feed, publishes human lists from OpenPose.
 */
OpenPoseROSIO::OpenPoseROSIO(OpenPose &openPose): it_(nh_)
{
    // Subscribe to input video feed and publish human lists as output
    std::string image_topic;
    std::string output_topic;

    nh_.param("/openpose_ros_node/image_topic", image_topic, std::string("/camera/image_raw"));
    nh_.param("/openpose_ros_node/output_topic", output_topic, std::string("/openpose_ros/human_list"));

    image_sub_ = it_.subscribe(image_topic, 1, &OpenPoseROSIO::processImage, this);
    openpose_human_list_pub_ = nh_.advertise<openpose_ros_msgs::OpenPoseHumanList>(output_topic, 10);
    cv_img_ptr_ = nullptr;
    openpose_ = &openPose;
}


/*
 * Process images by converting to OpenCV format and sending to OpenPose
 * to process.
 * After OpenPose has processed image, display updated frame in window, print
 * keypoints to terminal, and publish keypoints to ROS topic.
 */
void OpenPoseROSIO::processImage(const sensor_msgs::ImageConstPtr& msg)
{
    convertImage(msg);
    std::shared_ptr<std::vector<op::Datum>> datumToProcess = createDatum();

    bool successfullyEmplaced = openpose_->waitAndEmplace(datumToProcess);

    // Pop frame
    std::shared_ptr<std::vector<op::Datum>> datumProcessed;

    // If OpenPose successfully processes frame
    if (successfullyEmplaced && openpose_->waitAndPop(datumProcessed))
    {

        // Publish keypoints from OpenPose to ROS topic and log to console
        publishKeypoints(datumProcessed);

        // Display frame in window
        display(datumProcessed);

    }
    else
    {
        op::log("Processed datum could not be emplaced.", op::Priority::High,
                __LINE__, __FUNCTION__, __FILE__);
    }
}


/*
 * Converts image from sensor_msgs BGR8 format to OpenCV format.
 */
void OpenPoseROSIO::convertImage(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_img_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        rgb_image_header_ = msg->header;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}


/*
 * Creates vector of datums for processing.
 */
std::shared_ptr<std::vector<op::Datum>> OpenPoseROSIO::createDatum()
{
    // Close program when empty frame
    if (cv_img_ptr_ == nullptr)
    {
        return nullptr;
    }
    else // if (cv_img_ptr_ == nullptr)
    {
        // Create new datum
        auto datumsPtr = std::make_shared<std::vector<op::Datum>>();
        datumsPtr->emplace_back();
        auto& datum = datumsPtr->at(0);

        // Fill datum
        datum.cvInputData = cv_img_ptr_->image;

        return datumsPtr;
    }
}

/*
 * Displays image from OpenPose with rendered pose or heatmap.
 * Also displays bounding boxes for face and hands.
 * TODO: add user-defined flag for bounding box display
 */
bool OpenPoseROSIO::display(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr)
{

    bool faceBoundingBoxShow = 1;
    bool handBoundingBoxShow = 0;

    // User's displaying/saving/other processing here
    // datum.cvOutputData: rendered frame with pose or heatmaps
    // datum.poseKeypoints: Array<float> with the estimated pose
    char key = ' ';
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {

        // Get rendered image from OpenPose
        cv::Mat outputImg = datumsPtr->at(0).cvOutputData;

        // If we're drawing a bounding box for the face, get points and draw
        if (faceBoundingBoxShow)
        {

        }


        // If we're drawing a bounding box for hands, get points and draw

        // try to draw a circle
        cv::Point pt = cv::Point(250,250);
        int thickness = 1;
        int lineType = 8;
        cv::circle(outputImg, pt, 50, cv::Scalar( 0, 0, 255 ), thickness, lineType);

        cv::imshow("OpenPose Output", outputImg);
        // Display image and sleeps at least 1 ms (it usually sleeps ~5-10 msec to display the image)
        key = (char)cv::waitKey(1);
    }
    else
        op::log("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
    return (key == 27);
}

/*
 * Returns a CvImagePtr.
 */
cv_bridge::CvImagePtr& OpenPoseROSIO::getCvImagePtr()
{
    return cv_img_ptr_;
}


/*
 * Gets detected keypoints from OpenPose and publishes them to ROS topic.
 * Also logs keypoints to console.
 */
void OpenPoseROSIO::publishKeypoints(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr)
{

    // TODO: user-specified flag to control printing output to terminal
    bool consoleOutput = 1;

    // Make sure we have a valid datumsPtr
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {

        // Get all the pose keypoints from OpenPose
        const auto& poseKeypoints = datumsPtr->at(0).poseKeypoints;
        const auto& faceKeypoints = datumsPtr->at(0).faceKeypoints;
        const auto& leftHandKeypoints = datumsPtr->at(0).handKeypoints[0];
        const auto& rightHandKeypoints = datumsPtr->at(0).handKeypoints[1];
        std::vector<op::Rectangle<float>>& face_rectangles = datumsPtr->at(0).faceRectangles;

        // Get Heatmaps from OpenPose
        const auto& poseHeatMaps = datumsPtr->at(0).poseHeatMaps;
        const auto& faceHeatMaps = datumsPtr->at(0).faceHeatMaps;
        const auto& leftHandHeatMaps = datumsPtr->at(0).handHeatMaps[0];
        const auto& rightHandHeatMaps = datumsPtr->at(0).handHeatMaps[1];

        // OpenPose ROS msg
        openpose_ros_msgs::OpenPoseHumanList human_list_msg;
        human_list_msg.header.stamp = ros::Time::now();
        human_list_msg.rgb_image_header = rgb_image_header_;
        human_list_msg.num_humans = poseKeypoints.getSize(0);

        std::vector<openpose_ros_msgs::OpenPoseHuman> human_list(poseKeypoints.getSize(0));

        for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
        {
            openpose_ros_msgs::OpenPoseHuman human;

            int num_body_key_points_with_non_zero_prob = 0;
            for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
            {
                openpose_ros_msgs::PointWithProb body_point_with_prob;
                body_point_with_prob.x = poseKeypoints[{person, bodyPart, 0}];
                body_point_with_prob.y = poseKeypoints[{person, bodyPart, 1}];
                body_point_with_prob.prob = poseKeypoints[{person, bodyPart, 2}];
                if(body_point_with_prob.prob > 0)
                {
                    num_body_key_points_with_non_zero_prob++;
                }
                human.body_key_points_with_prob.at(bodyPart) = body_point_with_prob;
            }
            human.num_body_key_points_with_non_zero_prob = num_body_key_points_with_non_zero_prob;

            if(FLAGS_face)
            {
                int num_face_key_points_with_non_zero_prob = 0;

                for (auto facePart = 0 ; facePart < faceKeypoints.getSize(1) ; facePart++)
                {
                    openpose_ros_msgs::PointWithProb face_point_with_prob;
                    face_point_with_prob.x = faceKeypoints[{person, facePart, 0}];
                    face_point_with_prob.y = faceKeypoints[{person, facePart, 1}];
                    face_point_with_prob.prob = faceKeypoints[{person, facePart, 2}];
                    if(face_point_with_prob.prob > 0)
                    {
                        num_face_key_points_with_non_zero_prob++;
                    }
                    human.face_key_points_with_prob.at(facePart) = face_point_with_prob;
                }
                human.num_face_key_points_with_non_zero_prob = num_face_key_points_with_non_zero_prob;

                openpose_ros_msgs::BoundingBox face_bounding_box;
                face_bounding_box.x = face_rectangles.at(person).x;
                face_bounding_box.y = face_rectangles.at(person).y;
                face_bounding_box.width = face_rectangles.at(person).width;
                face_bounding_box.height = face_rectangles.at(person).height;
                human.face_bounding_box = face_bounding_box;
            }

            if(FLAGS_hand)
            {

                int num_right_hand_key_points_with_non_zero_prob = 0;
                int num_left_hand_key_points_with_non_zero_prob = 0;

                for (auto handPart = 0 ; handPart < rightHandKeypoints.getSize(1) ; handPart++)
                {
                    openpose_ros_msgs::PointWithProb right_hand_point_with_prob;
                    openpose_ros_msgs::PointWithProb left_hand_point_with_prob;
                    right_hand_point_with_prob.x = rightHandKeypoints[{person, handPart, 0}];
                    right_hand_point_with_prob.y = rightHandKeypoints[{person, handPart, 1}];
                    right_hand_point_with_prob.prob = rightHandKeypoints[{person, handPart, 2}];
                    if(right_hand_point_with_prob.prob > 0)
                    {
                        num_right_hand_key_points_with_non_zero_prob++;
                    }
                    left_hand_point_with_prob.x = leftHandKeypoints[{person, handPart, 0}];
                    left_hand_point_with_prob.y = leftHandKeypoints[{person, handPart, 1}];
                    left_hand_point_with_prob.prob = leftHandKeypoints[{person, handPart, 2}];
                    if(left_hand_point_with_prob.prob > 0)
                    {
                        num_left_hand_key_points_with_non_zero_prob++;
                    }
                    human.right_hand_key_points_with_prob.at(handPart) = right_hand_point_with_prob;
                    human.left_hand_key_points_with_prob.at(handPart) = left_hand_point_with_prob;
                }
                human.num_right_hand_key_points_with_non_zero_prob = num_right_hand_key_points_with_non_zero_prob;
                human.num_left_hand_key_points_with_non_zero_prob = num_left_hand_key_points_with_non_zero_prob;
            }

            human_list.at(person) = human;
        }

        human_list_msg.human_list = human_list;

        openpose_human_list_pub_.publish(human_list_msg);


        // If we are logging output to console, log keypoints and heatmaps
        if (consoleOutput)
        {

            printKeypoints(poseKeypoints, faceKeypoints, leftHandKeypoints, rightHandKeypoints);
            printHeatmaps(poseHeatMaps, faceHeatMaps, leftHandHeatMaps, rightHandHeatMaps);

        }

    }
    else
    {
        op::log("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);
    }


    // Return keypoints

}

/*
 * Prints keypoints to console.
 */
template <typename T> void OpenPoseROSIO::printKeypoints(T poseKeypoints, T faceKeypoints,
                                    T leftHandKeypoints, T rightHandKeypoints)
{

    op::log("\nKeypoints:");

    // Log person keypoints
    op::log("Person pose keypoints:");
    for (auto person = 0 ; person < poseKeypoints.getSize(0) ; person++)
    {
        op::log("Person " + std::to_string(person) + " (x, y, score):");
        for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
        {
            std::string valueToPrint;
            for (auto xyscore = 0 ; xyscore < poseKeypoints.getSize(2) ; xyscore++)
                valueToPrint += std::to_string(   poseKeypoints[{person, bodyPart, xyscore}]   ) + " ";
            op::log(valueToPrint);
        }
    }
    op::log(" ");

    // Log hand and face keypoints
    op::log("Face keypoints: " + faceKeypoints.toString());
    op::log("Left hand keypoints: " + leftHandKeypoints.toString());
    op::log("Right hand keypoints: " + rightHandKeypoints.toString());

}

/*
 * Prints heatmaps to console.
 */
template <typename T> void OpenPoseROSIO::printHeatmaps(T poseHeatMaps, T faceHeatMaps,
                                    T leftHandHeatMaps, T rightHandHeatMaps)
{

    if (!poseHeatMaps.empty())
    {
        // Log pose heatmaps
        op::log("Pose heatmaps size: [" + std::to_string(poseHeatMaps.getSize(0)) + ", "
                + std::to_string(poseHeatMaps.getSize(1)) + ", "
                + std::to_string(poseHeatMaps.getSize(2)) + "]");
        // Log face heatmaps
        op::log("Face heatmaps size: [" + std::to_string(faceHeatMaps.getSize(0)) + ", "
                + std::to_string(faceHeatMaps.getSize(1)) + ", "
                + std::to_string(faceHeatMaps.getSize(2)) + ", "
                + std::to_string(faceHeatMaps.getSize(3)) + "]");
        // Log left hand heatmaps
        op::log("Left hand heatmaps size: [" + std::to_string(leftHandHeatMaps.getSize(0)) + ", "
                + std::to_string(leftHandHeatMaps.getSize(1)) + ", "
                + std::to_string(leftHandHeatMaps.getSize(2)) + ", "
                + std::to_string(leftHandHeatMaps.getSize(3)) + "]");
        // Log right hand heatmaps
        op::log("Right hand heatmaps size: [" + std::to_string(rightHandHeatMaps.getSize(0)) + ", "
                + std::to_string(rightHandHeatMaps.getSize(1)) + ", "
                + std::to_string(rightHandHeatMaps.getSize(2)) + ", "
                + std::to_string(rightHandHeatMaps.getSize(3)) + "]");
  }

}
