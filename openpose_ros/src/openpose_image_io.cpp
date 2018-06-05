#include <openpose_image_io.h>

using namespace openpose_ros;

/*
 * Subscribes to the input video feed and starts processing.
 */
OpenPoseImageIO::OpenPoseImageIO(OpenPose &openPose): it_(nh_)
{
    // Subscribe to input video feed and publish human lists as output
    std::string image_topic;
    std::string output_topic;

    nh_.param("/openpose_ros_image_node/image_topic", image_topic, std::string("/camera/image_raw"));
    nh_.param("/openpose_ros_image_node/output_topic", output_topic, std::string("/openpose_ros/human_list"));

    image_sub_ = it_.subscribe(image_topic, 1, &OpenPoseImageIO::processImage, this);
    openpose_human_list_pub_ = nh_.advertise<openpose_ros_msgs::OpenPoseHumanList>(output_topic, 10);
    cv_img_ptr_ = nullptr;
    openpose_ = &openPose;
}


/*
 * Callback function for image topic subscriber.
 * Converts incoming image to OpenCV format and sends to OpenPose to process.
 * After OpenPose has processed image, sends frame to extraction functions:
 *    keypoint extraction
 *    heatmap extraction
 *    body part extraction
 *    face feature extraction
 * Then publishes message containing extracted features to ROS topic and
 * displays updated frame in window.
 */
void OpenPoseImageIO::processImage(const sensor_msgs::ImageConstPtr& msg)
{
    convertImage(msg);
    std::shared_ptr<std::vector<op::Datum>> datumToProcess = createDatum();

    bool successfullyEmplaced = openpose_->waitAndEmplace(datumToProcess);

    // Pop frame
    std::shared_ptr<std::vector<op::Datum>> datumProcessed;

    // If OpenPose successfully processes frame
    if (successfullyEmplaced && openpose_->waitAndPop(datumProcessed))
    {

        openpose_ros_msgs::OpenPoseHumanList human_list;

        // Get updated keypoints from OpenPose
        human_list = processFrame(datumProcessed);

        // Publish keypoints from OpenPose to ROS topic and log to console
        publishKeypoints(human_list);

        // Display frame in window
        display(datumProcessed, human_list);

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
void OpenPoseImageIO::convertImage(const sensor_msgs::ImageConstPtr& msg)
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
std::shared_ptr<std::vector<op::Datum>> OpenPoseImageIO::createDatum()
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
bool OpenPoseImageIO::display(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr,
                            const openpose_ros_msgs::OpenPoseHumanList human_list)
{

    // User's displaying/saving/other processing here
    // datum.cvOutputData: rendered frame with pose or heatmaps
    // datum.poseKeypoints: Array<float> with the estimated pose
    char key = ' ';
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {

        // Get rendered image from OpenPose
        cv::Mat outputImg = datumsPtr->at(0).cvOutputData;

        // get number of humans
        int num_humans = human_list.num_humans;

        // If we're drawing a bounding box for the face, get points and draw
        // TODO: handle multiple humans! Right now only handling one
        if (FLAGS_face && num_humans > 0)
        {

            openpose_ros_msgs::BoundingBox face_bounding_box = human_list.human_list[0].face_bounding_box;
            float face_x = face_bounding_box.x;
            float face_y = face_bounding_box.y;
            float face_width = face_bounding_box.width;
            float face_height = face_bounding_box.height;

            // Draw face bounding box in green on image
            cv::rectangle(outputImg, cv::Point(face_x, face_y), cv::Point(face_x+face_width, face_y+face_height),
                                      cv::Scalar(0,255,0), 2);

        }

        // TODO: If we're drawing bounding boxes for hands, get points and draw
        if (FLAGS_hand && num_humans > 0)
        {
            // Draw bounding box for left hand
            openpose_ros_msgs::BoundingBox left_hand_bounding_box = human_list.human_list[0].left_hand_bounding_box;
            float left_x = left_hand_bounding_box.x;
            float left_y = left_hand_bounding_box.y;
            float left_width = left_hand_bounding_box.width;
            float left_height = left_hand_bounding_box.height;

            cv::rectangle(outputImg, cv::Point(left_x, left_y), cv::Point(left_x+left_width, left_y+left_height),
                                      cv::Scalar(255,0,0), 2);

            // Draw bounding box for left hand
            openpose_ros_msgs::BoundingBox right_hand_bounding_box = human_list.human_list[0].right_hand_bounding_box;
            float right_x = right_hand_bounding_box.x;
            float right_y = right_hand_bounding_box.y;
            float right_width = right_hand_bounding_box.width;
            float right_height = right_hand_bounding_box.height;

            cv::rectangle(outputImg, cv::Point(right_x, right_y), cv::Point(right_x+right_width, right_y+right_height),
                                     cv::Scalar(255,0,0), 2);

        }


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
cv_bridge::CvImagePtr& OpenPoseImageIO::getCvImagePtr()
{
    return cv_img_ptr_;
}


/*
 * Gets detected keypoints from OpenPose.
 * Returns ROS msg containing list of humans and keypoints for each human.
 */
openpose_ros_msgs::OpenPoseHumanList OpenPoseImageIO::processFrame(
                                const std::shared_ptr<std::vector<op::Datum>>& datumsPtr)
{

    // TODO: user-specified flag to control printing output to terminal
    bool consoleOutput = 1;

    // Make sure we have a valid datumsPtr
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {


        // Get number of detected humans from OpenPose
        const auto& poseKeypoints = datumsPtr->at(0).poseKeypoints;
        int num_humans = poseKeypoints.getSize(0);

        // OpenPoseHumanList ROS msg
        openpose_ros_msgs::OpenPoseHumanList human_list_msg;
        human_list_msg.header.stamp = ros::Time::now();
        human_list_msg.rgb_image_header = rgb_image_header_;
        human_list_msg.num_humans = num_humans;

        // Vector containing OpenPoseHuman msg with keypoints for each human
        std::vector<openpose_ros_msgs::OpenPoseHuman> human_list(num_humans);


        // Process every human detected in the frame
        for (auto person = 0 ; person < num_humans; person++)
        {

            // Message for storing each individual human
            openpose_ros_msgs::OpenPoseHuman human;

            // Extract detected body parts
            human = extractBodyPartKeypoints(poseKeypoints, human, person);

            // If OpenPose is supposed to get face keypoints
            if (FLAGS_face)
            {
                  // Face keypoints and bounding box from OpenPose
                  const auto& faceKeypoints = datumsPtr->at(0).faceKeypoints;
                  std::vector<op::Rectangle<float>>& face_rectangles = datumsPtr->at(0).faceRectangles;
                  human = extractFaceKeypoints(faceKeypoints, face_rectangles, human, person);
            }

            // If OpenPose is supposed to detect hands
            if (FLAGS_hand)
            {

                // Hand keypoints and bounding boxes from OpenPose
                const auto& leftHandKeypoints = datumsPtr->at(0).handKeypoints[0];
                const auto& rightHandKeypoints = datumsPtr->at(0).handKeypoints[1];
                std::vector<std::array<op::Rectangle<float>, 2>>& hand_rectangles = datumsPtr->at(0).handRectangles;
                human = extractHandKeypoints(leftHandKeypoints, rightHandKeypoints,
                                             hand_rectangles, human, person);

            }

            // Get heatmaps from OpenPose
            //human_list_msg = getHeatMaps(datumsPtr, human_list_msg);

            // If we are logging output to console, log keypoints and heatmaps
            if (consoleOutput)
            {

                //printKeypoints(poseKeypoints, faceKeypoints, leftHandKeypoints, rightHandKeypoints);
                //printHeatmaps(poseHeatMaps, faceHeatMaps, leftHandHeatMaps, rightHandHeatMaps);

            }

            human_list.at(person) = human;
        }

        human_list_msg.human_list = human_list;

        // Return list of humans
        return human_list_msg;

    }
    else
    {
        op::log("Nullptr or empty datumsPtr found.", op::Priority::High, __LINE__, __FUNCTION__, __FILE__);

        // Return null
        // TODO: test for nullptr later when publishing, etc
        openpose_ros_msgs::OpenPoseHumanList human_list_msg;
        return human_list_msg;
    }

}



/*
 * Extracts keypoints for each body point given a human from OpenPose.
 */
template <typename T> openpose_ros_msgs::OpenPoseHuman OpenPoseImageIO::extractBodyPartKeypoints(
                              T poseKeypoints, openpose_ros_msgs::OpenPoseHuman human, int person_num)
{

        int num_body_key_points_with_non_zero_prob = 0;

        // For each detected body part, get keypoints and store
        for (auto bodyPart = 0 ; bodyPart < poseKeypoints.getSize(1) ; bodyPart++)
        {
            openpose_ros_msgs::PointWithProb body_point_with_prob;
            body_point_with_prob.x = poseKeypoints[{person_num, bodyPart, 0}];
            body_point_with_prob.y = poseKeypoints[{person_num, bodyPart, 1}];
            body_point_with_prob.prob = poseKeypoints[{person_num, bodyPart, 2}];

            if(body_point_with_prob.prob > 0)
            {
                num_body_key_points_with_non_zero_prob++;
            }
            human.body_key_points_with_prob.at(bodyPart) = body_point_with_prob;
        }
        human.num_body_key_points_with_non_zero_prob = num_body_key_points_with_non_zero_prob;

    return human;

}

/*
 * Extracts keypoints for each face given a human from OpenPose.
 */
template <typename T> openpose_ros_msgs::OpenPoseHuman OpenPoseImageIO::extractFaceKeypoints(
                              T faceKeypoints, std::vector<op::Rectangle<float>>& face_rectangles,
                              openpose_ros_msgs::OpenPoseHuman human, int person_num)
{

    int num_face_key_points_with_non_zero_prob = 0;

    for (auto facePart = 0 ; facePart < faceKeypoints.getSize(1) ; facePart++)
    {
        openpose_ros_msgs::PointWithProb face_point_with_prob;
        face_point_with_prob.x = faceKeypoints[{person_num, facePart, 0}];
        face_point_with_prob.y = faceKeypoints[{person_num, facePart, 1}];
        face_point_with_prob.prob = faceKeypoints[{person_num, facePart, 2}];

        if(face_point_with_prob.prob > 0)
        {
            num_face_key_points_with_non_zero_prob++;
        }

        human.face_key_points_with_prob.at(facePart) = face_point_with_prob;
    }
    human.num_face_key_points_with_non_zero_prob = num_face_key_points_with_non_zero_prob;

    // Get points for face bounding box
    openpose_ros_msgs::BoundingBox face_bounding_box;
    face_bounding_box.x = face_rectangles.at(person_num).x;
    face_bounding_box.y = face_rectangles.at(person_num).y;
    face_bounding_box.width = face_rectangles.at(person_num).width;
    face_bounding_box.height = face_rectangles.at(person_num).height;
    human.face_bounding_box = face_bounding_box;

    return human;

}


/*
 * Extracts keypoints for each hand given a human from OpenPose.
 */
template <typename T> openpose_ros_msgs::OpenPoseHuman OpenPoseImageIO::extractHandKeypoints(
                              T leftHandKeypoints, T rightHandKeypoints,
                              std::vector<std::array<op::Rectangle<float>, 2>>& hand_rectangles,
                              openpose_ros_msgs::OpenPoseHuman human, int person_num)
{

    int num_right_hand_key_points_with_non_zero_prob = 0;
    int num_left_hand_key_points_with_non_zero_prob = 0;

    for (auto handPart = 0 ; handPart < rightHandKeypoints.getSize(1) ; handPart++)
    {

        // Process right hand
        openpose_ros_msgs::PointWithProb right_hand_point_with_prob;
        right_hand_point_with_prob.x = rightHandKeypoints[{person_num, handPart, 0}];
        right_hand_point_with_prob.y = rightHandKeypoints[{person_num, handPart, 1}];
        right_hand_point_with_prob.prob = rightHandKeypoints[{person_num, handPart, 2}];

        if(right_hand_point_with_prob.prob > 0)
        {
            num_right_hand_key_points_with_non_zero_prob++;
        }

        human.right_hand_key_points_with_prob.at(handPart) = right_hand_point_with_prob;

        // Process left hand
        openpose_ros_msgs::PointWithProb left_hand_point_with_prob;
        left_hand_point_with_prob.x = leftHandKeypoints[{person_num, handPart, 0}];
        left_hand_point_with_prob.y = leftHandKeypoints[{person_num, handPart, 1}];
        left_hand_point_with_prob.prob = leftHandKeypoints[{person_num, handPart, 2}];

        if(left_hand_point_with_prob.prob > 0)
        {
            num_left_hand_key_points_with_non_zero_prob++;
        }

        human.left_hand_key_points_with_prob.at(handPart) = left_hand_point_with_prob;
    }

    human.num_right_hand_key_points_with_non_zero_prob = num_right_hand_key_points_with_non_zero_prob;
    human.num_left_hand_key_points_with_non_zero_prob = num_left_hand_key_points_with_non_zero_prob;

    // Get points for left hand bounding box
    openpose_ros_msgs::BoundingBox left_hand_bounding_box;
    left_hand_bounding_box.x = hand_rectangles.at(person_num)[0].x;
    left_hand_bounding_box.y = hand_rectangles.at(person_num)[0].y;
    left_hand_bounding_box.width = hand_rectangles.at(person_num)[0].width;
    left_hand_bounding_box.height = hand_rectangles.at(person_num)[0].height;
    human.left_hand_bounding_box = left_hand_bounding_box;

    // Get points for right hand bounding box
    openpose_ros_msgs::BoundingBox right_hand_bounding_box;
    right_hand_bounding_box.x = hand_rectangles.at(person_num)[1].x;
    right_hand_bounding_box.y = hand_rectangles.at(person_num)[1].y;
    right_hand_bounding_box.width = hand_rectangles.at(person_num)[1].width;
    right_hand_bounding_box.height = hand_rectangles.at(person_num)[1].height;
    human.right_hand_bounding_box = right_hand_bounding_box;

    return human;

}


/*
 * TODO: extract heatmaps
 */
openpose_ros_msgs::OpenPoseHumanList OpenPoseImageIO::getHeatMaps(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr,
                                        openpose_ros_msgs::OpenPoseHumanList human_list)
{

    // Get Heatmaps from OpenPose
    const auto& poseHeatMaps = datumsPtr->at(0).poseHeatMaps;
    const auto& faceHeatMaps = datumsPtr->at(0).faceHeatMaps;
    const auto& leftHandHeatMaps = datumsPtr->at(0).handHeatMaps[0];
    const auto& rightHandHeatMaps = datumsPtr->at(0).handHeatMaps[1];


}


/*
 * Publishes OpenPoseHumanList to ROS topic.
 */
void OpenPoseImageIO::publishKeypoints(const openpose_ros_msgs::OpenPoseHumanList human_list)
{

    openpose_human_list_pub_.publish(human_list);

}

/*
 * Prints keypoints to console.
 */
template <typename T> void OpenPoseImageIO::printKeypoints(T poseKeypoints, T faceKeypoints,
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
template <typename T> void OpenPoseImageIO::printHeatmaps(T poseHeatMaps, T faceHeatMaps,
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
