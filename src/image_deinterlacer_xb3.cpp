/**
 * Image deinterlacer xb3 subscribes to image_raw output of the
 * camera1394 driver, which is reading the data in format7_mode3
 * rgb8 mode from a Bumblebee XB3.
 * The received image is then deinterlaced and three seperate images are published:
 * xb3_left, xb3_right and xb3_center
 *
 * The code is based on http://answers.ros.org/question/66375/bumblebee-xb3-driver/
 *
 */

#include "ros/ros.h"

#include "image_transport/image_transport.h"
#include "image_transport/camera_publisher.h"

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "sensor_msgs/CameraInfo.h"
#include "camera_info_manager/camera_info_manager.h"

class xb3ImageDeinterlacer {
private:
    ros::NodeHandle left_nh_, right_nh_,center_nh_,nh_;
    image_transport::ImageTransport left_it_, right_it_, center_it_;
    image_transport::CameraPublisher left_pub_, right_pub_, center_pub_;
    image_transport::Subscriber image_sub_;

    sensor_msgs::CameraInfo leftInfo,centerInfo,rightInfo;
    sensor_msgs::Image left,right,center;
    std::vector<cv::Mat> spl;

public:
    xb3ImageDeinterlacer(ros::NodeHandle left_nh,
                         ros::NodeHandle center_nh,
                         ros::NodeHandle right_nh) :
        left_it_(left_nh),
        left_pub_(left_it_.advertiseCamera("image_raw", 1)),
        right_it_(right_nh),
        right_pub_(right_it_.advertiseCamera("image_raw", 1)),
        center_it_(center_nh),
        center_pub_(center_it_.advertiseCamera("image_raw", 1))
    {

        camera_info_manager::CameraInfoManager leftInfoMgr(left_nh);
        leftInfoMgr.loadCameraInfo("package://image_deinterlacer_xb3/xb3_driver/cal_left.yaml");
        leftInfo = leftInfoMgr.getCameraInfo();
        leftInfo.header.frame_id = "/camera";

        camera_info_manager::CameraInfoManager centerInfoMgr(center_nh);
        centerInfoMgr.loadCameraInfo("package://image_deinterlacer_xb3/xb3_driver/cal_center.yaml");
        centerInfo = centerInfoMgr.getCameraInfo();
        centerInfo.header.frame_id = "/camera";

        //        camera_info_manager::CameraInfoManager rightInfoMgr(rightNh);
        //        rightInfoMgr.loadCameraInfo("package://xb3_driver/cal_right.yaml");
        //        rightInfo = rightInfoMgr.getCameraInfo();
        //        rightInfo.header.frame_id = "/camera";

        left.header.frame_id = "/camera_left_frame";
        left.encoding=sensor_msgs::image_encodings::BAYER_GBRG8;
        left.height=leftInfo.height;
        left.width=leftInfo.width;
        left.step=left.width;

        leftInfo.roi.y_offset = 0;
        leftInfo.roi.x_offset = left.width/4 - 1;
        leftInfo.roi.height = left.height;
        leftInfo.roi.width = left.width/2;
        leftInfo.roi.do_rectify = true;

        center.header.frame_id = "/camera_center_frame";
        center.encoding=sensor_msgs::image_encodings::BAYER_GBRG8;
        center.height=centerInfo.height;
        center.width=centerInfo.width;
        center.step=center.width;

        centerInfo.roi.y_offset = 0;
        centerInfo.roi.x_offset = center.width/4 - 1;
        centerInfo.roi.height = center.height;
        centerInfo.roi.width = center.width/2;
        centerInfo.roi.do_rectify = true;

        right.header.frame_id = "/camera_right_frame";
        right.encoding=sensor_msgs::image_encodings::BAYER_GBRG8;
        right.height=960;//rightInfo.height;
        right.width=1280;//rightInfo.width;
        right.step=right.width;

        left.data.reserve(960*1280);
        center.data.reserve(960*1280);
        right.data.reserve(960*1280);

        for(unsigned int i=0; i<960*1280*3; i+=3) {
            right.data.push_back(0);
            center.data.push_back(0);
            left.data.push_back(0);
        }

    }

    void imageCallback(const sensor_msgs::ImagePtr& img){

        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

        try {
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // split into 3 cameras
        cv::split(cv_ptr->image, spl);

        // cast cv::Mats to std::vectors
        const unsigned char* p_left = spl[2].ptr<unsigned char>(0);
        std::vector<unsigned char> left_vec(p_left, p_left + spl[2].cols * spl[2].rows);

        const unsigned char* p_center = spl[1].ptr<unsigned char>(0);
        std::vector<unsigned char> center_vec(p_center, p_center + spl[1].cols* spl[1].rows);

        const unsigned char* p_right = spl[0].ptr<unsigned char>(0);
        std::vector<unsigned char> right_vec(p_right, p_right + spl[0].cols* spl[0].rows);


        // Publish cameras
        left.header.stamp = img->header.stamp;
        leftInfo.header.stamp = img->header.stamp;
        left.data = left_vec;
        left_pub_.publish(left,leftInfo);

        center.header.stamp = img->header.stamp;
        centerInfo.header.stamp = img->header.stamp;
        center.data = center_vec;
        center_pub_.publish(center,centerInfo);

        right.header.stamp = img->header.stamp;
        right.data = right_vec;
//        right_pub_.publish(right,rightInfo);
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "xb3_deinterlacer");
    ros::NodeHandle n;
    ros::NodeHandle npLeft("left"),npCenter("right"),npRight("most_right");
    xb3ImageDeinterlacer imgDI(npLeft,npCenter,npRight);
    ros::Subscriber sub1 = n.subscribe("/camera/image_raw", 1, &xb3ImageDeinterlacer::imageCallback, &imgDI);
    ros::spin();
    return 0;
}

