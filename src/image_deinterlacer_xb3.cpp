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
#include "sensor_msgs/CameraInfo.h"
#include "image_transport/image_transport.h"
#include "image_transport/camera_publisher.h"

#include "camera_info_manager/camera_info_manager.h"

class xb3ImageDeinterlacer {
private:
    ros::NodeHandle leftNh,centerNh,rightNh;
    sensor_msgs::CameraInfo leftInfo,centerInfo,rightInfo;
    image_transport::CameraPublisher pLeftImage,pRightImage,pCenterImage;
    sensor_msgs::Image left,right,center;

public:
    xb3ImageDeinterlacer(ros::NodeHandle n1, ros::NodeHandle n2, ros::NodeHandle n3) {

        leftNh=n1;
        centerNh=n2;
        rightNh=n3;

        image_transport::ImageTransport itLeft(leftNh);
        pLeftImage = itLeft.advertiseCamera("image_raw", 1);
//        camera_info_manager::CameraInfoManager leftInfoMgr(leftNh);
//        leftInfoMgr.loadCameraInfo("package://xb3_driver/cal_left.yaml");
//        leftInfo = leftInfoMgr.getCameraInfo();
//        leftInfo.header.frame_id = "/camera_frame";

        image_transport::ImageTransport itCenter(centerNh);
        pCenterImage = itCenter.advertiseCamera("image_raw", 1);
//        camera_info_manager::CameraInfoManager centerInfoMgr(centerNh);
//        centerInfoMgr.loadCameraInfo("package://xb3_driver/cal_center.yaml");
//        centerInfo = centerInfoMgr.getCameraInfo();
//        centerInfo.header.frame_id = "/camera_frame";

        image_transport::ImageTransport itRight(rightNh);
        pRightImage = itRight.advertiseCamera("image_raw", 1);
//        camera_info_manager::CameraInfoManager rightInfoMgr(rightNh);
//        rightInfoMgr.loadCameraInfo("package://xb3_driver/cal_right.yaml");
//        rightInfo = rightInfoMgr.getCameraInfo();
//        rightInfo.header.frame_id = "/camera_frame";



        left.header.frame_id = "/camera_left_frame";
        left.height=960; //leftInfo.height;
        left.width=1280; //leftInfo.width;
        left.step=left.width;
        left.encoding="mono8";

        center.header.frame_id = "/camera_center_frame";
        center.height=960;//centerInfo.height;
        center.width=1280;//centerInfo.width;
        center.step=center.width;
        center.encoding="mono8";

        right.header.frame_id = "/camera_right_frame";
        right.height=960;//rightInfo.height;
        right.width=1280;//rightInfo.width;
        right.step=right.width;
        right.encoding="mono8";

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

        int j = 0;
        for(unsigned int i=0; i<img->data.size(); i+=3) {

            right.data[j] = img->data[i];
            center.data[j] = img->data[i+1];
            left.data[j] = img->data[i+2];
            j++;
//            right.data.push_back(img->data[i]);
//            center.data.push_back(img->data[i+1]);
//            left.data.push_back(img->data[i+2]);
        }

        leftInfo.header.stamp=left.header.stamp=img->header.stamp;
        pLeftImage.publish(left,leftInfo);

        centerInfo.header.stamp=center.header.stamp=img->header.stamp;
        pCenterImage.publish(center,centerInfo);

        rightInfo.header.stamp=right.header.stamp=img->header.stamp;
        pRightImage.publish(right,rightInfo);
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "xb3_deinterlacer");
    ros::NodeHandle n;
    ros::NodeHandle npLeft("xb3_left"),npRight("xb3_right"),npCenter("xb3_center");
    xb3ImageDeinterlacer imgDI(npLeft,npCenter,npRight);
    ros::Subscriber sub1 = n.subscribe("/camera/image_raw", 1, &xb3ImageDeinterlacer::imageCallback, &imgDI);
    ros::spin();
    return 0;
}

