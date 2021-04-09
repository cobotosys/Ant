/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 2021-4-9          xuzhenhai
============================================================== **/

#include "RobotScene.h"

using namespace Falcon::Camera::Common;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPly(cv::Mat &cloudMat, cv::Mat &colorMat) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int ki = 0; ki < cloudMat.rows; ki = ki + 2) {
        for (int kj = 0; kj < cloudMat.cols; kj = kj + 2) {

            auto x = cloudMat.at<cv::Point3f>(ki, kj).x;
            auto y = cloudMat.at<cv::Point3f>(ki, kj).y;
            auto z = cloudMat.at<cv::Point3f>(ki, kj).z;

            if (z <= 0)
                continue;
            pcl::PointXYZRGB pointXyzrgb;
            pointXyzrgb.x = x / 1000.0;
            pointXyzrgb.y = y / 1000.0;
            pointXyzrgb.z = z / 1000.0;
            //todo 目前支持三通道图片
            pointXyzrgb.b = colorMat.at<cv::Vec3b>(ki, kj)[0];
            pointXyzrgb.g = colorMat.at<cv::Vec3b>(ki, kj)[1];
            pointXyzrgb.r = colorMat.at<cv::Vec3b>(ki, kj)[2];
            result->push_back(pointXyzrgb);
        }
    }
    return result;
}

int main() {
    std::string ROOT_PATH = std::getenv("SDK_PATH");
    std::string filename(ROOT_PATH);
    filename.append("/install/x86-64-install/devel/data/");
    filename.append(__COBOTSYS_MODULE_NAME__);
    filename.append("/aigrasp.cbtx");
    RobotScene robotSence(filename);
    robotSence.openWorkCellInCobotStudio();

#if 0
    {
        std::string ROOT_PATH = std::getenv("SDK_PATH");
        std::string filename(ROOT_PATH);
        filename.append("/install/x86-64-install/devel/data/");
        filename.append(__COBOTSYS_MODULE_NAME__);
        filename.append("/pointcloud/");
        cv::Mat colorMat;
        cv::Mat cloudMat;
        {
            cv::FileStorage fileStorage;
            fileStorage.open(filename + "/0.yaml", cv::FileStorage::READ);

            fileStorage["data"] >> cloudMat;
            fileStorage.release();
            colorMat = cv::imread(filename + "/0.bmp");
            if (colorMat.channels() == 1) {
                cv::cvtColor(colorMat, colorMat, cv::COLOR_GRAY2BGR);
            }
            cobot::studio::common::CobotStudioApi cobotStudioApi;
            auto pc = getPly(cloudMat, colorMat);
            robotSence.getCobotStudioApi().setPointCloud(cloudMat, colorMat, "camera_calibrate_frame",true);
//            cobotStudioApi.gripPointCloud(pc, rw::math::Transform3D<>::identity(), "Camera3D");
        }
    }
#endif
#if 0
    auto camera3D = robotSence.getCamera3DClient();
    auto images = camera3D->capture3DSync();
    boost::shared_ptr<Falcon::Camera::Common::VisionInputImage> cloudImage;
    boost::shared_ptr<Falcon::Camera::Common::VisionInputImage> colorImage;
    for (auto image:images) {
        if (image->type == Falcon::Camera::Common::ImageType::Cloud) {
            cloudImage = image;
        } else if (image->type == Falcon::Camera::Common::ImageType::Mono) {
            colorImage = image;
        }
    }
//    auto pointCloud = getPly(*cloudImage->image, *colorImage->image);
    robotSence.getCobotStudioApi().setPointCloud(*cloudImage->image, *colorImage->image, "camera_calibrate_frame");
#endif
    std::vector<rw::math::Transform3D<>> poses;
    rw::math::Transform3D<> t(rw::math::Vector3D<>(0.135, 0.038, 0.933));
    poses.push_back(t);
    robotSence.getCobotStudioApi().setAxes(poses, "camera_calibrate_frame");

    const rw::math::Transform3D<> &baseToCamera = robotSence.getCobotStudioApi().getTran(
            robotSence.getRobotDevice()->getBase()->getName(), "camera_calibrate_frame");
    t = baseToCamera * t;
    auto qs = robotSence.solver(t);
    rw::math::Q start(6, 3.25274, -2.17372, 2.04448, -1.41033, -1.64286, 0.123342);
    LOG_INFO << "qs size " << qs.size();
    if (qs.size() > 0) {
        rw::math::Q goalQ = qs[0];
        auto qpath = robotSence.query(start, goalQ);
        LOG_INFO << "path size " << qpath.size();

        auto traj = robotSence.makeLinearTrajectory(qpath);
        LOG_INFO << "traj " << traj->duration();

        if (traj->duration() > 0) {
            robotSence.executeTrajectory(traj);
        }
    }
}