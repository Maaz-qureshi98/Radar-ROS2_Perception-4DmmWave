#include "pointCloud.h"
#include <algorithm>
#include <arpa/inet.h>
#include <errno.h>
#include <iostream>
#include <math.h>
#include <netinet/in.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

using namespace std;

#define widthSet 8000
#define PORT 4040
#define vrMax 60
#define vrMin -60
#define errThr 3
#define PI 3.1415926

float rcsCal(float range, float azi, float snr, float* rcsBuf) {
    int ind = (azi * 180 / PI + 60.1) * 10;
    float rcs = powf32(range, 2.6) * snr / 5.0e6 / rcsBuf[ind];

    return rcs;
}

float hist(vector<POINTCLOUD> pointCloudVec, float* histBuf, float step) {
    int ind = 0;
    float vr = 0;

    for (int i = 0; i < pointCloudVec.size(); i++) {
        for (int j = 0; j < 30; j++) {
            if (abs(pointCloudVec[i].point[j].range) > 0) {
                vr = pointCloudVec[i].point[j].doppler /
                     cos(pointCloudVec[i].point[j].azi);
                ind = (vr - vrMin) / step;
                if (vr > 60 || vr < -60 || isnan(vr)) {
                    continue;
                }
                if (vr <= 0) {
                    histBuf[ind]++;
                }
            }
        }
    }
    return float(
               (max_element(histBuf, histBuf + (int((vrMax - vrMin) / step))) -
                histBuf)) *
               step +
           vrMin;
}


void calPoint(vector<POINTCLOUD> pointCloudVec,pcl::PointCloud<pcl::PointXYZHSV> &cloud,int installFlag,float *rcsBuf,float step,float *histBuf)
{

    for(int i = 0;i<pointCloudVec.size();i++)
    {
        for(int j = 0;j<30;j++)
        {
            if(abs(pointCloudVec[i].point[j].range)>0)
            {
                pointCloudVec[i].point[j].ele = installFlag*(pointCloudVec[i].point[j].ele);
                pointCloudVec[i].point[j].azi = -installFlag*asin(sin(pointCloudVec[i].point[j].azi-0*PI/180)/cos(pointCloudVec[i].point[j].ele));
                cloud.points[i*30+j].x = (pointCloudVec[i].point[j].range)*cos(pointCloudVec[i].point[j].azi)*cos(pointCloudVec[i].point[j].ele); 
                cloud.points[i*30+j].y = (pointCloudVec[i].point[j].range)*sin(pointCloudVec[i].point[j].azi)*cos(pointCloudVec[i].point[j].ele);; 
                cloud.points[i*30+j].z = (pointCloudVec[i].point[j].range)*sin(pointCloudVec[i].point[j].ele) ; 
                cloud.points[i*30+j].h = pointCloudVec[i].point[j].doppler; 
                cloud.points[i*30+j].s = rcsCal(pointCloudVec[i].point[j].range,pointCloudVec[i].point[j].azi,pointCloudVec[i].point[j].snr,rcsBuf);
            }
        }
    }
    memset(histBuf, 0, sizeof(float) * int((vrMax - vrMin) / step));
    float vrEst = hist(pointCloudVec, histBuf, step);
    float tmp;
    for (int i = 0; i < pointCloudVec.size(); i++) {
        for (int j = 0; j < 30; j++) {
            if (abs(pointCloudVec[i].point[j].range) > 0) {
                tmp = cloud.points[i * 30 + j].h -
                      vrEst * cos(pointCloudVec[i].point[j].azi);
                if (tmp < -errThr) {
                    cloud.points[i * 30 + j].v = -1;
                } else if (tmp > errThr) {
                    cloud.points[i * 30 + j].v = 1;
                } else {
                    cloud.points[i * 30 + j].v = 0;
                }
            }
        }
    }
}

int main(int argc, char** argv) {
    float* rcsBuf = (float*)malloc(1201 * sizeof(float));
    FILE* fp_rcs = fopen("data//rcs.dat", "rb");
    fread(rcsBuf, 1201, sizeof(float), fp_rcs);
    fclose(fp_rcs);

    rclcpp::init(argc, argv);  // ROS2 Node initialization
    auto node = rclcpp::Node::make_shared("altosRadar");  // Create a shared pointer to the ROS2 node

    // Publishers
    auto pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("altosRadar", 1);
    auto markerPub = node->create_publisher<visualization_msgs::msg::Marker>("TEXT_VIEW_FACING", 10);
    auto originPub = node->create_publisher<visualization_msgs::msg::Marker>("origin", 10);

    // Marker Initialization
    visualization_msgs::msg::Marker origin;
    origin.header.frame_id = "altosRadar";
    origin.type = visualization_msgs::msg::Marker::SPHERE;
    origin.action = visualization_msgs::msg::Marker::ADD;

    origin.pose.position.x = 0;
    origin.pose.position.y = 0;
    origin.pose.position.z = 0;
    origin.pose.orientation.x = 0;
    origin.pose.orientation.y = 0;
    origin.pose.orientation.z = 0;
    origin.pose.orientation.w = 1;

    origin.scale.x = 3;
    origin.scale.y = 3;
    origin.scale.z = 3;
    origin.color.r = 1.0;
    origin.color.g = 1.0;
    origin.color.b = 0.0;
    origin.color.a = 1;

    visualization_msgs::msg::Marker marker;
    marker.ns = "basic_shapes";
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.scale.z = 10;
    marker.color.b = 1.0f;
    marker.color.g = 1.0f;
    marker.color.r = 1.0f;
    marker.color.a = 1;
    
    geometry_msgs::msg::Pose pose;
    pose.position.x = (float)-5;
    pose.position.y = 0;
    pose.position.z = 0;

    sensor_msgs::msg::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZHSV> cloud;
    cloud.width = widthSet * 2;
    cloud.height = 1;
    cloud.points.resize(cloud.width * cloud.height);

    // Socket and Network Initialization
    struct sockaddr_in addr;
    struct sockaddr_in from;

    struct ip_mreq req;
    socklen_t len = sizeof(from);

    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (-1 == sockfd) {
        perror("socket");
        return 0;
    }
    struct timeval timeout = {1, 300};
    setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout,
               sizeof(struct timeval));
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout,
               sizeof(struct timeval));

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);
    addr.sin_addr.s_addr = INADDR_ANY;
    int ret = bind(sockfd, (struct sockaddr*)&addr, sizeof(addr));
    if (-1 == ret) {
        perror("bind");
        return 0;
    }

    req.imr_multiaddr.s_addr = inet_addr("224.1.2.4");
    req.imr_interface.s_addr = inet_addr(/*"0.0.0.0"*/ "192.168.3.1");
    ret = setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &req, sizeof(req));
    if (ret < 0) {
        perror("setsockopt");
        return 0;
    }

    // Additional Variable Initialization
    int recvFrameLen = 0;
    int frameNum = 0;
    int installFlag = -1;
    float step = 0.2;
    vector<POINTCLOUD> pointCloudVec;
    POINTCLOUD pointCloudBuf;
    char* recvBuf = (char*)&pointCloudBuf;
    float* histBuf = (float*)malloc(sizeof(float) * int((vrMax - vrMin) / step));
    unsigned short curObjInd;
    struct timeval tv;
    struct tm tm;
    int frameId[2] = {0, 0};
    int cntPointCloud[2] = {0, 0};
    gettimeofday(&tv, NULL);
    localtime_r(&tv.tv_sec, &tm);
    char filePath[1024];
    float vrEst = 0;
    unsigned char mode;
    sprintf(filePath, "data//%d_%d_%d_%d_%d_%d_altos.dat", tm.tm_year + 1900,
            tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    FILE* fp = fopen(filePath, "wb");
    bool sendFlag = 0;
    long tmpTime = pointCloudBuf.pckHeader.sec;
    FILE *fp_time = fopen("timeVal.txt","wt");
    while (rclcpp::ok()) {
    memset(recvBuf, 0, sizeof(POINTCLOUD));
    ret = recvfrom(sockfd, recvBuf, sizeof(POINTCLOUD), 0, (struct sockaddr*)&from, &len);
    if (ret > 0) {
        if ((pointCloudBuf.pckHeader.mode == 0 && cntPointCloud[1] > 0)) {
            if (pointCloudVec.size() * 30 < cntPointCloud[0] + cntPointCloud[1]) {
                printf("FrameId %d Loss %d pack(s) in %d packs------------------------\n", pointCloudBuf.pckHeader.frameId,
                       int(ceil(cntPointCloud[0] / 30) + ceil(cntPointCloud[1] / 30)) - pointCloudVec.size(),
                       int(ceil(cntPointCloud[0] / 30) + ceil(cntPointCloud[1] / 30)));
            }

            for (int k = 0; k < widthSet * 2; k++) {
                cloud.points[k].x = 0;
                cloud.points[k].y = 0;
                cloud.points[k].z = 0;
                cloud.points[k].h = 0;
                cloud.points[k].s = 0;
            }

            calPoint(pointCloudVec, cloud, installFlag, rcsBuf, step, histBuf);
            rclcpp::sleep_for(std::chrono::milliseconds(5));  // ROS2 sleep equivalent
            pcl::toROSMsg(cloud, output);
            output.header.frame_id = "altosRadar";
            marker.header.frame_id = "altosRadar";
            marker.header.stamp = node->now();  // ROS2 time equivalent
            ostringstream str;
            str << "pointNum:" << cntPointCloud[0] + cntPointCloud[1];
            marker.text = str.str();
            marker.pose = pose;
            markerPub->publish(marker);
            output.header.stamp = node->now();
            pub->publish(output);
            pointCloudVec.clear();
            cntPointCloud[0] = 0;
            cntPointCloud[1] = 0;
            struct timeval tv;
            long long t1;
            gettimeofday(&tv, NULL);
            t1 = tv.tv_sec * 1000ll + tv.tv_usec / 1000;
            localtime_r(&tmpTime, &tm);
            fprintf(fp_time, "1:%f\n", t1 / 1e3);
        }

        fwrite(recvBuf, 1, ret, fp);
        curObjInd = pointCloudBuf.pckHeader.curObjInd;
        mode = pointCloudBuf.pckHeader.mode;
        cntPointCloud[mode] = pointCloudBuf.pckHeader.objectCount;
        pointCloudVec.push_back(pointCloudBuf);

        if (mode == 1 && (curObjInd + 1) * 30 >= pointCloudBuf.pckHeader.objectCount) {
            if (pointCloudVec.size() * 30 < cntPointCloud[0] + cntPointCloud[1]) {
                printf("FrameId %d %ld Loss %ld pack(s) in %d packs------------------------\n",
                       pointCloudBuf.pckHeader.frameId, pointCloudVec.size(),
                       int(ceil(cntPointCloud[0] / 30) + ceil(cntPointCloud[1] / 30)) -
                           pointCloudVec.size(),
                       int(ceil(cntPointCloud[0] / 30) + ceil(cntPointCloud[1] / 30)));
            }

            for (int k = 0; k < widthSet * 2; k++) {
                cloud.points[k].x = 0;
                cloud.points[k].y = 0;
                cloud.points[k].z = 0;
                cloud.points[k].h = 0;
                cloud.points[k].s = 0;
            }

            calPoint(pointCloudVec, cloud, installFlag, rcsBuf, step, histBuf);
            rclcpp::sleep_for(std::chrono::milliseconds(5));  // ROS2 sleep equivalent
            pcl::toROSMsg(cloud, output);
            output.header.frame_id = "altosRadar";
            marker.header.frame_id = "altosRadar";
            marker.header.stamp = node->now();  // ROS2 time equivalent
            ostringstream str;
            str << "pointNum:" << cntPointCloud[0] + cntPointCloud[1];
            printf("pointNum of %d frame: %d\n",
                   pointCloudBuf.pckHeader.frameId,
                   cntPointCloud[0] + cntPointCloud[1]);
            marker.text = str.str();
            marker.pose = pose;
            markerPub->publish(marker);
            output.header.stamp = node->now();
            pub->publish(output);
            pointCloudVec.clear();
            cntPointCloud[0] = 0;
            cntPointCloud[1] = 0;
            struct timeval tv;
            long long t1;
            gettimeofday(&tv, NULL);
            t1 = tv.tv_sec * 1000ll + tv.tv_usec / 1000;
            localtime_r(&tmpTime, &tm);
            fprintf(fp_time, "2:%f\n", t1 / 1e3);
        }
    } else {
        printf("recv failed (timeOut)   %d\n", ret);
    }
}

close(sockfd);
free(histBuf);
fclose(fp);
fclose(fp_time);
return 0;

}
