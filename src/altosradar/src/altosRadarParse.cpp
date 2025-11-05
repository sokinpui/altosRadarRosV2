#include "pointCloud.h"
#include <algorithm>
#include <arpa/inet.h>
#include <errno.h>
#include <iostream>
#include <math.h>
#include <netinet/in.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>
#include <visualization_msgs/Marker.h>
using namespace std;
#define widthSet 8000
#define vrMax 60
#define vrMin -60
#define vStep 0.5
#define errThr 3
#define PI 3.1415926
#define GROUPIP "224.1.2.4"
#define GROUPPORT 4040
#define LOCALIP "192.168.3.1"
#define UNIPORT 4041
#define UNIFLAG 1

float rcsCal(float range, float azi, float snr, float* rcsBuf) {
    int ind = (azi * 180 / PI + 60.1) * 10;
    float rcs = powf32(range, 2.6) * snr / 5.0e6 / rcsBuf[ind];

    return rcs;
}

int socketGen()
{
    struct sockaddr_in addr;

    struct ip_mreq req;
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
    if(UNIFLAG)
    {
        addr.sin_family = AF_INET;
        addr.sin_port = htons(UNIPORT);
        addr.sin_addr.s_addr = inet_addr(LOCALIP);
        int ret = bind(sockfd, (struct sockaddr*)&addr, sizeof(addr));
        if (-1 == ret) {
            perror("bind");
            return 0;
        }
    }else
    {
        addr.sin_family = AF_INET;
        addr.sin_port = htons(GROUPPORT);
        addr.sin_addr.s_addr = INADDR_ANY;
        int ret = bind(sockfd, (struct sockaddr*)&addr, sizeof(addr));
        if (-1 == ret) {
            perror("bind");
            return 0;
        }

        req.imr_multiaddr.s_addr = inet_addr(GROUPIP);
        req.imr_interface.s_addr = inet_addr(LOCALIP);
        ;
        ret = setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &req, sizeof(req));
        if (ret < 0) {
            perror("setsockopt");
            return 0;
        }
    }


    // req.imr_multiaddr.s_addr = inet_addr(GROUPIP);
    // req.imr_interface.s_addr = inet_addr(/*"0.0.0.0"*/ "192.168.3.1");
    // ;
    // ret = setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &req, sizeof(req));
    // if (ret < 0) {
    //     perror("setsockopt");
    //     return 0;
    // }
    return sockfd;
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

void calPoint(vector<POINTCLOUD> pointCloudVec,pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud,int installFlag,float *rcsBuf,float step,float *histBuf,int pointNumPerPack)
{
    pcl::PointXYZHSV cloudPoint;
    for(int i = 0;i<pointCloudVec.size();i++)
    {
        for(int j = 0;j<pointNumPerPack;j++)
        {
            if(abs(pointCloudVec[i].point[j].range)>0)
            {
                pointCloudVec[i].point[j].ele = installFlag*(pointCloudVec[i].point[j].ele);

                cloudPoint.x = (pointCloudVec[i].point[j].range)*cos(pointCloudVec[i].point[j].azi)*cos(pointCloudVec[i].point[j].ele);
                cloudPoint.y = (pointCloudVec[i].point[j].range)*sin(pointCloudVec[i].point[j].azi)*cos(pointCloudVec[i].point[j].ele);;
                cloudPoint.z = (pointCloudVec[i].point[j].range)*sin(pointCloudVec[i].point[j].ele) ;
                cloudPoint.h = pointCloudVec[i].point[j].doppler;
                cloudPoint.s = rcsCal(pointCloudVec[i].point[j].range,pointCloudVec[i].point[j].azi,pointCloudVec[i].point[j].snr,rcsBuf);
                cloud->push_back(cloudPoint);
            }
        }
    }
    memset(histBuf, 0, sizeof(float) * int((vrMax - vrMin) / step));
    float vrEst = hist(pointCloudVec, histBuf, step);
    float tmp;
    for (int i = 0; i < pointCloudVec.size(); i++) {
        for (int j = 0; j < pointNumPerPack; j++) {
            if(i*pointNumPerPack+j>=cloud->size())
            {
                break;
            }
            if (abs(pointCloudVec[i].point[j].range) > 0) {
                tmp = (cloud->points[i * pointNumPerPack + j].h) -
                      vrEst * cos(pointCloudVec[i].point[j].azi);
                if (tmp < -errThr) {
                    cloud->points[i * pointNumPerPack + j].v = -1;
                } else if (tmp > errThr) {
                    cloud->points[i * pointNumPerPack + j].v = 1;
                } else {
                    cloud->points[i * pointNumPerPack + j].v = 0;
                }
            }
        }
    }
}

int main(int argc, char** argv) {
    // rcs read
    float* rcsBuf = (float*)malloc(1201 * sizeof(float));
    FILE* fp_rcs = fopen("data//rcs.dat", "rb");
    fread(rcsBuf, 1201, sizeof(float), fp_rcs);
    fclose(fp_rcs);

    // ros Init
    ros::init(argc, argv, "altosRadar");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("altosRadar", 1);
    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZHSV>);


    // pointcloud recv para
    vector<POINTCLOUD>  pointCloudVec;
    POINTCLOUD          pointCloudBuf;
    char*               recvBuf = (char*)&pointCloudBuf;
    int                 installFlag = -1;
    int                 pointNumPerPack = 30;
    int                 pointSizeByte = 44;
    int                 recvFrameLen = 0;
    int                 frameNum = 0;
    int                 frameId[2] = {0, 0};
    int                 cntPointCloud[2] = {0, 0};
    float               vrEst = 0;
    unsigned short      curObjInd;
    unsigned char       mode;
    float*              histBuf = (float*)malloc(sizeof(float) * int((vrMax - vrMin) / vStep));

    // pointcloud record init
    // char                filePath[1024];
    // struct              timeval tv;
    // struct              tm tm;
    // gettimeofday(&tv, NULL);
    // localtime_r(&tv.tv_sec, &tm);
    // sprintf(filePath, "data//%d_%d_%d_%d_%d_%d_altos.dat", tm.tm_year + 1900,
    //         tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    // FILE* fp = fopen(filePath, "wb");

    int sockfd = socketGen();
    struct sockaddr_in from;
    socklen_t len = sizeof(from);

    while(ros::ok())
    {
        memset(recvBuf,0,sizeof(POINTCLOUD));
        // int ret = fread(recvBuf,1368,1,fp);
        // printf("ret = %d\n",ret);
        int ret = recvfrom(sockfd, recvBuf, sizeof(POINTCLOUD), 0, (struct sockaddr *)&from, &len);
        if (ret > 0)
		{
            // fwrite(recvBuf, 1, ret, fp);
            curObjInd = pointCloudBuf.pckHeader.curObjInd;
            mode = pointCloudBuf.pckHeader.mode;
            cntPointCloud[mode] = pointCloudBuf.pckHeader.objectCount;
            pointCloudVec.push_back(pointCloudBuf);
            if ((mode == 1 && (curObjInd + 1) * pointNumPerPack >= pointCloudBuf.pckHeader.objectCount)) {
                // if (pointCloudVec.size() * pointNumPerPack < cntPointCloud[0] + cntPointCloud[1]) {
                //     printf(
                //         "FrameId %d %ld Loss %ld pack(s) in %d "
                //         "packs------------------------\n",
                //         pointCloudBuf.pckHeader.frameId, pointCloudVec.size(),
                //         int(ceil(cntPointCloud[0] / pointNumPerPack) +
                //             ceil(cntPointCloud[1] / pointNumPerPack)) -
                //             pointCloudVec.size(),
                //         int(ceil(cntPointCloud[0] / pointNumPerPack) +
                //             ceil(cntPointCloud[1] / pointNumPerPack)));
                // }
                calPoint(pointCloudVec, cloud, installFlag, rcsBuf, vStep,
                         histBuf,pointNumPerPack);
                ros::Duration(0.01).sleep();
                pcl::toROSMsg(*cloud, output);
                output.header.frame_id = "altosRadar";
                printf("0 pointNum of %d frame: %d\n",
                       pointCloudBuf.pckHeader.frameId,
                       cntPointCloud[0] + cntPointCloud[1]);
                output.header.stamp = ros::Time::now();
                pub.publish(output);
                pointCloudVec.clear();
                cloud.reset(new pcl::PointCloud<pcl::PointXYZHSV>);
                cntPointCloud[0] = 0;
                cntPointCloud[1] = 0;
            }
        } else {
            printf("recv failed (timeOut)   %d\n", ret);
        }
    }

    free(histBuf);
    close(sockfd);
    return 0;
}
