/*
 *  RPLIDAR ROS NODE
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "rplidar.h"
#include "util/ParamReader.h"
#include <chrono>
#include <cmath>
#include <memory.h>
#include "hector_slam.h"

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)

using namespace rp::standalone::rplidar;

RPlidarDriver * drv = NULL;


// @todo
// struct LaserScan{
//     float seq;  //数据类型 ID
//     std::chrono::steady_clock::time_point stamp;
//     std::string frame_id;
//     float angle_min;
//     float angle_max;
//     float angle_increment;
//     float time_increment;
//     float scan_time;
//     float range_min;
//     float range_max;
//     std::vector<float> ranges;
//     std::vector<float> intensities;
// };

//该函数通过进程间通讯的方法进行改进
LaserScan& publish_scan(//ros::Publisher *pub,
                  rplidar_response_measurement_node_hq_t *nodes,
                  size_t node_count,std::chrono::steady_clock::time_point start,
                  double scan_time, bool inverted,
                  float angle_min, float angle_max,
                  float max_distance,
                  std::string frame_id)
{
    static int scan_count = 0;
    LaserScan* scan_msg = new LaserScan;

    scan_msg->stamp = start;
    scan_msg->frame_id = frame_id;
    scan_count++;

    bool reversed = (angle_max > angle_min);
    if ( reversed ) {
      scan_msg->angle_min =  M_PI - angle_max;
      scan_msg->angle_max =  M_PI - angle_min;
    } else {
      scan_msg->angle_min =  M_PI - angle_min;
      scan_msg->angle_max =  M_PI - angle_max;
    }
    scan_msg->angle_increment =
        (scan_msg->angle_max - scan_msg->angle_min) / (double)(node_count-1);

    scan_msg->scan_time = scan_time;
    scan_msg->time_increment = scan_time / (double)(node_count-1);
    scan_msg->range_min = 0.15;
    scan_msg->range_max = max_distance;//8.0;

    scan_msg->intensities.resize(node_count);
    scan_msg->ranges.resize(node_count);
    bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
    if (!reverse_data) {
        for (size_t i = 0; i < node_count; i++) {
            float read_value = (float) nodes[i].dist_mm_q2/4.0f/1000;
            if (read_value == 0.0)
                scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
            else
                scan_msg->ranges[i] = read_value;
            scan_msg->intensities[i] = (float) (nodes[i].quality >> 2);
        }
    } else {
        for (size_t i = 0; i < node_count; i++) {
            float read_value = (float)nodes[i].dist_mm_q2/4.0f/1000;
            if (read_value == 0.0)
                scan_msg->ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
            else
                scan_msg->ranges[node_count-1-i] = read_value;
            scan_msg->intensities[node_count-1-i] = (float) (nodes[i].quality >> 2);
        }
    }
    return *scan_msg;
}



bool getRPLIDARDeviceInfo(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_info_t devinfo;

    op_result = drv->getDeviceInfo(devinfo);
    if (IS_FAIL(op_result)) {
        if (op_result == RESULT_OPERATION_TIMEOUT) {
            cout<<"Error, operation time out. RESULT_OPERATION_TIMEOUT! "<<endl;
        } else {
            cout<<"Error, unexpected error, code: "<<op_result<<endl;
        }
        return false;
    }

    // print out the device serial number, firmware and hardware version number..
    cout<<"RPLIDAR S/N: "<<endl;
    for (int pos = 0; pos < 16 ;++pos) {
        cout<<hex<<devinfo.serialnum[pos];
    }
    cout<<endl;
    cout<<"Firmware Ver: "<< (devinfo.firmware_version>>8) <<"."<<(devinfo.firmware_version & 0xFF)<<endl;
    cout<<"Hardware Rev: "<<(int)devinfo.hardware_version<<endl;
    return true;
}

bool checkRPLIDARHealth(RPlidarDriver * drv)
{
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;
    op_result = drv->getHealth(healthinfo);
    if (IS_OK(op_result)) { 
        cout<<"RPLidar health status : "<< int(healthinfo.status)<<endl;
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            cout<<"Error, rplidar internal error detected. Please reboot the device to retry."<<endl;
            return false;
        } else {
            return true;
        }

    } else {
        cout<<"Error, cannot retrieve rplidar health code: "<< op_result<<endl;
        return false;
    }
}

//bool stop_motor(std_srvs::Empty::Request &req,
//                               std_srvs::Empty::Response &res)
//{
//  if(!drv)
//       return false;
//
//  ROS_DEBUG("Stop motor");
//  drv->stopMotor();
//  return true;
//}

//bool start_motor(std_srvs::Empty::Request &req,
//                               std_srvs::Empty::Response &res)
//{
//  if(!drv)
//       return false;
//  if(drv->isConnected())
//  {
//      ROS_DEBUG("Start motor");
//      u_result ans=drv->startMotor();
//
//      ans=drv->startScan(0,1);
//   }
//   else printf("lost connection");
//  return true;
//}

static float getAngle(const rplidar_response_measurement_node_hq_t& node)
{
    return node.angle_z_q14 * 90.f / 16384.f;
}

int main(int argc, char * argv[]) {
    
    /* 初始化 */
    std::string channel_type;
    std::string tcp_ip;
    std::string serial_port;
    int tcp_port = 20108;
    int serial_baudrate = 115200;
    std::string frame_id;
    bool inverted = false;
    bool angle_compensate = false;
    float max_distance = 8.0;
    //角度补偿
    int angle_compensate_multiple = 1;//it stand of angle compensate at per 1 degree
    std::string scan_mode;
    std::string hector_param_file;

    ParamterReader paramter("../param/rplidar.txt");
    channel_type = paramter.getString("channel_type","serial");
    tcp_ip = paramter.getString("tcp_ip","192.168.0.7");
    tcp_port = paramter.getData<int>("tcp_port",20108);
    serial_port = paramter.getString("serial_port","/dev/ttyUSB0");
    // serial_baudrate = paramter.getData<int>("serial_baudrate",115200);
    frame_id = paramter.getString("frame_id","laser_frame");
    inverted = paramter.getData<bool>("inverted",false);
    angle_compensate = paramter.getData<bool>("angle_compensate",false);
    scan_mode = paramter.getString("scan_mode",string());
    hector_param_file = paramter.getString("hector_param_file",std::string());



    u_result    op_result;

    // 创建 driver
    if(channel_type == "tcp"){
        drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_TCP);
    }
    else{
        drv = RPlidarDriver::CreateDriver(rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT);
    }

    if (!drv) {
        cout<<"Create Driver fail, exit"<<endl;
        return -2;
    }

    // 连接 connect
    if(channel_type == "tcp"){
        // make connection...
        if (IS_FAIL(drv->connect(tcp_ip.c_str(), (_u32)tcp_port))) {
            cout<<"Error, cannot bind to the specified serial port "<< serial_port.c_str()<<endl;
            RPlidarDriver::DisposeDriver(drv);
            return -1;
        }

    }
    else{
       // make connection...
        if (IS_FAIL(drv->connect(serial_port.c_str(), (_u32)serial_baudrate))) {
             cout<<"Error, cannot bind to the specified serial port "<<serial_port.c_str()<<endl;
            RPlidarDriver::DisposeDriver(drv);
            return -1;
        }

    }
    
    // get rplidar device info
    if (!getRPLIDARDeviceInfo(drv)) {
        return -1;
    }

    // check health...
    if (!checkRPLIDARHealth(drv)) {
        RPlidarDriver::DisposeDriver(drv);
        return -1;
    }

    // 开启 电机
    drv->startMotor();

    RplidarScanMode current_scan_mode;
    // 判断扫描模式
    if (scan_mode.empty()) {    // 为空用典型扫描模式
        op_result = drv->startScan(false /* not force scan */, true /* use typical scan mode */, 0, &current_scan_mode);
    }
    else {                      // 检测当前模式是否合理
        std::vector<RplidarScanMode> allSupportedScanModes;
        op_result = drv->getAllSupportedScanModes(allSupportedScanModes);

        if (IS_OK(op_result)) {     // 获得了所有的扫描模式
            _u16 selectedScanMode = _u16(-1);
            for (std::vector<RplidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                if (iter->scan_mode == scan_mode) {
                    selectedScanMode = iter->id;
                    break;
                }
            }

            if (selectedScanMode == _u16(-1)) {     // 没找到匹配的模式
                cout<<"scan mode " <<scan_mode.c_str()<< " is not supported by lidar, supported modes:"<<endl;
                for (std::vector<RplidarScanMode>::iterator iter = allSupportedScanModes.begin(); iter != allSupportedScanModes.end(); iter++) {
                    cout<<iter->scan_mode<<" max_distance: "<<iter->max_distance<<" m, Point number: "  
                        <<(1000/iter->us_per_sample)<<endl;
                }
                op_result = RESULT_OPERATION_FAIL;
            } 
            else {                                  // 找到扫描模型开启
                op_result = drv->startScanExpress(false /* not force scan */, selectedScanMode, 0, &current_scan_mode);
            }
        }
    }

    if(IS_OK(op_result))
    {
        //default frequent is 10 hz (by motor pwm value),  current_scan_mode.us_per_sample is the number of scan point per us
        angle_compensate_multiple = (int)(1000*1000/current_scan_mode.us_per_sample/10.0/360.0);
        //1000/us_per_sample=point/ms *1000 = point/s /10 = point/0.1s /360 = point/度  in一圈 
        if(angle_compensate_multiple < 1) 
          angle_compensate_multiple = 1;
        max_distance = current_scan_mode.max_distance;
        cout<<"current scan mode: "<<current_scan_mode.scan_mode<<" max_distance: "<<current_scan_mode.max_distance
            <<" m, Point number: "<<(1000/current_scan_mode.us_per_sample)<<" K , angle_compensate: "<<angle_compensate_multiple 
            <<endl;
    }
    else
    {
        cout<<"Can not start scan: %08x!"<<op_result<<endl;
    }

    std::chrono::steady_clock::time_point start_scan_time;
    std::chrono::steady_clock::time_point end_scan_time;
    std::chrono::duration<double> scan_duration;

    HectorSLAM hector_slam(hector_param_file);

    while (1) { //@todo 这里要做一个类似于ros::ok()一样的东西
        rplidar_response_measurement_node_hq_t nodes[360*8];
        size_t   count = _countof(nodes); //= 360*8

        start_scan_time = std::chrono::steady_clock::now();
        op_result = drv->grabScanDataHq(nodes, count);
        end_scan_time = std::chrono::steady_clock::now();
        scan_duration = std::chrono::duration_cast<std::chrono::duration<double>>(start_scan_time-end_scan_time);

        if (op_result == RESULT_OK) {
            op_result = drv->ascendScanData(nodes, count);
            float angle_min = DEG2RAD(0.0f);
            float angle_max = DEG2RAD(359.0f);
            if (op_result == RESULT_OK) {
                if (angle_compensate) {
                    //const int angle_compensate_multiple = 1;
                    const int angle_compensate_nodes_count = 360*angle_compensate_multiple;
                    int angle_compensate_offset = 0;
                    rplidar_response_measurement_node_hq_t angle_compensate_nodes[angle_compensate_nodes_count];
                    memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(rplidar_response_measurement_node_hq_t));

                    int i = 0, j = 0;
                    for( ; i < count; i++ ) {
                        if (nodes[i].dist_mm_q2 != 0) {
                            float angle = getAngle(nodes[i]);
                            int angle_value = (int)(angle * angle_compensate_multiple);
                            if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
                            for (j = 0; j < angle_compensate_multiple; j++) {

                                int angle_compensate_nodes_index = angle_value-angle_compensate_offset+j;
                                if(angle_compensate_nodes_index >= angle_compensate_nodes_count)
                                    angle_compensate_nodes_index = angle_compensate_nodes_count-1;
                                angle_compensate_nodes[angle_compensate_nodes_index] = nodes[i];
                            }
                        }
                    }
  
                    LaserScan& scan = publish_scan( angle_compensate_nodes, angle_compensate_nodes_count,
                             start_scan_time, scan_duration.count(), inverted,
                             angle_min, angle_max, max_distance,
                             frame_id);
                    
                    if(hector_slam.shut_down_)
                        break;
                    hector_slam.scanCallback(scan);
                }
                else {
                    int start_node = 0, end_node = 0;
                    int i = 0;
                    // find the first valid node and last valid node
                    while (nodes[i++].dist_mm_q2 == 0);
                    start_node = i-1;
                    i = count -1;
                    while (nodes[i--].dist_mm_q2 == 0);
                    end_node = i+1;

                    angle_min = DEG2RAD(getAngle(nodes[start_node]));
                    angle_max = DEG2RAD(getAngle(nodes[end_node]));

                    LaserScan& scan = publish_scan(&nodes[start_node], end_node-start_node +1,
                             start_scan_time, scan_duration.count(), inverted,
                             angle_min, angle_max, max_distance,
                             frame_id);

                    if(hector_slam.shut_down_)
                        break;
                    cout<<hector_slam.shut_down_<<endl;
                    hector_slam.scanCallback(scan);

               }
            }
            else if (op_result == RESULT_OPERATION_FAIL) {
                // All the data is invalid, just publish them
                float angle_min = DEG2RAD(0.0f);
                float angle_max = DEG2RAD(359.0f);
                LaserScan& scan = publish_scan( nodes, count,
                             start_scan_time, scan_duration.count(), inverted,
                             angle_min, angle_max, max_distance,
                             frame_id);
                if(hector_slam.shut_down_)
                    break;
                hector_slam.scanCallback(scan);

            }
        }
        if(inverted ==1) break;
        // ros::spinOnce();
    }

    // done!
    drv->stopMotor();
    drv->stop();
    RPlidarDriver::DisposeDriver(drv);
    return 0;
}
