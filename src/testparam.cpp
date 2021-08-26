#include "util/ParamReader.h"
#include "opencv2/opencv.hpp"
using namespace  std;


void InitParams(string filename){
    ParamterReader paramterReader(filename);
    int state = paramterReader.getData<int>("state",0);
    bool use_odom_ = paramterReader.getData<bool>("use_odom",false);
    double p_map_pub_period_ = paramterReader.getData<double>("map_pub_period",10);
    string load_map_path_ = paramterReader.getString("load_map_path",std::string());
    double p_map_resolution_ = paramterReader.getData<double>("map_resolution",0.05);
    int p_map_size_ = paramterReader.getData<int>("map_size",2048);
    double p_map_start_x_ = paramterReader.getData<double>("map_start_x",0.5);
    double p_map_start_y_ = paramterReader.getData<double>("map_start_y",0.5);
    int p_map_multi_res_levels_ = paramterReader.getData<int>("map_multi_res_levels",3);
    double p_update_factor_free_ = paramterReader.getData<double>("update_factor_free",0.4);
    double p_update_factor_occupied_ = paramterReader.getData<double>("update_factor_occupied",0.9);
    double p_map_update_distance_threshold_ = paramterReader.getData<double>("map_update_distance_thresh",0.4);
    double p_map_update_angle_threshold_ = paramterReader.getData<double>("map_update_angle_thresh",0.9);
    bool save_map_ = paramterReader.getData<bool>("save_map",false);
    string save_map_path_ = paramterReader.getString("save_map_path",std::string());
//    p_base_frame_ = paramterReader.getString("map_size","fas");
//    p_map_frame_ = paramterReader.getData<int>("map_size",1024);
//    p_odom_frame_ = paramterReader.getData<int>("map_size",1024);


    cout<<"p_map_resolution_"<<p_map_resolution_<<endl;
    cout<<"p_map_size_"<<p_map_size_<<endl;
    cout<<"p_update_factor_free_"<<p_update_factor_free_<<endl;
    cout<<"p_map_multi_res_levels_"<<p_map_multi_res_levels_<<endl;
    cerr<<"save_map_:"<<save_map_<<std::endl;
}

int main(){

//    InitParams("../param/hector_slam.txt");
    cv::Mat mat = cv::imread("../map/map.png",-1);
    for(int i = 0;i<mat.rows;i++){
        uchar* datas = mat.ptr<uchar>(i);
        for(int j=0;j<mat.cols;j++){
            if(datas[j]==255)
                cout<<int(datas[j])<<" ";
        }
        cout<<endl;
        cout<<endl;
        cout<<endl;
        cout<<endl;
    }

    return 0;


}