#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <string>
#include <iostream>
#include <csignal>
#include <unistd.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>


#include "IMU_Processing.hpp"

#include "sensor_msgs/NavSatFix.h"
#include "gnss_data.hpp"
#include <boost/filesystem.hpp>


#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001) 
#define PUBFRAME_PERIOD     (20)

int    add_point_size = 0, kdtree_delete_counter = 0;
int edge_plane_flag = 3;
bool   pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f; 
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;  
string map_file_path, lid_topic, imu_topic, gps_topic, truth_topic;

double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;  
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;  
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double filter_size_edge_min = 0, filter_size_inten_edge_min = 0;
double cube_len = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    scan_count = 0, publish_count = 0;
int    feats_down_size = 0, NUM_MAX_ITERATIONS = 0, pcd_save_interval = -1, pcd_index = 0;
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false, pub_gps_path = false, pub_odom_path = false, is_our_weight = false, pub_truth_path = false;


vector<BoxPointType> cub_needrm;  
vector<PointVector>  Nearest_Points;    
vector<double>       extrinT(3, 0.0);    //雷达相对于IMU的外参T
vector<double>       extrinR(9, 0.0);    //雷达相对于IMU的外参R
deque<double>                     time_buffer;           // 激光雷达数据时间戳缓存队列
deque<PointCloudXYZI::Ptr>        lidar_buffer;  //记录特征提取或间隔采样后的lidar（特征）数据
deque<double>        feature_time_buffer;  //记录特征提取或间隔采样后的lidar（特征）数据
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;// IMU数据缓存队列

//一些点云变量
PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI()); //提取地图中的特征点，IKD-tree获得
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());   //去畸变的特征

//TODO czy
//PointCloudXYZI::Ptr feats_intensity_undistort(new PointCloudXYZI());   //去畸变的特征
PointCloudXYZI::Ptr feats_undistort_s(new PointCloudXYZI());   //去畸变的几何面特征
PointCloudXYZI::Ptr feats_undistort_p(new PointCloudXYZI());   //去畸变的几何线特征
PointCloudXYZI::Ptr feats_undistort_ip(new PointCloudXYZI());   //去畸变的强度线特征
PointCloudXYZI::Ptr feats_down_body_s(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_p(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_ip(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_ip_tmp(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_ip_window(new PointCloudXYZI());
pcl::VoxelGrid<PointType> downSizeFilterPlane;//单帧内降采样使用voxel grid
pcl::VoxelGrid<PointType> downSizeFilterIntenPlane;//单帧内降采样使用voxel grid
int    feats_down_size_s = 0 , feats_down_size_p  = 0, feats_down_size_ip = 0;
int feats_down_body_ip_window_num = 0;
bool is_reset_window_point = true;
bool  is_intensity_edge_windows = false;
int    window_point_size_ip = 0;
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());          //畸变纠正后降采样的单帧点云，lidar系
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());         //畸变纠正后降采样的单帧点云，W系

//下采样的体素点云
pcl::VoxelGrid<PointType> downSizeFilterSurf;//单帧内降采样使用voxel grid
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree; // ikd-tree类

V3D Lidar_T_wrt_IMU(Zero3d);  // T lidar to imu (imu = r * lidar + t)
M3D Lidar_R_wrt_IMU(Eye3d);       // R lidar to imu (imu = r * lidar + t)

/*** EKF inputs and output ***/
// ESEKF操作
MeasureGroup Measures;

esekfom::esekf kf;

state_ikfom state_point;        // 状态
Eigen::Vector3d pos_lid;    // world系下lidar坐标

//输出的路径参数
nav_msgs::Path path;//包含了一系列位姿
nav_msgs::Odometry odomAftMapped; //只包含了一个位姿
geometry_msgs::PoseStamped msg_body_pose;//位姿

//激光和imu处理操作
shared_ptr<Preprocess> p_pre(new Preprocess());// 定义指向激光雷达数据的预处理类Preprocess的智能指针

//按下ctrl+c后唤醒所有线程
void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();//  会唤醒所有等待队列中阻塞的线程 线程被唤醒后，会通过轮询方式获得锁，获得锁前也一直处理运行状态，不会被再次阻塞。
}

bool CreateFile(std::ofstream& ofs, std::string file_path) {
    ofs.open(file_path.c_str(), std::ios::app);
    if (!ofs) {
       std::cout << "无法生成文件: " << file_path << std::endl;
        return false;
    }

    return true;
}

bool CreateDirectory(std::string directory_path) {
    if (!boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::create_directory(directory_path);
    }
    if (!boost::filesystem::is_directory(directory_path)) {
        std::cout << "无法建立文件夹: " << directory_path << std::endl;
        return false;
    }
    return true;
}


  void pubImage(const ros::Publisher pubImage, const cv::Mat& this_image, std_msgs::Header this_header, string image_format)
  {
      static cv_bridge::CvImage bridge;
      bridge.header = this_header;
      bridge.encoding = image_format;
      bridge.image = this_image;
      pubImage.publish(bridge.toImageMsg());
  }

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    mtx_buffer.lock();//加锁
    scan_count ++;// wzl没用到
    double preprocess_start_time = omp_get_wtime();//记录时间 wzl没用到
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
        feature_time_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);// 点云预处理

    double delt_t  =  p_pre->avia_delt_t;
    feature_time_buffer.push_back(delt_t);

    lidar_buffer.push_back(ptr);   //将点云放入缓冲区
    time_buffer.push_back(msg->header.stamp.toSec());    //将时间放入缓冲区
    last_timestamp_lidar = msg->header.stamp.toSec();   //记录最后一个时间
    mtx_buffer.unlock();
    sig_buffer.notify_all();// 唤醒所有线程
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;

bool Sync_truth_Data_my(std::deque<geometry_msgs::PoseStamped>& UnsyncedData, double & x,double & y, double & z,double sync_time)
{
    while (UnsyncedData.size() >= 2) {
        //std::cout <<  "UnsyncedData.front().time -  sync_time  =" << UnsyncedData.front().time -  sync_time << std::endl;
        if (UnsyncedData.front().header.stamp.toSec() > sync_time)
            return false;
        //std::cout <<  "UnsyncedData.at(1).time -  sync_time  =" << UnsyncedData.at(1).time -  sync_time << std::endl;
        if (UnsyncedData.at(1).header.stamp.toSec() < sync_time) {
            UnsyncedData.pop_front();
            continue;
        }
        if (sync_time - UnsyncedData.front().header.stamp.toSec() > 0.5) {
            UnsyncedData.pop_front();
            break;
        }
        if (UnsyncedData.at(1).header.stamp.toSec() - sync_time > 0.5) {
            UnsyncedData.pop_front();
            break;
        }
        break;
    }
    if (UnsyncedData.size() < 2)
        return false;

    geometry_msgs::PoseStamped front_data = UnsyncedData.at(0);
    geometry_msgs::PoseStamped back_data = UnsyncedData.at(1);

    double front_scale = (back_data.header.stamp.toSec() - sync_time) / (back_data.header.stamp.toSec() - front_data.header.stamp.toSec());
    double back_scale = (sync_time - front_data.header.stamp.toSec()) / (back_data.header.stamp.toSec() - front_data.header.stamp.toSec());
    x = front_data.pose.position.x * front_scale + back_data.pose.position.x * back_scale;
    y = front_data.pose.position.y * front_scale + back_data.pose.position.y * back_scale;
    z = front_data.pose.position.z * front_scale + back_data.pose.position.z * back_scale;

    return true;
}


bool SyncData_my(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time) {
    while (UnsyncedData.size() >= 2) {
        //std::cout <<  "UnsyncedData.front().time -  sync_time  =" << UnsyncedData.front().time -  sync_time << std::endl;
        if (UnsyncedData.front().time > sync_time)
            return false;
        //std::cout <<  "UnsyncedData.at(1).time -  sync_time  =" << UnsyncedData.at(1).time -  sync_time << std::endl;
        if (UnsyncedData.at(1).time < sync_time) {
            UnsyncedData.pop_front();
            continue;
        }
        if (sync_time - UnsyncedData.front().time > 0.5) {
            UnsyncedData.pop_front();
            break;
        }
        if (UnsyncedData.at(1).time - sync_time > 0.5) {
            UnsyncedData.pop_front();
            break;
        }
        break;
    }
    if (UnsyncedData.size() < 2)
        return false;

    GNSSData front_data = UnsyncedData.at(0);
    GNSSData back_data = UnsyncedData.at(1);
    GNSSData synced_data;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.status = back_data.status;
    synced_data.longitude = front_data.longitude * front_scale + back_data.longitude * back_scale;
    synced_data.latitude = front_data.latitude * front_scale + back_data.latitude * back_scale;
    synced_data.altitude = front_data.altitude * front_scale + back_data.altitude * back_scale;
    synced_data.local_E = front_data.local_E * front_scale + back_data.local_E * back_scale;
    synced_data.local_N = front_data.local_N * front_scale + back_data.local_N * back_scale;
    synced_data.local_U = front_data.local_U * front_scale + back_data.local_U * back_scale;

    SyncedData.push_back(synced_data);
    
    return true;
}

void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) 
{
    mtx_buffer.lock();    // 互斥锁
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;    
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
        feature_time_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }
    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;// 标记已经进行时间同步
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    double delt_t  =  p_pre->avia_delt_t;
    feature_time_buffer.push_back(delt_t);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);
    
    mtx_buffer.unlock();
    sig_buffer.notify_all();// 唤醒所有线程
}
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    //std::cout << "imu数据"<< std::endl;
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());//将IMU时间戳对齐到激光雷达时间戳
    }

    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);// 系统默认都需要进行的时间校准

    double timestamp = msg->header.stamp.toSec();// IMU时间戳

    mtx_buffer.lock();
    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }
    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();  //解锁
    sig_buffer.notify_all();// 唤醒阻塞的线程，当持有锁的线程释放锁时，这些线程中的一个会获得锁。而其余的会接着尝试获得锁
}

std::deque<GNSSData> ungnss_data_buff;
std::deque<GNSSData> gnss_data_buff;
GeographicLib::LocalCartesian geo_converter;
bool  origin_position_inited = false;

void gps_cbk(const sensor_msgs::NavSatFixConstPtr &gps_in) 
{
    GNSSData gnss_data;
    gnss_data.time = gps_in->header.stamp.toSec();
    gnss_data.latitude = gps_in->latitude;
    gnss_data.longitude = gps_in->longitude;
    gnss_data.altitude = gps_in->altitude;
    gnss_data.status = gps_in->status.status;
    gnss_data.service = gps_in->status.service;

    mtx_buffer.lock();
    ungnss_data_buff.push_back(gnss_data);
    mtx_buffer.unlock();  //解锁
    sig_buffer.notify_all();// 唤醒阻塞的线程，当持有锁的线程释放锁时，这些线程中的一个会获得锁。而其余的会接着尝试获得锁
}

std::deque<geometry_msgs::PoseStamped> untruth_data_buff;
std::deque<geometry_msgs::PoseStamped> truth_data_buff;
void truth_cbk(const geometry_msgs::PoseStampedConstPtr &truth_in) 
{
    //std::cout << " untruth_data_buff.size()= " << untruth_data_buff.size()<<std::endl;
    geometry_msgs::PoseStamped truth_data;
    truth_data.header.seq = truth_in->header.seq;
    truth_data.header.stamp = truth_in->header.stamp;
    truth_data.header.frame_id = truth_in->header.frame_id;
    truth_data.pose.position.x = truth_in->pose.position.x;
    truth_data.pose.position.y = truth_in->pose.position.y;
    truth_data.pose.position.z = truth_in->pose.position.z;

    mtx_buffer.lock();
    untruth_data_buff.push_back(truth_data);
    std::cout << " untruth_data_buff.size()= " << untruth_data_buff.size()<<std::endl;
    mtx_buffer.unlock();  //解锁
    sig_buffer.notify_all();// 唤醒阻塞的线程，当持有锁的线程释放锁时，这些线程中的一个会获得锁。而其余的会接着尝试获得锁
}


double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) //如果缓存队列中没有数据，则返回false
    {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {

        meas.lidar = lidar_buffer.front();
        if(!lidar_buffer.empty() )
        {
            meas.feature_time_ = feature_time_buffer.front();
        }
        meas.lidar_beg_time = time_buffer.front();       
        meas.is_truth = false;
        if (meas.lidar->points.size() <= 5) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime; //此次雷达点云结束时刻
            ROS_WARN("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime; //此次雷达点云结束时刻
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000); //此次雷达点云结束时刻
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;
        lidar_pushed = true;

    }
    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    double lidar_beg_time = meas.lidar_beg_time;

    if(pub_truth_path)
    {
        if(!untruth_data_buff.empty())
        {
            double x,y,z;
            bool sync_truth_data_flag = Sync_truth_Data_my(untruth_data_buff, x,y,z, lidar_beg_time);

            if(sync_truth_data_flag)
            {
                    meas.x_ = x;
                    meas.y_ = y;
                    meas.z_ = z;
                    meas.is_truth = true;
            }
        
        }

    }

    if(pub_gps_path)
    {
        if(origin_position_inited == true && !ungnss_data_buff.empty())
        {
            bool sync_flag = SyncData_my(ungnss_data_buff, gnss_data_buff, lidar_beg_time);
            if(sync_flag == true)
            {
                if(gnss_data_buff.at(0).time == lidar_beg_time)
                {
                    meas.gnssdata = gnss_data_buff.at(0);
                    meas.is_truth = true;
                    gnss_data_buff.pop_front();
                    //std::cout << "true|lidar_beg_time = " << lidar_beg_time << std::endl;
                }
            }
        }
        if(origin_position_inited == false && !ungnss_data_buff.empty())
        {
            bool sync_flag = SyncData_my(ungnss_data_buff, gnss_data_buff, lidar_beg_time);
            //std::cout << "sync_flag = " << sync_flag << std::endl;
            if(sync_flag == true)
            {
                if(gnss_data_buff.at(0).time == lidar_beg_time)
                {
                    origin_position_inited = true;
                    geo_converter.Reset( gnss_data_buff.at(0).latitude, gnss_data_buff.at(0).longitude , gnss_data_buff.at(0).altitude );
                    meas.gnssdata = gnss_data_buff.at(0);
                    meas.is_truth = true;
                    gnss_data_buff.pop_front(); 
                    //std::cout << "false|lidar_beg_time = " << lidar_beg_time << std::endl;
                }
            }
        }
    }
    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();//获取imu数据的时间戳
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front()); //将imu数据放到meas中
        imu_buffer.pop_front();
    }



    lidar_buffer.pop_front();//将lidar数据弹出
    if(!lidar_buffer.empty() )
    {
        feature_time_buffer.pop_front();
    }
    time_buffer.pop_front();//将时间戳弹出
    lidar_pushed = false;  //将lidar_pushed置为false，代表lidar数据已经被放到meas中了
    return true;
}

void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot.matrix() * (state_point.offset_R_L_I.matrix()*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
    po->normal_z = pi->normal_z;
}
template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot.matrix() * (state_point.offset_R_L_I.matrix()*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

BoxPointType LocalMap_Points;           // ikd-tree地图立方体的2个角点
bool Localmap_Initialized = false;      // 局部地图是否初始化
void lasermap_fov_segment()
{
    cub_needrm.clear();     // 清空需要移除的区域
    kdtree_delete_counter = 0;

    V3D pos_LiD = pos_lid;      // global系下lidar位置
    if (!Localmap_Initialized)
    { 
        for (int i = 0; i < 3; i++)
        {
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;  
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        //与包围盒最小值边界点距离
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints); // 移除较远包围盒
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    PointVector points_history;
    ikdtree.acquire_removed_points(points_history); //返回被剔除的点

    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm); //删除指定范围内的点
}
void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I.matrix()*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
         if( feats_down_body->points[i].normal_y > 3 )
             continue;
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
    
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType mid_point;   //点所在体素的中心
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]); //如果距离最近的点都在体素外，则该点不需要Downsample
                continue;
            }
            for (int j = 0; j < NUM_MATCH_POINTS; j ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[j], mid_point) < dist)  //如果近邻点距离 < 当前点距离，不添加该点
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1)); 
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());  
void publish_frame_world(const ros::Publisher & pubLaserCloudFull_)
{
    if(scan_pub_en) 
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);//判断是否需要降采样
        int size = laserCloudFullRes->points.size(); //获取待转换点云的大小
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));//创建一个点云用于存储转换到世界坐标系的点云

        for (int i = 0; i < size; i++)
        {
            pointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);//从激光雷达坐标系转换到世界坐标系
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull_.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            pointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);//转换到世界坐标系
        }
        *pcl_wait_save += *laserCloudWorld;  // 把结果压入到pcd中

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body_intensity_gradient(const ros::Publisher & publish_frame_body_intensity_gradient )
{
    int size = feats_down_body_ip->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));//创建一个点云用于存储转换到IMU系的点云

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(& feats_down_body_ip->points[i], \
                            &laserCloudIMUBody->points[i]);//转换到IMU坐标系
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    publish_frame_body_intensity_gradient.publish(laserCloudmsg);
    //publish_count -= PUBFRAME_PERIOD;
}

void publish_frame_body_geo_plane(const ros::Publisher & publish_frame_body_geo_plane )
{
    int size = feats_down_body_s->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_down_body_s->points[i], \
                            &laserCloudIMUBody->points[i]);//转换到IMU坐标系
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    publish_frame_body_geo_plane.publish(laserCloudmsg);
    //publish_count -= PUBFRAME_PERIOD;
}

void publish_frame_body_geo_edge(const ros::Publisher & publish_frame_body_geo_edge )
{
    int size = feats_down_body_p->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));//创建一个点云用于存储转换到IMU系的点云

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_down_body_p->points[i], \
                            &laserCloudIMUBody->points[i]);//转换到IMU坐标系
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    publish_frame_body_geo_edge.publish(laserCloudmsg);
    //publish_count -= PUBFRAME_PERIOD;
}

void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);//转换到IMU坐标系
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}
void publish_map(const ros::Publisher & pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}
template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);//将eskf求得的位置传入
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);

    auto q_ = Eigen::Quaterniond(state_point.rot.matrix());//将eskf求得的姿态传入
    out.pose.orientation.x = q_.coeffs()[0];
    out.pose.orientation.y = q_.coeffs()[1];
    out.pose.orientation.z = q_.coeffs()[2];
    out.pose.orientation.w = q_.coeffs()[3];   
}
void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped.publish(odomAftMapped);

    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body" ) );//发布tf变换
}
void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;
    nh.param<bool>("publish/path_en",path_en, true);
    nh.param<bool>("publish/scan_publish_en",scan_pub_en, true);             
    nh.param<bool>("publish/dense_publish_en",dense_pub_en, true);          
    nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true);    
    nh.param<int>("max_iteration",NUM_MAX_ITERATIONS,4);                 
    nh.param<string>("map_file_path",map_file_path,"");                   
    nh.param<string>("common/lid_topic",lid_topic,"/livox/lidar");        
    nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");          
    nh.param<string>("common/gps_topic", gps_topic,"/rostopic echo /leica/pose/relative");          
    nh.param<string>("common/truth_topic", truth_topic,"/leica/pose/relative");    
    nh.param<bool>("common/time_sync_en", time_sync_en, false);           
    nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    nh.param<double>("filter_size_corner",filter_size_corner_min,0.5);     
    nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);   
    nh.param<double>("filter_size_map",filter_size_map_min,0.5);      
    nh.param<double>("cube_side_length",cube_len,200);                    
    nh.param<float>("mapping/det_range",DET_RANGE,300.f);       
    nh.param<double>("mapping/fov_degree",fov_deg,180);             
    nh.param<double>("mapping/gyr_cov",gyr_cov,0.1);                    
    nh.param<double>("mapping/acc_cov",acc_cov,0.1);                     
    nh.param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);                
    nh.param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);               
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);               
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);     
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);           
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);       
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);  
    nh.param<int>("edge_plane_flag", edge_plane_flag, 3); 
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);           
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>()); 
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>()); 
    
    nh.param<double>("preprocess/image_gradient_threshold",p_pre->image_gradient_threshold,50);
    nh.param<double>("filter_size_edge",filter_size_edge_min,0.5);    // VoxelGrid降采样时的体素大小
    nh.param<double>("filter_size_inten_edge",filter_size_inten_edge_min,0.5);    // VoxelGrid降采样时的体素大小


    nh.param<float>("preprocess/intensity_min_range",p_pre->intensity_min_range,3);
    nh.param<float>("preprocess/current_range_max",p_pre->current_range_max, 20);
    nh.param<float>("preprocess/current_range_max_livox_hap",p_pre->current_range_max_livox_hap, 500);

    nh.param<float>("preprocess/intensity_dient_min",p_pre->intensity_dient_min,10);
    nh.param<float>("preprocess/intensity_dient_max",p_pre->intensity_dient_max,15);
    nh.param<float>("preprocess/fov_degree",p_pre->fov_degree,180);
    nh.param<float>("preprocess/vertical_degree",p_pre->vertical_degree,30);
    nh.param<float>("preprocess/vertical_degree_min",p_pre->vertical_degree_min,-15);
    nh.param<float>("preprocess/vertical_degree_max",p_pre->vertical_degree_max,15);
    nh.param<float>("preprocess/delete_fov_degree",p_pre->delete_fov_degree,0);
    nh.param<float>("preprocess/range_blind",p_pre->range_blind,2);
    nh.param<float>("preprocess/image_fov_resolution",p_pre->image_fov_resolution,0.2);
    nh.param<int>("preprocess/image_vertical_rate",p_pre->image_vertical_rate,4);
    nh.param<int>("preprocess/lookfor_min_num",p_pre->lookfor_min_num,2);
    nh.param<int>("preprocess/lookfor_max_num",p_pre->lookfor_max_num,1);
    nh.param<bool>("publish/pub_gps_path",pub_gps_path, false); //pub_truth_path
    nh.param<bool>("publish/pub_truth_path",pub_truth_path, false); //pub_truth_path
    nh.param<bool>("publish/pub_odom_path",pub_odom_path, false); 
    nh.param<bool>("mapping/is_our_weight",is_our_weight, true); 
    nh.param<bool>("mapping/is_intensity_edge_windows",is_intensity_edge_windows, false); 

    std::string WORK_SPACE_PATH = "/home/bupo/my_study/ig_lio_git/ig_lio";

    std::cout << "几何面点的滤波器参数为 filter_size_surf_min = " << filter_size_surf_min << std::endl;
    std::cout << "几何线点的滤波器参数为 filter_size_edge_min = " << filter_size_edge_min << std::endl;
    std::cout << "强度线点的滤波器参数为 filter_size_inten_edge_min = " << filter_size_inten_edge_min << std::endl;
    cout<<"Lidar_type: "<<p_pre->lidar_type<<endl;

    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="camera_init";
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);

    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    ros::Subscriber sub_gps = nh.subscribe(gps_topic, 200000, gps_cbk);
    ros::Subscriber sub_truth = nh.subscribe(truth_topic, 200000, truth_cbk);
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);

    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);

    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/Odometry", 100000);

    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path> ("/path", 100000);
    ros::Publisher pub_image_intensity  = nh.advertise<sensor_msgs::Image>("/image_intensity", 1);
    // 发布距离图像
    ros::Publisher pub_image_range  = nh.advertise<sensor_msgs::Image>("/image_range", 1);
    // 发布强度梯度图像
    ros::Publisher pub_image_intensity_gradient  = nh.advertise<sensor_msgs::Image>("/image_intensity_gradient", 1);
    ros::Publisher pub_pl_intensity_gradient = nh.advertise<sensor_msgs::PointCloud2>("/cloud_intensity_gradient", 100000);
    ros::Publisher pub_pl_geo_plane = nh.advertise<sensor_msgs::PointCloud2>("/cloud_geo_plane", 100000);
    ros::Publisher pub_pl_geo_edge = nh.advertise<sensor_msgs::PointCloud2>("/cloud_geo_edge", 100000);

    downSizeFilterPlane.setLeafSize(filter_size_edge_min, filter_size_edge_min, filter_size_edge_min);
    downSizeFilterIntenPlane.setLeafSize(filter_size_inten_edge_min, filter_size_inten_edge_min, filter_size_inten_edge_min);
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);


    shared_ptr<ImuProcess> p_imu1(new ImuProcess());
    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu1->set_param(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU, V3D(gyr_cov, gyr_cov, gyr_cov), V3D(acc_cov, acc_cov, acc_cov), 
                        V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov), V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    signal(SIGINT, SigHandle);

    ros::Rate rate(5000);

    while (ros::ok())
    {
        if (flg_exit) break;
        ros::spinOnce();
        if(sync_packages(Measures))  
        {
            if (flg_first_scan)
            {
                feats_down_body_ip_tmp.reset(new PointCloudXYZI());
                first_lidar_time = Measures.lidar_beg_time;
                p_imu1->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }

            double feature_time_cur = Measures.feature_time_;
            double t00 = omp_get_wtime();
            p_imu1->Process(Measures, kf, feats_undistort);  

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot.matrix() * state_point.offset_T_L_I;
            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;
            lasermap_fov_segment();    
        
            feats_undistort_ip.reset(new PointCloudXYZI());
            feats_undistort_p.reset(new PointCloudXYZI());
            feats_undistort_s.reset(new PointCloudXYZI());
            feats_down_body_ip_tmp.reset(new PointCloudXYZI());

            float plane_range_ave = 0;
            int before_plane_volter_num = 0;
            float delta_range_ave = 0; 

            for(int i = 0; i < feats_undistort->points.size(); i++)
            {
                PointType tmp_pt;
                tmp_pt =  feats_undistort->points[i];
                if(tmp_pt.normal_z == 3)
                {
                    tmp_pt.intensity = tmp_pt.normal_y;
                    feats_undistort_ip->points.push_back(tmp_pt);

                }
                else if(tmp_pt.normal_z == 2)
                {
                    tmp_pt.intensity = tmp_pt.normal_y;
                    feats_undistort_p->points.push_back(tmp_pt);
                }
                else if(tmp_pt.normal_z == 1)
                {
                    tmp_pt.intensity = tmp_pt.normal_y;
                    feats_undistort_s->points.push_back(tmp_pt);

                }
            }
            feats_down_body_s.reset(new PointCloudXYZI());
            downSizeFilterSurf.setInputCloud(feats_undistort_s);
            downSizeFilterSurf.filter(*feats_down_body_s);  
            feats_down_size_s = feats_down_body_s->points.size(); 
            for(int i= 0; i < feats_down_size_s; i++)
            {
                feats_down_body_s->points[i].normal_z = 1;
                
                plane_range_ave += feats_down_body_s->points[i].intensity;
            }
            plane_range_ave /= feats_down_size_s;
            for(int i= 0; i < feats_down_size_s; i++)
            {
                delta_range_ave += ( feats_down_body_s->points[i].intensity -  plane_range_ave ) * ( feats_down_body_s->points[i].intensity -  plane_range_ave );
            }
            delta_range_ave = sqrt(delta_range_ave/( feats_down_size_s - 1 ));
            double plane_cov = 1/(1+exp(-delta_range_ave));
            feats_down_body_p.reset(new PointCloudXYZI());
            downSizeFilterPlane.setInputCloud(feats_undistort_p);//获得去畸变后的点云数据
            downSizeFilterPlane.filter(*feats_down_body_p);  //滤波降采样后的点云数据
            feats_down_size_p = feats_down_body_p->points.size(); //记录滤波后的点云数量
            for(int i= 0; i < feats_down_size_p; i++)
            {
                feats_down_body_p->points[i].normal_z = 2;
            }

            feats_down_body_ip.reset(new PointCloudXYZI());
            downSizeFilterIntenPlane.setInputCloud(feats_undistort_ip);//获得去畸变后的点云数据
            downSizeFilterIntenPlane.filter(*feats_down_body_ip);  //滤波降采样后的点云数据
            feats_down_size_ip = feats_down_body_ip->points.size(); //记录滤波后的点云数量
            for(int i= 0; i < feats_down_size_ip; i++)
            {
                feats_down_body_ip->points[i].normal_z = 3;
            }

            *feats_down_body = * feats_down_body_s;
            *feats_down_body += * feats_down_body_p;
            *feats_down_body += * feats_down_body_ip;

            if(is_intensity_edge_windows)
            {
                if(is_reset_window_point)
                {
                    window_point_size_ip = feats_down_body_ip_window->points.size();
                    for(int i= 0; i < window_point_size_ip; i++)
                    {
                        feats_down_body_ip_window->points[i].normal_z = 4;
                    }
                    *feats_down_body += * feats_down_body_ip_window;
                }
            }

            feats_down_size = feats_down_body->points.size(); 


            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            if(ikdtree.Root_Node == nullptr)
            {
                ikdtree.set_downsample_param(filter_size_map_min);
                feats_down_world->resize(feats_down_size);
                for(int i = 0; i < feats_down_size; i++)
                {
                    pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));  
                }
                ikdtree.Build(feats_down_world->points);     
                continue;
            }
            
            if(1) 
            {             
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            /*** iterated state estimation ***/
            Nearest_Points.resize(feats_down_size);    
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, feats_down_body, ikdtree, Nearest_Points, NUM_MAX_ITERATIONS, extrinsic_est_en , edge_plane_flag, plane_cov ,is_our_weight);
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot.matrix() * state_point.offset_T_L_I;

            if(is_intensity_edge_windows)
            {
                if(!is_reset_window_point)
                {
                    if( feats_down_body_ip_window_num > 2 )
                    {
                        feats_down_body_ip_window.reset(new PointCloudXYZI());
                        feats_down_body_ip_window_num = 0;
                    }
                    feats_down_body_ip_tmp->resize(feats_down_size_ip);
                    for(int i = 0; i < feats_down_size_ip; i++)
                    {
                        pointBodyToWorld(&(feats_down_body_ip->points[i]), &(feats_down_body_ip_tmp->points[i])); 
                    }
                    std::cout << "feats_down_body_ip_tmp->points.size() = " <<  feats_down_body_ip_tmp->points.size() <<  std::endl;
                    *feats_down_body_ip_window += *feats_down_body_ip_tmp;
                    feats_down_body_ip_window_num++;
                    std::cout << "feats_down_body_ip_window->points.size() = " <<  feats_down_body_ip_window->points.size() <<  std::endl;
                    if(feats_down_body_ip_window_num >= 2 )
                    {
                        is_reset_window_point = true;
                    }
                }
                else
                {
                    feats_down_body_ip_window.reset(new PointCloudXYZI());
                    feats_down_body_ip_window_num = 0;
                    is_reset_window_point = false;
                }
            }




            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);

            /*** add the feature points to map kdtree ***/
            feats_down_world->resize(feats_down_size);
            map_incremental();
            
            double t11 = omp_get_wtime();
            double zong_time = t11 - t00+feature_time_cur;

            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en) 
            {
                publish_frame_body(pubLaserCloudFull_body);
                publish_frame_body_intensity_gradient(pub_pl_intensity_gradient);
                publish_frame_body_geo_plane(pub_pl_geo_plane);
                publish_frame_body_geo_edge(pub_pl_geo_edge);
            }


            pubImage(pub_image_intensity, p_pre->image_intensity, p_pre->image_header, "bgr8");
            pubImage(pub_image_range, p_pre->image_range, p_pre->image_header, "bgr8");
            pubImage(pub_image_intensity_gradient, p_pre->image_intensity_gradient, p_pre->image_header, "bgr8");
             publish_map(pubLaserCloudMap);


        }
        rate.sleep();
    }

    /**************** save map ****************/
    if ( pcd_save_en)
    {
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        for (size_t i = 1; i <= pcd_index; i++)
        {
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZINormal>);
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(i) + string(".pcd"));
            pcl::PCDReader reader;
            reader.read(all_points_dir,*cloud_temp);
            *cloud = *cloud + *cloud_temp;
        }
    
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    return 0;
}
