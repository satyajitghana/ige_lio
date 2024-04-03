#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include <cassert>

#include <ros/package.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/filesystem.hpp>

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum LID_TYPE
{
  AVIA = 1,
  VELO16,
  OUST64,
  RS32,
  HAP
}; 



enum TIME_UNIT
{
  SEC = 0,
  MS = 1,
  US = 2,
  NS = 3
};

enum Feature
{
  Nor,    
  Poss_Plane,
  Real_Plane, 
  Edge_Jump, 
  Edge_Plane,
  Wire,  
  ZeroPoint  
};


enum Surround
{
  Prev, 
  Next  
};

enum E_jump
{
  Nr_nor,  // 
  Nr_zero, // 
  Nr_180,  // 
  Nr_inf,  // 
  Nr_blind //
};

struct orgtype
{
  double range; // 
  double dista; //
  double angle[2];  //
  double intersect; // 这
  E_jump edj[2];    // 
  Feature ftype;    // 

  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;//
    intersect = 2;
  }
};
namespace velodyne_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;// 
    float intensity;// 
    float time;// 
    float sec;
    float nsec;// 
    uint16_t ring; // 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW// 
  };
} 

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(float, sec, sec)(float, nsec, nsec)(std::uint16_t, ring, ring))

namespace rslidar_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring = 0;
    double timestamp = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} 
POINT_CLOUD_REGISTER_POINT_STRUCT(rslidar_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))

namespace ouster_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;      
    float intensity;  // 
    uint32_t t; // 
    uint16_t reflectivity;// 
    uint8_t ring;    
    uint16_t ambient;         // 
    uint32_t range;      // 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW// 
  };
} // namespace ouster_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

class Preprocess
{
  public:

  Preprocess();// 
  ~Preprocess();// 
  
  void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out); // 
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);// 
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);
  PointCloudXYZI pl_full, pl_corn, pl_surf; // 
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  vector<orgtype> typess[128]; //maximum 128 line lidar
  float time_unit_scale;//
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit;// 
  double blind; // 
  bool feature_enabled, given_offset_time; // 
  ros::Publisher pub_full, pub_surf, pub_corn; // 
  double image_gradient_threshold; // 

  // TODO czy    
  cv::Mat image_visualization;
    cv::Mat image_range;
  //cv::Mat image_noise;
  cv::Mat image_intensity;// 
  cv::Mat image_intensity_gradient;//
  std_msgs::Header image_header;
  PointCloudXYZI pl_intensity_gradient;
  int rs_image_to_pl[32*1800]; // 
  int rs_image_flag[32*1800]; // 
  int livoxmid_image_to_pl[30*5*1800]; //
  int livoxmid_image_flag[30*5*1800]; // 
  vector<int> index_ig;

  float intensity_dient_min; // 
  float intensity_dient_max; //
  float fov_degree; //
  float vertical_degree; //
  float vertical_degree_min; //
  float vertical_degree_max; //
  float delete_fov_degree; //
  float range_blind; // 
  float current_range_max; //
  float current_range_max_livox_hap; //
  float image_fov_resolution; //
  int image_vertical_rate; //
  int lookfor_min_num; // 
  int lookfor_max_num; //
  float intensity_min_range;

  //time
  double avia_delt_t;
  std::ofstream feature_time;


  private:
  void rs_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);// 
  void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);// 
  void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);// 
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);// 
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);// 
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);
  bool CreateFile(std::ofstream& ofs, std::string file_path);
  bool CreateDirectory(std::string directory_path);


  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;

};
