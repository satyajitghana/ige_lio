#include "preprocess.h"

#define RETURN0 0x00
#define RETURN0AND1 0x10

const bool time_list1(rslidar_ros::Point &x, rslidar_ros::Point &y) {return (x.timestamp < y.timestamp);};

Preprocess::Preprocess()
    : feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1)
{
  inf_bound = 10;   // 
  N_SCANS = 6;      //
  SCAN_RATE = 10;//
  group_size = 8;// 
  disA = 0.01;// ,
  disB = 0.1; // // ,
  p2l_ratio = 225; // 
  limit_maxmid = 6.25;// 
  limit_midmin = 6.25;// 
  limit_maxmin = 3.24; // 
  jump_up_limit = 170.0;
  jump_down_limit = 8.0;
  cos160 = 160.0;
  edgea = 2; //
  edgeb = 0.1;//.
  smallp_intersect = 172.5;
  smallp_ratio = 1.2;   //..
  given_offset_time = false; //


  jump_up_limit = cos(jump_up_limit / 180 * M_PI);   //
  jump_down_limit = cos(jump_down_limit / 180 * M_PI);  //
  cos160 = cos(cos160 / 180 * M_PI); //夹角限制
  smallp_intersect = cos(smallp_intersect / 180 * M_PI); //..
}

Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;//
  lidar_type = lid_type;     //
  blind = bld;           //
  point_filter_num = pfilt_num;//
}

/**
 * @brief 
 *
 * @param msg ::CustomMsg
 * @param pcl_out ::PointCloud<pcl::PointXYZINormal>
 */
void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  avia_handler(msg);
  *pcl_out = pl_surf;
}

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  switch (time_unit)//wzl
  {
  case SEC:
    time_unit_scale = 1.e3f;
    break;
  case MS:
    time_unit_scale = 1.f;
    break;
  case US:
    time_unit_scale = 1.e-3f;
    break;
  case NS:
    time_unit_scale = 1.e-6f;
    break;
  default:
    time_unit_scale = 1.f;
    break;
  }

  switch (lidar_type)
  {
  case OUST64:
    oust64_handler(msg);
    break;

  case VELO16:
    velodyne_handler(msg);
    break;

  case RS32://wzl
    rs_handler(msg);
    break;

  default:
    printf("Error LiDAR Type");
    break;
  }
  *pcl_out = pl_surf;
}

bool Preprocess::CreateFile(std::ofstream& ofs, std::string file_path) {
    ofs.open(file_path.c_str(), std::ios::app);
    if (!ofs) {
       std::cout << "无法生成文件: " << file_path << std::endl;
        return false;
    }

    return true;
}

bool Preprocess::CreateDirectory(std::string directory_path) {
    if (!boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::create_directory(directory_path);
    }
    if (!boost::filesystem::is_directory(directory_path)) {
        std::cout << "无法建立文件夹: " << directory_path << std::endl;
        return false;
    }
    return true;
}




void Preprocess::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    // 
  pl_surf.clear();  // 
  pl_corn.clear();  // 
  pl_full.clear();      // 

  pl_intensity_gradient.clear();
  vector<int>().swap(index_ig);



  int plsize = msg->point_num;// 


  pl_corn.reserve(plsize);// // 
  pl_surf.reserve(plsize);// 
  pl_full.resize(plsize); // 

// TODO czy
  // std::cout  <<  pl_orig.size() << std::endl;
  //reset images
  int image_u = int(vertical_degree) * image_vertical_rate ;
  int image_v = fov_degree/image_fov_resolution;
  image_range = cv::Mat(image_u, image_v, CV_8UC1, cv::Scalar(0));// 
  //image_noise = cv::Mat(N_SCANS, 1800, CV_8UC1, cv::Scalar(0));
  image_intensity = cv::Mat(image_u, image_v, CV_8UC1, cv::Scalar(0));
  image_intensity_gradient = cv::Mat(image_u, image_v, CV_8UC1, cv::Scalar(0));

  for (int i = 0; i < N_SCANS; i++)//
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);// 
  }
  uint valid_num = 0;// 


  if (feature_enabled)
  {
    std::cout << "进行特征提取" << std::endl;
      double avia_t0 = omp_get_wtime();
      // 
    double y_angle_max = 0;//
    double y_angle_min = 0;//
    double yaw_angle_max = 0;//
    double yaw_angle_min = 0;//
    double intensity_min = 255;//
    double intensity_max = 0;//
    int invaild_pl = 0;
    double intensity_dient_tmp = 0;
    int current_point_line = 0; // 
    int current_point_line_size = 0;//
    for (uint i = 1; i < plsize; i++)
    {
      int layer = 0;

      if ((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        pl_full[i].x = msg->points[i].x;       // 
        pl_full[i].y = msg->points[i].y;              // 
        pl_full[i].z = msg->points[i].z;        // 
        pl_full[i].intensity = msg->points[i].reflectivity;          // 
        pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points  // 使用曲率作为每个激光点的时间（好像是相对于当前帧第一个点的相对时间）
        // std::cout << "pl_full[ "<< i  << "].curvature = " << pl_full[i].curvature << std::endl;
         //std::cout << "msg->points[i].line " << int(msg->points[i].line) << std::endl;

        double yaw_angle = atan2(msg->points[i].y, msg->points[i].x) * 57.2957 + fov_degree/2;
        double y_angle = atan2(msg->points[i].z, sqrt( msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y )) * 57.2957;
        float current_range = sqrt(  msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y + msg->points[i].z * msg->points[i].z  );
        float current_intensity = std::min( pl_full[i].intensity ,  255.0f);
        pl_full[i].normal_y = current_range;// 
        
        if( current_intensity < 5)
        {
          current_intensity = 0;
        }
        
        
        if( current_intensity < intensity_min )
        {
          intensity_min = current_intensity;
        }
        if(current_intensity > intensity_max)
        {
          intensity_max = current_intensity;
        }

        
        if( y_angle < y_angle_min )
        {
          y_angle_min = y_angle;
        }
        if( y_angle > y_angle_max )
        {
          y_angle_max = y_angle;
        }
        if( yaw_angle-fov_degree/2 < yaw_angle_min )
        {
          yaw_angle_min = yaw_angle-fov_degree/2;
        }
        if( yaw_angle-fov_degree/2 > yaw_angle_max )
        {
          yaw_angle_max = yaw_angle-fov_degree/2;
        }
        

        layer = int(( y_angle + abs(vertical_degree_min) ) *image_vertical_rate) ;

        bool is_new = false;

        if( i % point_filter_num == 0)
        {
          if (((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) || (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7)) && current_range > range_blind && current_range <  current_range_max_livox_hap)
          {
            //std::cout <<  "pl_full[i].z = " << pl_full[i].z << std::endl;
            //std::cout <<  "layer = " << layer << std::endl;
            if(layer < image_u && layer >= 0 && yaw_angle >= delete_fov_degree && yaw_angle <= (fov_degree - delete_fov_degree) )
            {
              if ((current_intensity > 200  || current_intensity < 50 )&&  current_range < intensity_min_range)
                continue;
              int v  = yaw_angle/image_fov_resolution;
              int u = layer;
              if(u >0 && v > 5)
              {
                image_range.at<uint8_t>((image_u-1 - u) , image_v-1-v) = std::min(current_range * 20, 255.0f);
                image_intensity.at<uint8_t>((image_u-1 - u), image_v-1-v) = current_intensity*2;
                livoxmid_image_to_pl[u * image_v + v] = invaild_pl;
                pl_full[i].normal_x = u * image_v + v;
                current_point_line = msg->points[i].line;
                pl_buff[current_point_line].push_back(pl_full[i]);// 
                //
                int flag_min = 0;
                int flag_min_1 = 0;
                int flag_min_2 = 0;
                int flag_max = 0;
                int flag_max_1 = 0;
                int flag_max_2 = 0;
                bool flag_min_ok = false;
                bool flag_max_ok = false;
                float intensity_max_ave = 0; // 
                float intensity_min_ave = 0; // 
                float selected_between_range = 0 ; // 
                float intensity_edge_weight = 0; //
                //int lookfor_min_num = 2;
                //int lookfor_max_num = 1;
                if(current_intensity != 0)
                {
                  current_point_line_size = pl_buff[current_point_line].points.size();
                  intensity_max_ave = current_intensity;
                  if(lookfor_max_num != 0)
                  {
                    for(int i = 0; i < lookfor_max_num; i++)
                    {
                      if(current_point_line_size < lookfor_min_num+lookfor_max_num+1) break;
                      if(pl_buff[current_point_line].points[current_point_line_size-1-(i+1)].intensity == 0) continue;
                      intensity_dient_tmp =  current_intensity - pl_buff[current_point_line].points[current_point_line_size-1-(i+1)].intensity  ;
                    
                      intensity_max_ave += pl_buff[current_point_line].points[current_point_line_size-1-(i+1)].intensity;
                      float  intensity_dient_min_tmp =  min(intensity_dient_min ,  float(current_intensity * 0.15));
                      if(abs(intensity_dient_tmp) < intensity_dient_min)
                      {
                        if( intensity_dient_tmp <= 0 )
                          flag_max_1++;
                        else
                          flag_max_2++;
                        flag_max = max(flag_max_1, flag_max_2);
                        if(flag_max == lookfor_max_num)
                        {
                          flag_max_ok = true;

                          intensity_max_ave /= lookfor_max_num;
                        
                        }
                      }
                    }
                  }
                  else
                  {
                    flag_max_ok = true;
                  }

                  if(flag_max_ok == true)
                  {
                    for(int i = 0; i < lookfor_min_num; i++)
                    {
                      //if(current_point_line_size < lookfor_min_num) break;
                      if(pl_buff[current_point_line].points[current_point_line_size-1-lookfor_max_num-(i+1)].intensity == 0) continue;

                      intensity_dient_tmp =  pl_buff[current_point_line].points[current_point_line_size-1-lookfor_max_num].intensity - pl_buff[current_point_line].points[current_point_line_size-1-lookfor_max_num-(i+1)].intensity ;
                      
                      intensity_min_ave += pl_buff[current_point_line].points[current_point_line_size-1-lookfor_max_num-(i+1)].intensity;

                      float  intensity_dient_max_tmp =  max(intensity_dient_max ,  float(current_intensity * 0.20));
                      if(abs(intensity_dient_tmp) > intensity_dient_max_tmp)
                      {
                        if(intensity_dient_tmp<=0)
                          flag_min_1++;
                        else
                          flag_min_2++;

                        flag_min = max(flag_min_1,flag_min_2);
                        if(flag_min  ==  lookfor_min_num)
                        {

                          intensity_min_ave /= lookfor_min_num;
                          float selected_between_range_x = pl_buff[current_point_line].points[current_point_line_size-1-lookfor_max_num].x - pl_buff[current_point_line].points[current_point_line_size-1-lookfor_max_num-1].x;
                          float selected_between_range_y = pl_buff[current_point_line].points[current_point_line_size-1-lookfor_max_num].y - pl_buff[current_point_line].points[current_point_line_size-1-lookfor_max_num-1].y;
                          float selected_between_range_z = pl_buff[current_point_line].points[current_point_line_size-1-lookfor_max_num].z - pl_buff[current_point_line].points[current_point_line_size-1-lookfor_max_num-1].z;
                          selected_between_range = sqrt( selected_between_range_x * selected_between_range_x + selected_between_range_y* selected_between_range_y + selected_between_range_z*selected_between_range_z);
                          
                          //std::cout << "(intensity_max_ave - intensity_min_ave) / 255 = " << (intensity_max_ave - intensity_min_ave) / 255 << std::endl;
                          intensity_edge_weight =1+(exp(-abs( (intensity_max_ave - intensity_min_ave) / 255 ) ));
                          //std::cout << "intensity_edge_weight = " << intensity_edge_weight << std::endl;
                          intensity_edge_weight = intensity_edge_weight * ( 1/(1+exp(-selected_between_range)) );
                          //std::cout << "intensity_edge_weight = " << intensity_edge_weight << std::endl;
                          pl_buff[current_point_line].points[current_point_line_size-1-lookfor_max_num].normal_y = intensity_edge_weight; // 不能少

                          image_intensity_gradient.at<uint8_t>((image_u-1 - u), image_v-1-(v-lookfor_max_num)) = abs(pl_buff[current_point_line].points[current_point_line_size-1-lookfor_max_num].intensity - pl_buff[current_point_line].points[current_point_line_size-1-lookfor_max_num-1].intensity) * 2;

                          pl_buff[current_point_line].points[current_point_line_size-1-lookfor_max_num].normal_z = 3; // 不能少

                          PointType point_tmp;
                          point_tmp.x = (pl_buff[current_point_line].points[current_point_line_size-1-lookfor_max_num].x + pl_buff[current_point_line].points[current_point_line_size-1-lookfor_max_num-1].x)/2;
                          point_tmp.y = (pl_buff[current_point_line].points[current_point_line_size-1-lookfor_max_num].y + pl_buff[current_point_line].points[current_point_line_size-1-lookfor_max_num-1].y)/2;
                          point_tmp.z = (pl_buff[current_point_line].points[current_point_line_size-1-lookfor_max_num].z + pl_buff[current_point_line].points[current_point_line_size-1-lookfor_max_num-1].z)/2;
                          point_tmp.intensity = pl_buff[current_point_line].points[current_point_line_size-1-lookfor_max_num].intensity;
                          point_tmp.curvature = pl_buff[current_point_line].points[current_point_line_size-1-lookfor_max_num].curvature;
                          point_tmp.normal_y = intensity_edge_weight;//normal_x
                          point_tmp.normal_z = 3;

                          pl_intensity_gradient.points.push_back(point_tmp);

                        }
                      }
                    }
                  }
                }
                invaild_pl++;
              }
            }
          }
        }
      }
    }
    
    std::cout << "intensity_min = " << intensity_min  <<  std::endl;
    std::cout << "intensity_max = " << intensity_max  <<  std::endl;
    std::cout << "y_angle_min = " << y_angle_min  <<  std::endl;
    std::cout << "y_angle_max = " << y_angle_max  <<  std::endl;
    std::cout << "yaw_angle_min = " << yaw_angle_min  <<  std::endl;
    std::cout << "yaw_angle_max = " << yaw_angle_max  <<  std::endl;
    
    static int avia_count = 0;
    static double avia_time = 0.0;
    avia_count++;

    //   
    for (int j = 0; j < N_SCANS; j++)
    {

      if (pl_buff[j].size() <= 5)
        continue;
      pcl::PointCloud<PointType> &pl = pl_buff[j];
      plsize = pl.size();
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(plsize);
      plsize--;
      for (uint i = 0; i < plsize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);  // 
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = sqrt(vx * vx + vy * vy + vz * vz);
      }

      types[plsize].range = sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);
      give_feature(pl, types); //
      // pl_surf += pl;
    }

    std::cout <<  " pl_surf = " << pl_surf.points.size() << std::endl;
    std::cout <<  " pl_corn = " << pl_corn.points.size() << std::endl;
    pl_surf += pl_corn;
    //std::cout <<  " pl_surf + pl_corn = " << pl_surf.points.size() << std::endl;
    std::cout <<  " pl_intensity_gradient = " << pl_intensity_gradient.points.size() << std::endl;
    pl_surf += pl_intensity_gradient;
    //std::cout <<  " pl_surf + pl_corn +pl_intensity_gradient = " << pl_surf.points.size() << std::endl;

    double avia_t1 = omp_get_wtime();
    avia_delt_t = avia_t1 - avia_t0;
    avia_time += avia_delt_t;
    printf("avia Feature extraction time: %lf \n", avia_time / avia_count);

    static bool is_file_created = false;
    std::string WORK_SPACE_PATH = "/home/inkers/satyajit/catkin_ige_lio";
    if(1)
    {
      if (!is_file_created) 
      {
        if (!CreateDirectory(WORK_SPACE_PATH + "/slam_data/trajectory"))
          {
            is_file_created = false;
            printf("avia Feature extraction : NO feature_time Directory");
          }
        if (!CreateFile(feature_time, WORK_SPACE_PATH + "/slam_data/trajectory/feature_time.txt"))
          {
            is_file_created = false;
            printf("avia Feature extraction : NO feature_time.txt");
          }
        is_file_created = true;
      }

      if(is_file_created)
      {
        feature_time << avia_delt_t;
        feature_time << std::endl;
      }
    }

  }
  else
  {
    // 
    for (uint i = 1; i < plsize; i++)
    {

      if ((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))
      {
        valid_num++;// 

        // 
        if (valid_num % point_filter_num == 0)
        {
          pl_full[i].x = msg->points[i].x;// 
          pl_full[i].y = msg->points[i].y;// 
          pl_full[i].z = msg->points[i].z; // 
          pl_full[i].intensity = msg->points[i].reflectivity;// 
          pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points, curvature unit: ms
          pl_full[i].normal_z = 1;


          if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) || (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7) && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z > (blind * blind)))
          {
            //std::cout << "pl_full[i].curvature: " << pl_full[i].curvature << std::endl;
            pl_surf.push_back(pl_full[i]);
          }
        }
      }
    }
    //std::cout << "pl_surf.size(): " << pl_surf.size() << std::endl;
  }

   if (1)
  {
    cv::cvtColor(image_intensity, image_intensity, CV_GRAY2RGB);
    cv::cvtColor(image_range, image_range, CV_GRAY2RGB);
    cv::cvtColor(image_intensity_gradient, image_intensity_gradient, CV_GRAY2RGB);
    cv::putText(image_intensity, "Intensity", cv::Point2f(5, 20 ), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
    cv::putText(image_range, "Range",     cv::Point2f(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
    cv::putText(image_intensity_gradient, "Intensity_Gradient",     cv::Point2f(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
    image_header = msg->header;
  }
}



void Preprocess::oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  
  //TODO czy
  pl_intensity_gradient.clear();
  vector<int>().swap(index_ig);

  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.reserve(plsize);


  /*** These variables only works when no point timestamps given ***/
  double omega_l = 0.361 * SCAN_RATE; // scan angular velocity//   
  std::vector<bool> is_first(N_SCANS, true);  //  N_SCANS = 32
  std::vector<double> yaw_fp(N_SCANS, 0.0);   // yaw of first scan point
  std::vector<float> yaw_last(N_SCANS, 0.0);  // yaw of last scan point
  std::vector<float> time_last(N_SCANS, 0.0); // last offset time
  /*****************************************************************/

  int image_beilv = image_vertical_rate;
  int image_u = N_SCANS  * image_beilv;
  int image_v = fov_degree/image_fov_resolution;

// TODO czy
  assert((int)pl_orig.size() % N_SCANS * image_v == 0);
  // std::cout  <<  pl_orig.size() << std::endl ;
  //reset images

  image_range = cv::Mat(image_u, image_v, CV_8UC1, cv::Scalar(0));// 
  //image_noise = cv::Mat(N_SCANS, 1800, CV_8UC1, cv::Scalar(0));
  image_intensity = cv::Mat(image_u, image_v, CV_8UC1, cv::Scalar(0));
  image_intensity_gradient = cv::Mat(image_u, image_v, CV_8UC1, cv::Scalar(0));

  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    int invaild_pl = 0;

    for (uint i = 0; i < plsize; i++)
    {
      float current_range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      if (current_range < (blind * blind))
        continue;
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
      if (yaw_angle >= 180.0)
        yaw_angle -= 360.0;
      if (yaw_angle <= -180.0)
        yaw_angle += 360.0;
      //std::cout << "pl_orig.points[i].t = " << pl_orig.points[i].t << std::endl;
      added_pt.curvature = pl_orig.points[i].t * time_unit_scale;
      //std::cout << "added_pt.curvature = " << added_pt.curvature << std::endl;
      int layer = (atan2(sqrt(added_pt.y*added_pt.y+added_pt.x*added_pt.x), added_pt.z)* 57.3-80)/2;
      //std::cout << "ring = " << ring << std::endl;
      if (layer < N_SCANS)
      {
        pl_buff[layer].push_back(added_pt);
      }

    float current_intensity = std::min(added_pt.intensity, 255.0f);
    yaw_angle = yaw_angle + fov_degree/2; //  
    pl_full[i].normal_y = current_range;// 
    if( current_intensity < 5)
    {
      current_intensity = 0;
    }
    
        if (i % 1 == 0) // 
    {
      if( current_range >= blind )//&& current_range <= 30)
      {
        if (layer < N_SCANS && layer >= 0)
        {
          int v = yaw_angle/0.2;
          int u = layer;
          if(u >0 && v > 5)
          {
            image_range.at<uint8_t>((N_SCANS-1 - u) * image_beilv, image_v-1-v) = std::min(current_range * 20, 255.0f);
            image_intensity.at<uint8_t>((N_SCANS-1 - u)*image_beilv, image_v-1-v) = current_intensity;
            rs_image_to_pl[u * image_v + v] = invaild_pl;
            rs_image_flag[u * image_v + v] = 0;
            added_pt.normal_x = u * image_v + v;
            pl_full.points.push_back(added_pt);
            if(i % point_filter_num == 0)// 
                pl_buff[layer].points.push_back(added_pt);
            invaild_pl++;
          }

        }
      }

    }
    
    }

    for (int j = 0; j < N_SCANS; j++)
    {

      PointCloudXYZI &pl = pl_buff[j];
      int linesize = pl.size();
      if( linesize<=0 )
      {
        continue;
      }
      //std::cout <<  j <<"  linesize = "  << linesize << std::endl;
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }


  //std::cout <<  " pl_surf = " << pl_surf.points.size() << std::endl;
  //std::cout <<  " pl_corn = " << pl_corn.points.size() << std::endl;
  pl_surf += pl_corn;
  //std::cout <<  " pl_surf + pl_corn = " << pl_surf.points.size() << std::endl;

  float i_intensity;
  float i_1_intensity;
  float i_2_intensity;
  int intensity_tmp = 0;
  float i_intensity_gradient;
  float tmp1,tmp2;

  int flag_min = 0;
  int flag_min_1 = 0;
  int flag_min_2 = 0;
  int flag_max = 0;
  int flag_max_1 = 0;
  int flag_max_2 = 0;
  bool flag_min_ok = false;
  bool flag_max_ok = false;

  float intensity_max_ave = 0; // 
  float intensity_max_ave_tmp1 = 0;
  float intensity_max_ave_tmp2 = 0;

  float intensity_min_ave = 0; // 
  float intensity_min_ave_tmp1 = 0;
  float intensity_min_ave_tmp2 = 0;
  
  float selected_between_range = 0 ; // 
  float intensity_edge_weight = 0; //
  int neigher_point  = 0; // 

  bool flag_shuiping = false;
  bool flag_chuizhi = false;


    for (int u = 0; u < N_SCANS * image_beilv; u++) 
  {
    if(u%image_beilv != 0) continue;

    for (int v = 0; v < image_v; v++) 
    {
      //rs_image_flag[(u/image_beilv)*image_v+v] = 0;

      i_intensity_gradient = 0;
      flag_max_ok = false;
      flag_min_ok = false;
      flag_shuiping = false;
      flag_chuizhi = false;
      flag_min_1 = 0;
      flag_min_2 = 0;
      flag_max_1 = 0;
      flag_max_2 = 0;
      intensity_max_ave_tmp1 = 0;
      intensity_max_ave_tmp2 = 0;
      intensity_min_ave_tmp1 = 0;
      intensity_min_ave_tmp2 = 0;
      
      if( u < 3 * image_beilv)
      {
        if ( v <= 5 ) 
          continue;

        i_intensity = image_intensity.at<uint8_t>(u, v); // 
        float  intensity_dient_min_tmp =  min(intensity_dient_min ,  float(i_intensity * 0.15));
        float  intensity_dient_max_tmp =  max(intensity_dient_max ,  float(i_intensity * 0.20));
        if (i_intensity == 0) //  0 
          continue;
        intensity_max_ave = i_intensity;
        intensity_min_ave = 0;

        for(int i = 1; i < 2; i ++)
        {
          if(rs_image_flag[(u/image_beilv)*image_v+v+i] != 1)
          {
            i_1_intensity = image_intensity.at<uint8_t>(u, v + i); // 
            if (i_1_intensity != 0)
            {
              i_intensity_gradient = i_1_intensity - i_intensity;
              if(abs(i_intensity_gradient) <  intensity_dient_min_tmp )
              {
                //intensity_max_ave += i_1_intensity;

                if( i_intensity_gradient <= 0 )
                {
                  flag_max_1++;
                  intensity_max_ave_tmp1 += i_1_intensity;
                }
                else 
                {
                  flag_max_2++;
                  intensity_max_ave_tmp2 += i_1_intensity;
                }

                 if( flag_max_1 >  flag_max_2)
                 {
                  flag_max = flag_max_1;
                  intensity_max_ave += intensity_max_ave_tmp1;
                 }
                 else
                 {
                  flag_max = flag_max_2;
                  intensity_max_ave += intensity_max_ave_tmp2;
                 }
                //flag_max = max(flag_max_1, flag_max_2);

                if( flag_max >= lookfor_max_num )
                {
                  flag_max_ok = true;

                  intensity_max_ave /= flag_max;
                }

              }
              //rs_image_flag[(u/image_beilv)*1800+v] = 1;
              //rs_image_flag[(u/image_beilv)*1800+v-i] = 1;
            }
          }

          if(rs_image_flag[(u/image_beilv)*image_v+v-i] != 1)
          {
            i_1_intensity = image_intensity.at<uint8_t>(u, v - i); // 

            if (i_1_intensity != 0)
            {
              i_intensity_gradient = i_intensity - i_1_intensity  ;
              if(abs(i_intensity_gradient) >  intensity_dient_max_tmp )
              {
                if(neigher_point == 0 )
                  neigher_point = i;
                //intensity_min_ave += i_1_intensity;

                if( i_intensity_gradient <= 0 )
                {
                  flag_min_1++;
                  intensity_min_ave_tmp1 += i_1_intensity;
                }
                else 
                {
                  flag_min_2++;
                  intensity_min_ave_tmp2 += i_1_intensity;
                }

                 if( flag_min_1 >  flag_min_2)
                 {
                  flag_max = flag_min_1;
                  intensity_min_ave += intensity_min_ave_tmp1;
                 }
                 else
                 {
                  flag_min = flag_min_2;
                  intensity_min_ave += intensity_min_ave_tmp2;
                 }
                //flag_min = max(flag_min_1, flag_min_2);

                if( flag_min >= lookfor_min_num )
                {
                  flag_min_ok = true;

                  intensity_min_ave /= flag_min;
                }
              }
              //rs_image_flag[(u/image_beilv)*1800+v] = 1;
              //rs_image_flag[(u/image_beilv)*1800+v-i] = 1;
            }
          }
          else 
          {
            break;
          }
        }

        if(flag_max_ok == true && flag_min_ok == true)
        {
          int index = rs_image_to_pl[(N_SCANS-1  - (u/image_beilv))*image_v+(image_v-1 - v)];
          int index_neigher = rs_image_to_pl[(N_SCANS-1  - (u/image_beilv))*image_v+(image_v-1 - (v - neigher_point ))];
          float selected_between_range_x = pl_full.points[index].x - pl_full.points[index_neigher].x;
          float selected_between_range_y = pl_full.points[index].y - pl_full.points[index_neigher].y;
          float selected_between_range_z = pl_full.points[index].z - pl_full.points[index_neigher].z;
          selected_between_range = sqrt( selected_between_range_x * selected_between_range_x + selected_between_range_y* selected_between_range_y + selected_between_range_z*selected_between_range_z);

          //std::cout << "(intensity_max_ave - intensity_min_ave) / 255 = " << (intensity_max_ave - intensity_min_ave) / 255 << std::endl;
          intensity_edge_weight =1+(exp(-abs( (intensity_max_ave - intensity_min_ave) / 255 ) ));
          //std::cout << "intensity_edge_weight = " << intensity_edge_weight << std::endl;
          intensity_edge_weight = intensity_edge_weight * ( 1/(1+exp(-selected_between_range)) );
          //std::cout << "intensity_edge_weight = " << intensity_edge_weight << std::endl;
          //pl_full.points[index].normal_y = intensity_edge_weight;
          i_intensity_gradient = abs( pl_full.points[index].intensity - pl_full.points[index_neigher].intensity );

          if (i_intensity_gradient > 25)
          {
            image_intensity_gradient.at<uint8_t>(u , v) = i_intensity_gradient * 5;

            PointType point_tmp;
            point_tmp.x = (pl_full.points[index].x + pl_full.points[index_neigher].x)/2;
            point_tmp.y = (pl_full.points[index].y + pl_full.points[index_neigher].y)/2;
            point_tmp.z = (pl_full.points[index].z + pl_full.points[index_neigher].z)/2;
            point_tmp.intensity = pl_full.points[index].intensity;
            point_tmp.curvature = pl_full.points[index].curvature;
            point_tmp.normal_y = intensity_edge_weight;//normal_x
            point_tmp.normal_z = 3;

            pl_intensity_gradient.points.push_back(point_tmp);

            rs_image_flag[(u/image_beilv)*image_v + v] = 1;
            rs_image_flag[(u/image_beilv)*image_v + v - neigher_point ] = 1;
          }
        }
      }
      else
      {

        if ( v <= 5 ) 
          continue;

        i_intensity = image_intensity.at<uint8_t>(u, v); // 
        float  intensity_dient_min_tmp =  min(intensity_dient_min ,  float(i_intensity * 0.15));
        float  intensity_dient_max_tmp =  max(intensity_dient_max ,  float(i_intensity * 0.20));
        if (i_intensity == 0) //  
          continue;
        intensity_max_ave = i_intensity;
        for(int i = 1; i < 2; i ++)
        {
          if(rs_image_flag[(u/image_beilv)*image_v+v+i] != 1)
          {
            i_1_intensity = image_intensity.at<uint8_t>(u, v + i); // 
            if (i_1_intensity != 0)
            {
              i_intensity_gradient = i_1_intensity - i_intensity;
              if(abs(i_intensity_gradient) <  intensity_dient_min_tmp )
              {
                //intensity_max_ave += i_1_intensity;

                if( i_intensity_gradient <= 0 )
                {
                  flag_max_1++;
                  intensity_max_ave_tmp1 += i_1_intensity;
                }
                else 
                {
                  flag_max_2++;
                  intensity_max_ave_tmp2 += i_1_intensity;
                }

                 if( flag_max_1 >  flag_max_2)
                 {
                  flag_max = flag_max_1;
                  intensity_max_ave += intensity_max_ave_tmp1;
                 }
                 else
                 {
                  flag_max = flag_max_2;
                  intensity_max_ave += intensity_max_ave_tmp2;
                 }
                //flag_max = max(flag_max_1, flag_max_2);

                if( flag_max >= lookfor_max_num )
                {
                  flag_max_ok = true;

                  intensity_max_ave /= flag_max;
                }

              }
              //rs_image_flag[(u/image_beilv)*1800+v] = 1;
              //rs_image_flag[(u/image_beilv)*1800+v-i] = 1;
            }
          }

          if(rs_image_flag[(u/image_beilv)*image_v+v-i] != 1)
          {
            i_1_intensity = image_intensity.at<uint8_t>(u, v - i); // 取出上一个同一scan的点的强度

            if (i_1_intensity != 0)
            {
              i_intensity_gradient = i_intensity - i_1_intensity  ;
              if(abs(i_intensity_gradient) >  intensity_dient_max_tmp )
              {
                if(neigher_point == 0 )
                  neigher_point = i;
                //intensity_min_ave += i_1_intensity;

                if( i_intensity_gradient <= 0 )
                {
                  flag_min_1++;
                  intensity_min_ave_tmp1 += i_1_intensity;
                }
                else 
                {
                  flag_min_2++;
                  intensity_min_ave_tmp2 += i_1_intensity;
                }

                 if( flag_min_1 >  flag_min_2)
                 {
                  flag_max = flag_min_1;
                  intensity_min_ave += intensity_min_ave_tmp1;
                 }
                 else
                 {
                  flag_min = flag_min_2;
                  intensity_min_ave += intensity_min_ave_tmp2;
                 }
                //flag_min = max(flag_min_1, flag_min_2);

                if( flag_min >= lookfor_min_num )
                {
                  flag_min_ok = true;

                  intensity_min_ave /= flag_min;
                }
              }
              //rs_image_flag[(u/image_beilv)*1800+v] = 1;
              //rs_image_flag[(u/image_beilv)*1800+v-i] = 1;
            }
          }
          else 
          {
            break;
          }
        }

        if(flag_max_ok == true && flag_min_ok == true)
        {
          int index = rs_image_to_pl[(N_SCANS-1  - (u/image_beilv))*image_v+(image_v-1 - v)];
          int index_neigher = rs_image_to_pl[(N_SCANS-1  - (u/image_beilv))*image_v+(image_v-1 - (v - neigher_point ))];
          float selected_between_range_x = pl_full.points[index].x - pl_full.points[index_neigher].x;
          float selected_between_range_y = pl_full.points[index].y - pl_full.points[index_neigher].y;
          float selected_between_range_z = pl_full.points[index].z - pl_full.points[index_neigher].z;
          selected_between_range = sqrt( selected_between_range_x * selected_between_range_x + selected_between_range_y* selected_between_range_y + selected_between_range_z*selected_between_range_z);

          //std::cout << "(intensity_max_ave - intensity_min_ave) / 255 = " << (intensity_max_ave - intensity_min_ave) / 255 << std::endl;
          intensity_edge_weight =1+(exp(-abs( (intensity_max_ave - intensity_min_ave) / 255 ) ));
          //std::cout << "intensity_edge_weight = " << intensity_edge_weight << std::endl;
          intensity_edge_weight = intensity_edge_weight * ( 1/(1+exp(-selected_between_range)) );
          //std::cout << "intensity_edge_weight = " << intensity_edge_weight << std::endl;
          //pl_full.points[index].normal_y = intensity_edge_weight;

          i_intensity_gradient = abs( pl_full.points[index].intensity - pl_full.points[index_neigher].intensity );

          if (i_intensity_gradient > 25)
          {
            image_intensity_gradient.at<uint8_t>(u , v) = i_intensity_gradient * 5;

            PointType point_tmp;
            point_tmp.x = (pl_full.points[index].x + pl_full.points[index_neigher].x)/2;
            point_tmp.y = (pl_full.points[index].y + pl_full.points[index_neigher].y)/2;
            point_tmp.z = (pl_full.points[index].z + pl_full.points[index_neigher].z)/2;
            point_tmp.intensity = pl_full.points[index].intensity;
            point_tmp.curvature = pl_full.points[index].curvature;
            point_tmp.normal_y = intensity_edge_weight;//normal_x
            point_tmp.normal_z = 3;

            pl_intensity_gradient.points.push_back(point_tmp);

            rs_image_flag[(u/image_beilv)*image_v + v] = 1;
            rs_image_flag[(u/image_beilv)*image_v + v - neigher_point ] = 1;
          }
        }
        else
        {
          for(int i = 1; i < 2; i ++)
          {
            if(rs_image_flag[((u/image_beilv)-i)*image_v + v - i] != 1) 
            {
              i_2_intensity = image_intensity.at<uint8_t>(u - image_beilv*i, v ); // 取出上一个同一scan的点的强度
              {
                if (i_2_intensity != 0)
                {
                  int index = rs_image_to_pl[(N_SCANS-1  - (u/image_beilv))*image_v+(image_v-1 - v)];
                  int index_neigher = rs_image_to_pl[(N_SCANS-1  - ((u/image_beilv) - i ))*image_v+(image_v-1 - (v ))];
                  float selected_between_range_x = pl_full.points[index].x - pl_full.points[index_neigher].x;
                  float selected_between_range_y = pl_full.points[index].y - pl_full.points[index_neigher].y;
                  float selected_between_range_z = pl_full.points[index].z - pl_full.points[index_neigher].z;
                  selected_between_range = sqrt( selected_between_range_x * selected_between_range_x + selected_between_range_y* selected_between_range_y + selected_between_range_z*selected_between_range_z);
                  if(selected_between_range > 0.5)
                    break;
                  //std::cout << "(intensity_max_ave - intensity_min_ave) / 255 = " << (intensity_max_ave - intensity_min_ave) / 255 << std::endl;
                  intensity_edge_weight =1+(exp(-abs( (pl_full.points[index].intensity - pl_full.points[index_neigher].intensity) / 255 ) ));
                  //std::cout << "intensity_edge_weight = " << intensity_edge_weight << std::endl;
                  intensity_edge_weight = intensity_edge_weight * ( 1/(1+exp(-selected_between_range)) );
                  //std::cout << "intensity_edge_weight = " << intensity_edge_weight << std::endl;
                  //pl_full.points[index].normal_y = intensity_edge_weight;

                  i_intensity_gradient = abs( pl_full.points[index].intensity - pl_full.points[index_neigher].intensity );

                  if (i_intensity_gradient > 25)
                  {
                    image_intensity_gradient.at<uint8_t>(u , v) = i_intensity_gradient * 5;

                    PointType point_tmp;
                    point_tmp.x = (pl_full.points[index].x + pl_full.points[index_neigher].x)/2;
                    point_tmp.y = (pl_full.points[index].y + pl_full.points[index_neigher].y)/2;
                    point_tmp.z = (pl_full.points[index].z + pl_full.points[index_neigher].z)/2;
                    point_tmp.intensity = pl_full.points[index].intensity;
                    point_tmp.curvature = pl_full.points[index].curvature;
                    point_tmp.normal_y = intensity_edge_weight;//normal_x
                    point_tmp.normal_z = 3;

                    pl_intensity_gradient.points.push_back(point_tmp);

                    rs_image_flag[(u/image_beilv)*image_v + v] = 1;
                    rs_image_flag[ ((u/image_beilv) - i)*image_v + v ] = 1;
                  }
                  break;
                }
              }
            }
          }
        }
      }
    }
  }

  pl_surf += pl_intensity_gradient;


  }
  else
  {
    double time_stamp = msg->header.stamp.toSec();
    // cout << "===================================" << endl;
    // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
    for (int i = 0; i < pl_orig.points.size(); i++)
    {
      if (i % point_filter_num != 0)
        continue;

      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;

      if (range < (blind * blind))
        continue;

      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.curvature = pl_orig.points[i].t * time_unit_scale; // curvature unit: ms
      added_pt.normal_z = 1;
      pl_surf.points.push_back(added_pt);
    }
  }

   if (1)
  {

    cv::cvtColor(image_intensity, image_intensity, CV_GRAY2RGB);
    cv::cvtColor(image_range, image_range, CV_GRAY2RGB);
    cv::cvtColor(image_intensity_gradient, image_intensity_gradient, CV_GRAY2RGB);
    cv::putText(image_intensity, "Intensity", cv::Point2f(5, 20 ), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
    cv::putText(image_range, "Range",     cv::Point2f(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
    cv::putText(image_intensity_gradient, "Intensity_Gradient",     cv::Point2f(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255,0,255), 2);
    image_header = msg->header;
  }
}



void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<velodyne_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  if (plsize == 0)
    return;
  pl_surf.reserve(plsize);

  /*** These variables only works when no point timestamps given ***/
  double omega_l = 0.361 * SCAN_RATE; // scan angular velocity
  std::vector<bool> is_first(N_SCANS, true);
  std::vector<double> yaw_fp(N_SCANS, 0.0);   // yaw of first scan point
  std::vector<float> yaw_last(N_SCANS, 0.0);  // yaw of last scan point
  std::vector<float> time_last(N_SCANS, 0.0); // last offset time
  /*****************************************************************/

  if (pl_orig.points[plsize - 1].time > 0)
  {
    given_offset_time = true;
  }
  else
  {
    given_offset_time = false;
    double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
    double yaw_end = yaw_first;
    int layer_first = pl_orig.points[0].ring;
    for (uint i = plsize - 1; i > 0; i--)
    {
      if (pl_orig.points[i].ring == layer_first)
      {
        yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
        break;
      }
    }
  }

  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    //sort(pl_orig.points.begin(), pl_orig.points.begin(), time_list2);

    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      int layer = pl_orig.points[i].ring;
      if (layer >= N_SCANS)
        continue;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time * time_unit_scale; // units: ms

      if (!given_offset_time)
      {
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
        if (is_first[layer])
        {
          // printf("layer: %d; is first: %d", layer, is_first[layer]);
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }

        if (yaw_angle <= yaw_fp[layer])
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
        }
        else
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
        }

        if (added_pt.curvature < time_last[layer])
          added_pt.curvature += 360.0 / omega_l;

        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      pl_buff[layer].points.push_back(added_pt);
    }

    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = pl.size();
      if (linesize < 2)
        continue;
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  else
  {
    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;
      // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;

      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = pl_orig.points[i].time * time_unit_scale; // curvature unit: ms // cout<<added_pt.curvature<<endl;
      // std::cout << "added_pt.curvature:" << added_pt.curvature << std::endl;


      if (!given_offset_time)
      {
        int layer = pl_orig.points[i].ring;
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

        if (is_first[layer])
        {
          // printf("layer: %d; is first: %d", layer, is_first[layer]);
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }

        // compute offset time
        if (yaw_angle <= yaw_fp[layer])
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
        }
        else
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
        }

        if (added_pt.curvature < time_last[layer])
          added_pt.curvature += 360.0 / omega_l;

        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      if (i % point_filter_num == 0)
      {
        if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind * blind))
        {
          pl_surf.points.push_back(added_pt);
        }
      }
    }
  }
}


void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)
{
  int plsize = pl.size();//
  int plsize2;
  if (plsize == 0)
  {
    printf("something wrong\n");
    return;
  }
  uint head = 0;
  // 
  while (types[head].range < blind)
  {
    head++;
  }


  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;// 

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());//
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());//

  uint i_nex = 0, i2;  // 
  uint last_i = 0; // 
  uint last_i_nex = 0;// 
  int last_state = 0;// 

    //
  int plane_type;

  for (uint i = head; i < plsize2; i++)
  {
    if (types[i].range < blind)// 
    {
      continue;
    }

    i2 = i;// i2 

    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);// 

    if (plane_type == 1) //
    {

      for (uint j = i; j <= i_nex; j++)
      {
        if (j != i && j != i_nex)
        {

          types[j].ftype = Real_Plane;
        }
        else
        {

          types[j].ftype = Poss_Plane;
        }
      }

      if (last_state == 1 && last_direct.norm() > 0.1)
      {
        double mod = last_direct.transpose() * curr_direct;
        if (mod > -0.707 && mod < 0.707)
        {

          types[i].ftype = Edge_Plane;
        }
        else
        {

          types[i].ftype = Real_Plane;
        }
      }

      i = i_nex - 1;
      last_state = 1;
    }
    else // if(plane_type == 2)
    {

      i = i_nex;
      last_state = 0;//
    }


    last_i = i2;  //
    last_i_nex = i_nex;    //
    last_direct = curr_direct;//
  }

  plsize2 = plsize > 3 ? plsize - 3 : 0; // 
  for (uint i = head + 3; i < plsize2; i++)
  {

    if (types[i].range < blind || types[i].ftype >= Real_Plane)
    {
      continue;
    }

    if (types[i - 1].dista < 1e-16 || types[i].dista < 1e-16)
    {
      continue;
    }

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);//
    Eigen::Vector3d vecs[2];

    for (int j = 0; j < 2; j++)
    {
      int m = -1;
      if (j == 1)
      {
        m = 1;
      }

      if (types[i + m].range < blind)
      {
        if (types[i].range > inf_bound) // inf_bound = 10;   // 
        {
          types[i].edj[j] = Nr_inf; // 
        }
        else
        {
          types[i].edj[j] = Nr_blind; // 
        }
        continue;
      }

      vecs[j] = Eigen::Vector3d(pl[i + m].x, pl[i + m].y, pl[i + m].z);
      vecs[j] = vecs[j] - vec_a; 

      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm(); 
      //std::cout <<  "types[i].angle[j] = " << types[i].angle[j] << std::endl;
      if (types[i].angle[j] < jump_up_limit)//cos(170)
      {
        types[i].edj[j] = Nr_180;// 
        //std::cout << "Nr_180" << std::endl;
      }
      else if (types[i].angle[j] > jump_down_limit)//cos(8)
      {
        types[i].edj[j] = Nr_zero;// 
        //std::cout << "Nr_zero" << std::endl;
      }// 

    }

    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();//
    if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_zero && types[i].dista > 0.0225 && types[i].dista > 4 * types[i - 1].dista)
    {
      if (types[i].intersect > cos160)//
      {
        if (edge_jump_judge(pl, types, i, Prev))// p
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }

    else if (types[i].edj[Prev] == Nr_zero && types[i].edj[Next] == Nr_nor && types[i - 1].dista > 0.0225 && types[i - 1].dista > 4 * types[i].dista)
    {
      if (types[i].intersect > cos160)
      {
        if (edge_jump_judge(pl, types, i, Next))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }

    else if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_inf)
    {
      if (edge_jump_judge(pl, types, i, Prev))
      {
        types[i].ftype = Edge_Jump;
      }
    }

    else if (types[i].edj[Prev] == Nr_inf && types[i].edj[Next] == Nr_nor)
    {
      if (edge_jump_judge(pl, types, i, Next))
      {
        types[i].ftype = Edge_Jump;
      }
    }

    else if (types[i].edj[Prev] > Nr_nor && types[i].edj[Next] > Nr_nor)
    {
      if (types[i].ftype == Nor)
      {
        types[i].ftype = Wire;// 
      }
    }
  }

  plsize2 = plsize - 1;
  double ratio;

  for (uint i = head + 1; i < plsize2; i++)
  {

    if (types[i].range < blind || types[i - 1].range < blind || types[i + 1].range < blind)
    {
      continue;
    }

    if (types[i - 1].dista < 1e-8 || types[i].dista < 1e-8)
    {
      continue;
    }

    if (types[i].ftype == Nor)
    {

      if (types[i - 1].dista > types[i].dista)
      {
        ratio = types[i - 1].dista / types[i].dista;
      }
      else
      {
        ratio = types[i].dista / types[i - 1].dista;
      }

      if (types[i].intersect < smallp_intersect && ratio < smallp_ratio)
      {

        if (types[i - 1].ftype == Nor)
        {
          types[i - 1].ftype = Real_Plane;
        }
        if (types[i + 1].ftype == Nor)
        {
          types[i + 1].ftype = Real_Plane;
        }
        types[i].ftype = Real_Plane;
      }
    }
  }
  //
  int last_surface = -1;
  for (uint j = head; j < plsize; j++)
  {

    if (types[j].ftype == Poss_Plane || types[j].ftype == Real_Plane)
    {
      if (last_surface == -1)
      {
        last_surface = j;
      }

      if (j == uint(last_surface + point_filter_num - 1))// 
      {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.intensity = pl[j].intensity;
        ap.curvature = pl[j].curvature;

        ap.normal_x = pl[j].normal_x;
        ap.normal_y = pl[j].normal_y;
        ap.normal_z = 1;

        pl_surf.push_back(ap);

        last_surface = -1;
      }
    }
    else
    {

      if (types[j].ftype == Edge_Jump || types[j].ftype == Edge_Plane)
      {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.intensity = pl[j].intensity;
        ap.curvature = pl[j].curvature;

        ap.normal_x = pl[j].normal_x;
        ap.normal_y = pl[j].normal_y;
        ap.normal_z = 2;
        
        pl_corn.push_back(ap);
      }

      if (last_surface != -1)
      {
        PointType ap;
        for (uint k = last_surface; k < j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.intensity += pl[k].intensity;
          ap.curvature += pl[k].curvature;

          ap.normal_x += pl[j].normal_x;
          ap.normal_y += pl[j].normal_y;

        }
        ap.x /= (j - last_surface);
        ap.y /= (j - last_surface);
        ap.z /= (j - last_surface);
        ap.intensity /= (j - last_surface);
        ap.curvature /= (j - last_surface);

        ap.normal_x =  int(ap.normal_x/(j - last_surface));
        ap.normal_y /= (j - last_surface);
        ap.normal_z = 1;

        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
{
  pl.height = 1;
  pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "livox";
  output.header.stamp = ct;
}


int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{

  double group_dis = disA * types[i_cur].range + disB;//disA = 0.01  disB = 0.1
  group_dis = group_dis * group_dis;// 
  // i_nex = i_cur;

  double two_dis;
  vector<double> disarr;//
  disarr.reserve(20);


  for (i_nex = i_cur; i_nex < i_cur + group_size; i_nex++)//
  {
    if (types[i_nex].range < blind)//
    {
      curr_direct.setZero();//
      return 2;// 2
    }
    disarr.push_back(types[i_nex].dista); //
  }

  for (;;) //
  {
    if ((i_cur >= pl.size()) || (i_nex >= pl.size()))//
      break;

    if (types[i_nex].range < blind)//
    {
      curr_direct.setZero();//
      return 2;// 2
    }

    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx * vx + vy * vy + vz * vz;//
    if (two_dis >= group_dis)//
    {
      break;
    }
    disarr.push_back(types[i_nex].dista); //
    i_nex++;// i_nex 
  }

  double leng_wid = 0;
  double v1[3], v2[3];
  for (uint j = i_cur + 1; j < i_nex; j++)
  {
    if ((j >= pl.size()) || (i_cur >= pl.size()))
      break;

    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    v2[0] = v1[1] * vz - vy * v1[2];
    v2[1] = v1[2] * vx - v1[0] * vz;
    v2[2] = v1[0] * vy - vx * v1[1];

    double lw = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2];
    if (lw > leng_wid)
    {
      leng_wid = lw;//()
    }
  }
  if ((two_dis * two_dis / leng_wid) < p2l_ratio) //p2l_ratio = 225; // 
  {
    curr_direct.setZero(); //
    return 0;
  }

  uint disarrsize = disarr.size();
  for (uint j = 0; j < disarrsize - 1; j++)
  {
    for (uint k = j + 1; k < disarrsize; k++)
    {
      if (disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }

  if (disarr[disarr.size() - 2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }

  if (lidar_type == AVIA)
  {
    double dismax_mid = disarr[0] / disarr[disarrsize / 2];
    double dismid_min = disarr[disarrsize / 2] / disarr[disarrsize - 2];

    if (dismax_mid >= limit_maxmid || dismid_min >= limit_midmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  else
  {
    double dismax_min = disarr[0] / disarr[disarrsize - 2];
    if (dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }

  curr_direct << vx, vy, vz;
  curr_direct.normalize();//
  return 1;
}


bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  if (nor_dir == 0)
  {
    if (types[i - 1].range < blind || types[i - 2].range < blind) //
    {
      return false;
    }
  }
  else if (nor_dir == 1)
  {
    if (types[i + 1].range < blind || types[i + 2].range < blind)//
    {
      return false;
    }
  }

  double d1 = types[i + nor_dir - 1].dista;
  double d2 = types[i + 3 * nor_dir - 2].dista;
  double d;

  if (d1 < d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

  if (d1 > edgea * d2 || (d1 - d2) > edgeb)
  {

    return false;
  }

  return true;
}


void Preprocess::rs_handler(const sensor_msgs::PointCloud2_<allocator<void>>::ConstPtr &msg)
{
  
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  //TODO czy
  pl_intensity_gradient.clear();
  vector<int>().swap(index_ig);

  pcl::PointCloud<rslidar_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);
  int plsize = pl_orig.points.size();
  //std::cout  << "plsize = " << plsize << std::endl;
  pl_surf.reserve(plsize);
  pl_corn.reserve(plsize);
  pl_full.reserve(plsize);
  /*** These variables only works when no point timestamps given ***/
  double omega_l = 0.361 * SCAN_RATE; // scan angular velocity//   
  std::vector<bool> is_first(N_SCANS, true);  //  N_SCANS = 32
  std::vector<double> yaw_fp(N_SCANS, 0.0);   // yaw of first scan point
  std::vector<float> yaw_last(N_SCANS, 0.0);  // yaw of last scan point
  std::vector<float> time_last(N_SCANS, 0.0); // last offset time
  /*****************************************************************/

  int image_beilv = image_vertical_rate;
  int image_u = N_SCANS  * image_beilv;
  int image_v = fov_degree/image_fov_resolution;
  int image_intensity_gradient_rate = 50;
// TODO czy
  assert((int)pl_orig.size() % N_SCANS * image_v == 0);
  // std::cout  <<  pl_orig.size() << std::endl ;
  //reset images

  image_range = cv::Mat(image_u, image_v, CV_8UC1, cv::Scalar(0));// 0 是黑色 ； 255 是白色
  //image_noise = cv::Mat(N_SCANS, 1800, CV_8UC1, cv::Scalar(0));
  image_intensity = cv::Mat(image_u, image_v, CV_8UC1, cv::Scalar(0));
  image_intensity_gradient = cv::Mat(image_u, image_v, CV_8UC1, cv::Scalar(0));

  if (pl_orig.points[plsize - 1].timestamp > 0) // todo check pl_orig.points[plsize - 1].time
  {
    given_offset_time = true;
  }
  else
  {
    given_offset_time = false;
    double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578; //

    double yaw_end = yaw_first;
    int layer_first = pl_orig.points[0].ring; // ( 0)  
    for (uint i = plsize - 1; i > 0; i--)     //   
    {
      if (pl_orig.points[i].ring == layer_first)
      {
        yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578; // 
        break;
      }
    }
  }
    double rs_t0 = omp_get_wtime();
  if(feature_enabled)
  {
    static int rs_count = 0;
    static double rs_time = 0.0;
    rs_count++;

    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    sort(pl_orig.points.begin(), pl_orig.points.begin(), time_list1);

    int invaild_pl = 0;

  for (int i = 0; i < plsize; i++)
  {
    PointType added_pt;

    int layer = pl_orig.points[i].ring; // 
    //std::cout << "layer = " << layer << std::endl;
      if (layer >= N_SCANS)
        continue;
    added_pt.normal_x = 0; // 
    added_pt.normal_y = 0;
    added_pt.normal_z = 0;
    added_pt.x = pl_orig.points[i].x;
    added_pt.y = pl_orig.points[i].y;
    added_pt.z = pl_orig.points[i].z;
    added_pt.intensity = pl_orig.points[i].intensity;
    //std::cout << "added_pt.intensity = " <<  added_pt.intensity<< std::endl;
    added_pt.curvature = (pl_orig.points[i].timestamp - pl_orig.points[0].timestamp) * 1000.0; // curvature unit: ms//存放当前帧当前点距离第一个点的时间间隔
    // std::cout << "added_pt.curvature:" << added_pt.curvature << std::endl;

    // TODO czy
    float current_range = std::sqrt(added_pt.x*added_pt.x + added_pt.y*added_pt.y + added_pt.z*added_pt.z);
    float current_intensity = std::min(float(added_pt.intensity), 255.0f);

    double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957 + fov_degree/2; // yaw 
    pl_full[i].normal_y = current_range;// 
    if( current_intensity < 2)
    {
      current_intensity = 0;
    }
    if ((current_intensity > 220  || current_intensity < 10 )&&  current_range < intensity_min_range)
      continue;
    //if (layer == 0) std::cout <<  "layer = " << 0 << std::endl;
    if (!given_offset_time)// 
    {
      if (is_first[layer]) // 
      {
        // printf("layer: %d; is first: %d", layer, is_first[layer]);
        yaw_fp[layer] = yaw_angle; //
        is_first[layer] = false;
        added_pt.curvature = 0.0; // 时间设为 0 
        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
        continue;
      }

      // compute offset time
      if (yaw_angle <= yaw_fp[layer]) // 
      {
        added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
      }
      else
      {
        added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
      }

      if (added_pt.curvature < time_last[layer])
        added_pt.curvature += 360.0 / omega_l;

      yaw_last[layer] = yaw_angle;
      time_last[layer] = added_pt.curvature;
    }

    if (i % 1 == 0) // 
    {
      if( current_range >= blind && current_range <= current_range_max) // 
      {
        if (layer < N_SCANS && layer >= 0)
        {
          int v = yaw_angle/image_fov_resolution;
          int u = layer;
          if(u >0 && v > 5)
          {
            image_range.at<uint8_t>((N_SCANS-1 - u) * image_beilv, image_v-1-v) = std::min(current_range * 20, 255.0f);
            image_intensity.at<uint8_t>((N_SCANS-1 - u)*image_beilv, image_v-1-v) = current_intensity;
            rs_image_to_pl[u * image_v + v] = invaild_pl;
            rs_image_flag[u * image_v + v] = 0;
            added_pt.normal_x = u * image_v + v;
            pl_full.points.push_back(added_pt);
            if(i % point_filter_num == 0)//  
                pl_buff[layer].points.push_back(added_pt);
            invaild_pl++;
          }

        }
      }
    }
  }

  // 寻找面点
    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      int linesize = pl.size();
      if (linesize < 2)
        continue;
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }

  //std::cout <<  " pl_surf = " << pl_surf.points.size() << std::endl;
  //std::cout <<  " pl_corn = " << pl_corn.points.size() << std::endl;
  pl_surf += pl_corn;
  //std::cout <<  " pl_surf + pl_corn = " << pl_surf.points.size() << std::endl;

  float i_intensity;
  float i_1_intensity;
  float i_2_intensity;
  int intensity_tmp = 0;
  float i_intensity_gradient;
  float tmp1,tmp2;

  int flag_min = 0;
  int flag_min_1 = 0;
  int flag_min_2 = 0;
  int flag_max = 0;
  int flag_max_1 = 0;
  int flag_max_2 = 0;
  bool flag_min_ok = false;
  bool flag_max_ok = false;

  float intensity_max_ave = 0; // 
  float intensity_max_ave_tmp1 = 0;
  float intensity_max_ave_tmp2 = 0;

  float intensity_min_ave = 0; // 
  float intensity_min_ave_tmp1 = 0;
  float intensity_min_ave_tmp2 = 0;
  
  float selected_between_range = 0 ; // 
  float intensity_edge_weight = 0; //
  int neigher_point  = 0; // 

  bool flag_shuiping = false;
  bool flag_chuizhi = false;

  for (int u = 0; u < N_SCANS * image_beilv; u++) 
  {
    if(u%image_beilv != 0) continue;

    for (int v = 0; v < image_v; v++) 
    {
      //rs_image_flag[(u/image_beilv)*image_v+v] = 0;

      i_intensity_gradient = 0;
      flag_max_ok = false;
      flag_min_ok = false;
      flag_shuiping = false;
      flag_chuizhi = false;
      flag_min_1 = 0;
      flag_min_2 = 0;
      flag_max_1 = 0;
      flag_max_2 = 0;
      intensity_max_ave_tmp1 = 0;
      intensity_max_ave_tmp2 = 0;
      intensity_min_ave_tmp1 = 0;
      intensity_min_ave_tmp2 = 0;
      
      if( u < 3 * image_beilv)
      {
        if ( v <= 5 ) 
          continue;

        i_intensity = image_intensity.at<uint8_t>(u, v); // 取出强度
        float  intensity_dient_min_tmp =  min(intensity_dient_min ,  float(i_intensity * 0.15));
        float  intensity_dient_max_tmp =  max(intensity_dient_max ,  float(i_intensity * 0.20));
        if (i_intensity == 0) // 对于强度为 0 ，说明没有对应的三维点存在
          continue;
        intensity_max_ave = i_intensity;
        intensity_min_ave = 0;

        for(int i = 1; i < 2; i ++)
        {
          if(rs_image_flag[(u/image_beilv)*image_v+v+i] != 1)
          {
            i_1_intensity = image_intensity.at<uint8_t>(u, v + i); // 取出上一个同一scan的点的强度
            if (i_1_intensity != 0)
            {
              i_intensity_gradient = i_1_intensity - i_intensity;
              if(abs(i_intensity_gradient) <  intensity_dient_min_tmp )
              {
                //intensity_max_ave += i_1_intensity;

                if( i_intensity_gradient <= 0 )
                {
                  flag_max_1++;
                  intensity_max_ave_tmp1 += i_1_intensity;
                }
                else 
                {
                  flag_max_2++;
                  intensity_max_ave_tmp2 += i_1_intensity;
                }

                 if( flag_max_1 >  flag_max_2)
                 {
                  flag_max = flag_max_1;
                  intensity_max_ave += intensity_max_ave_tmp1;
                 }
                 else
                 {
                  flag_max = flag_max_2;
                  intensity_max_ave += intensity_max_ave_tmp2;
                 }
                //flag_max = max(flag_max_1, flag_max_2);

                if( flag_max >= lookfor_max_num )
                {
                  flag_max_ok = true;

                  intensity_max_ave /= flag_max;
                }

              }
              //rs_image_flag[(u/image_beilv)*1800+v] = 1;
              //rs_image_flag[(u/image_beilv)*1800+v-i] = 1;
            }
          }

          if(rs_image_flag[(u/image_beilv)*image_v+v-i] != 1)
          {
            i_1_intensity = image_intensity.at<uint8_t>(u, v - i); // 取出上一个同一scan的点的强度

            if (i_1_intensity != 0)
            {
              i_intensity_gradient = i_intensity - i_1_intensity  ;
              if(abs(i_intensity_gradient) >  intensity_dient_max_tmp )
              {
                if(neigher_point == 0 )
                  neigher_point = i;
                //intensity_min_ave += i_1_intensity;

                if( i_intensity_gradient <= 0 )
                {
                  flag_min_1++;
                  intensity_min_ave_tmp1 += i_1_intensity;
                }
                else 
                {
                  flag_min_2++;
                  intensity_min_ave_tmp2 += i_1_intensity;
                }

                 if( flag_min_1 >  flag_min_2)
                 {
                  flag_max = flag_min_1;
                  intensity_min_ave += intensity_min_ave_tmp1;
                 }
                 else
                 {
                  flag_min = flag_min_2;
                  intensity_min_ave += intensity_min_ave_tmp2;
                 }
                //flag_min = max(flag_min_1, flag_min_2);

                if( flag_min >= lookfor_min_num )
                {
                  flag_min_ok = true;

                  intensity_min_ave /= flag_min;
                }
              }
              //rs_image_flag[(u/image_beilv)*1800+v] = 1;
              //rs_image_flag[(u/image_beilv)*1800+v-i] = 1;
            }
          }
          else 
          {
            break;
          }
        }

        if(flag_max_ok == true && flag_min_ok == true)
        {
          int index = rs_image_to_pl[(N_SCANS-1  - (u/image_beilv))*image_v+(image_v-1 - v)];
          int index_neigher = rs_image_to_pl[(N_SCANS-1  - (u/image_beilv))*image_v+(image_v-1 - (v - neigher_point ))];
          float selected_between_range_x = pl_full.points[index].x - pl_full.points[index_neigher].x;
          float selected_between_range_y = pl_full.points[index].y - pl_full.points[index_neigher].y;
          float selected_between_range_z = pl_full.points[index].z - pl_full.points[index_neigher].z;
          selected_between_range = sqrt( selected_between_range_x * selected_between_range_x + selected_between_range_y* selected_between_range_y + selected_between_range_z*selected_between_range_z);

          //std::cout << "(intensity_max_ave - intensity_min_ave) / 255 = " << (intensity_max_ave - intensity_min_ave) / 255 << std::endl;
          intensity_edge_weight =1+(exp(-abs( (intensity_max_ave - intensity_min_ave) / 255 ) ));
          //std::cout << "intensity_edge_weight = " << intensity_edge_weight << std::endl;
          intensity_edge_weight = intensity_edge_weight * ( 1/(1+exp(-selected_between_range)) );
          //std::cout << "intensity_edge_weight = " << intensity_edge_weight << std::endl;
          //pl_full.points[index].normal_y = intensity_edge_weight;
          i_intensity_gradient = abs( pl_full.points[index].intensity - pl_full.points[index_neigher].intensity );

          if (i_intensity_gradient > 20)
          {
            image_intensity_gradient.at<uint8_t>(u , v) = i_intensity_gradient * image_intensity_gradient_rate;

            PointType point_tmp;
            point_tmp.x = (pl_full.points[index].x + pl_full.points[index_neigher].x)/2;
            point_tmp.y = (pl_full.points[index].y + pl_full.points[index_neigher].y)/2;
            point_tmp.z = (pl_full.points[index].z + pl_full.points[index_neigher].z)/2;
            point_tmp.intensity = pl_full.points[index].intensity;
            point_tmp.curvature = pl_full.points[index].curvature;
            point_tmp.normal_y = intensity_edge_weight;//normal_x
            point_tmp.normal_z = 3;

            pl_intensity_gradient.points.push_back(point_tmp);

            rs_image_flag[(u/image_beilv)*image_v + v] = 1;
            rs_image_flag[(u/image_beilv)*image_v + v - neigher_point ] = 1;
          }
        }
      }
      else
      {

        if ( v <= 5 ) 
          continue;

        i_intensity = image_intensity.at<uint8_t>(u, v); // 取出强度
        float  intensity_dient_min_tmp =  min(intensity_dient_min ,  float(i_intensity * 0.15));
        float  intensity_dient_max_tmp =  max(intensity_dient_max ,  float(i_intensity * 0.20));
        if (i_intensity == 0) // 对于强度为 0 ，说明没有对应的三维点存在
          continue;
        intensity_max_ave = i_intensity;
        for(int i = 1; i < 2; i ++)
        {
          if(rs_image_flag[(u/image_beilv)*image_v+v+i] != 1)
          {
            i_1_intensity = image_intensity.at<uint8_t>(u, v + i); // 取出上一个同一scan的点的强度
            if (i_1_intensity != 0)
            {
              i_intensity_gradient = i_1_intensity - i_intensity;
              if(abs(i_intensity_gradient) <  intensity_dient_min_tmp )
              {
                //intensity_max_ave += i_1_intensity;

                if( i_intensity_gradient <= 0 )
                {
                  flag_max_1++;
                  intensity_max_ave_tmp1 += i_1_intensity;
                }
                else 
                {
                  flag_max_2++;
                  intensity_max_ave_tmp2 += i_1_intensity;
                }

                 if( flag_max_1 >  flag_max_2)
                 {
                  flag_max = flag_max_1;
                  intensity_max_ave += intensity_max_ave_tmp1;
                 }
                 else
                 {
                  flag_max = flag_max_2;
                  intensity_max_ave += intensity_max_ave_tmp2;
                 }
                //flag_max = max(flag_max_1, flag_max_2);

                if( flag_max >= lookfor_max_num )
                {
                  flag_max_ok = true;

                  intensity_max_ave /= flag_max;
                }

              }
              //rs_image_flag[(u/image_beilv)*1800+v] = 1;
              //rs_image_flag[(u/image_beilv)*1800+v-i] = 1;
            }
          }

          if(rs_image_flag[(u/image_beilv)*image_v+v-i] != 1)
          {
            i_1_intensity = image_intensity.at<uint8_t>(u, v - i); // 取出上一个同一scan的点的强度

            if (i_1_intensity != 0)
            {
              i_intensity_gradient = i_intensity - i_1_intensity  ;
              if(abs(i_intensity_gradient) >  intensity_dient_max_tmp )
              {
                if(neigher_point == 0 )
                  neigher_point = i;
                //intensity_min_ave += i_1_intensity;

                if( i_intensity_gradient <= 0 )
                {
                  flag_min_1++;
                  intensity_min_ave_tmp1 += i_1_intensity;
                }
                else 
                {
                  flag_min_2++;
                  intensity_min_ave_tmp2 += i_1_intensity;
                }

                 if( flag_min_1 >  flag_min_2)
                 {
                  flag_max = flag_min_1;
                  intensity_min_ave += intensity_min_ave_tmp1;
                 }
                 else
                 {
                  flag_min = flag_min_2;
                  intensity_min_ave += intensity_min_ave_tmp2;
                 }
                //flag_min = max(flag_min_1, flag_min_2);

                if( flag_min >= lookfor_min_num )
                {
                  flag_min_ok = true;

                  intensity_min_ave /= flag_min;
                }
              }
              //rs_image_flag[(u/image_beilv)*1800+v] = 1;
              //rs_image_flag[(u/image_beilv)*1800+v-i] = 1;
            }
          }
          else 
          {
            break;
          }
        }

        if(flag_max_ok == true && flag_min_ok == true)
        {
          int index = rs_image_to_pl[(N_SCANS-1  - (u/image_beilv))*image_v+(image_v-1 - v)];
          int index_neigher = rs_image_to_pl[(N_SCANS-1  - (u/image_beilv))*image_v+(image_v-1 - (v - neigher_point ))];
          float selected_between_range_x = pl_full.points[index].x - pl_full.points[index_neigher].x;
          float selected_between_range_y = pl_full.points[index].y - pl_full.points[index_neigher].y;
          float selected_between_range_z = pl_full.points[index].z - pl_full.points[index_neigher].z;
          selected_between_range = sqrt( selected_between_range_x * selected_between_range_x + selected_between_range_y* selected_between_range_y + selected_between_range_z*selected_between_range_z);

          //std::cout << "(intensity_max_ave - intensity_min_ave) / 255 = " << (intensity_max_ave - intensity_min_ave) / 255 << std::endl;
          intensity_edge_weight =1+(exp(-abs( (intensity_max_ave - intensity_min_ave) / 255 ) ));
          //std::cout << "intensity_edge_weight = " << intensity_edge_weight << std::endl;
          intensity_edge_weight = intensity_edge_weight * ( 1/(1+exp(-selected_between_range)) );

          i_intensity_gradient = abs( pl_full.points[index].intensity - pl_full.points[index_neigher].intensity );

          if (i_intensity_gradient > 20)
          {
            image_intensity_gradient.at<uint8_t>(u , v) = i_intensity_gradient * image_intensity_gradient_rate;

            PointType point_tmp;
            point_tmp.x = (pl_full.points[index].x + pl_full.points[index_neigher].x)/2;
            point_tmp.y = (pl_full.points[index].y + pl_full.points[index_neigher].y)/2;
            point_tmp.z = (pl_full.points[index].z + pl_full.points[index_neigher].z)/2;
            point_tmp.intensity = pl_full.points[index].intensity;
            point_tmp.curvature = pl_full.points[index].curvature;
            point_tmp.normal_y = intensity_edge_weight;//normal_x
            point_tmp.normal_z = 3;

            pl_intensity_gradient.points.push_back(point_tmp);

            rs_image_flag[(u/image_beilv)*image_v + v] = 1;
            rs_image_flag[(u/image_beilv)*image_v + v - neigher_point ] = 1;
          }
        }
        else
        {
          for(int i = 1; i < 2; i ++)
          {
            if(rs_image_flag[((u/image_beilv)-i)*image_v + v - i] != 1) 
            {
              i_2_intensity = image_intensity.at<uint8_t>(u - image_beilv*i, v ); // 取出上一个同一scan的点的强度
              {
                if (i_2_intensity != 0)
                {
                  int index = rs_image_to_pl[(N_SCANS-1  - (u/image_beilv))*image_v+(image_v-1 - v)];
                  int index_neigher = rs_image_to_pl[(N_SCANS-1  - ((u/image_beilv) - i ))*image_v+(image_v-1 - (v ))];
                  float selected_between_range_x = pl_full.points[index].x - pl_full.points[index_neigher].x;
                  float selected_between_range_y = pl_full.points[index].y - pl_full.points[index_neigher].y;
                  float selected_between_range_z = pl_full.points[index].z - pl_full.points[index_neigher].z;
                  selected_between_range = sqrt( selected_between_range_x * selected_between_range_x + selected_between_range_y* selected_between_range_y + selected_between_range_z*selected_between_range_z);
                  if(selected_between_range > 0.5)
                    break;
                  //std::cout << "(intensity_max_ave - intensity_min_ave) / 255 = " << (intensity_max_ave - intensity_min_ave) / 255 << std::endl;
                  intensity_edge_weight =1+(exp(-abs( (pl_full.points[index].intensity - pl_full.points[index_neigher].intensity) / 255 ) ));
                  //std::cout << "intensity_edge_weight = " << intensity_edge_weight << std::endl;
                  intensity_edge_weight = intensity_edge_weight * ( 1/(1+exp(-selected_between_range)) );
                  i_intensity_gradient = abs( pl_full.points[index].intensity - pl_full.points[index_neigher].intensity );

                  if (i_intensity_gradient > 25)
                  {
                    image_intensity_gradient.at<uint8_t>(u , v) = i_intensity_gradient * image_intensity_gradient_rate;

                    PointType point_tmp;
                    point_tmp.x = (pl_full.points[index].x + pl_full.points[index_neigher].x)/2;
                    point_tmp.y = (pl_full.points[index].y + pl_full.points[index_neigher].y)/2;
                    point_tmp.z = (pl_full.points[index].z + pl_full.points[index_neigher].z)/2;
                    point_tmp.intensity = pl_full.points[index].intensity;
                    point_tmp.curvature = pl_full.points[index].curvature;
                    point_tmp.normal_y = intensity_edge_weight;//normal_x
                    point_tmp.normal_z = 3;

                    pl_intensity_gradient.points.push_back(point_tmp);

                    rs_image_flag[(u/image_beilv)*image_v + v] = 1;
                    rs_image_flag[ ((u/image_beilv) - i)*image_v + v ] = 1;
                  }
                  break;
                }
              }
            }
          }
        }
      }
    }
  }
  pl_surf += pl_intensity_gradient;
    double avia_t1 = omp_get_wtime();
    avia_delt_t = avia_t1 - rs_t0;
    rs_time += avia_delt_t;

    static bool is_file_created = false;
    std::string WORK_SPACE_PATH = "/home/bupo/my_study/ig_lio_git/ig_lio";
    if(1)
    {
      if (!is_file_created) 
      {
        if (!CreateDirectory(WORK_SPACE_PATH + "/slam_data/trajectory"))
          {
            is_file_created = false;
            printf("avia Feature extraction : NO feature_time Directory");
          }
        if (!CreateFile(feature_time, WORK_SPACE_PATH + "/slam_data/trajectory/feature_time.txt"))
          {
            is_file_created = false;
            printf("avia Feature extraction : NO feature_time.txt");
          }
        is_file_created = true;
      }

      if(is_file_created)
      {
        feature_time << avia_delt_t;
        feature_time << std::endl;
      }
    }

  }
  else // 不进行特征提取
  {
    for (int i = 0; i < plsize; i++)
    {
      PointType added_pt;

      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = (pl_orig.points[i].timestamp - pl_orig.points[0].timestamp) * 1000.0; // curvature unit: ms//存放当前帧当前点距离第一个点的时间间隔
      // std::cout << "added_pt.curvature:" << added_pt.curvature << std::endl;
      added_pt.normal_z = 1;

      if (!given_offset_time)
      {
        int layer = pl_orig.points[i].ring;
        double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

        if (is_first[layer])
        {
          // printf("layer: %d; is first: %d", layer, is_first[layer]);
          yaw_fp[layer] = yaw_angle;
          is_first[layer] = false;
          added_pt.curvature = 0.0;
          yaw_last[layer] = yaw_angle;
          time_last[layer] = added_pt.curvature;
          continue;
        }

        // compute offset time
        if (yaw_angle <= yaw_fp[layer])
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
        }
        else
        {
          added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
        }

        if (added_pt.curvature < time_last[layer])
          added_pt.curvature += 360.0 / omega_l;

        yaw_last[layer] = yaw_angle;
        time_last[layer] = added_pt.curvature;
      }

      if (i % point_filter_num == 0)
      {
        if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind * blind) && abs(added_pt.z) < 4)
        {
          pl_surf.points.push_back(added_pt);
        }
      }
    }

  }
   if (1)
  {
    cv::cvtColor(image_intensity, image_intensity, CV_GRAY2RGB);
    cv::cvtColor(image_range, image_range, CV_GRAY2RGB);
    cv::cvtColor(image_intensity_gradient, image_intensity_gradient, CV_GRAY2RGB);
    cv::putText(image_intensity, " ", cv::Point2f(5, 20 ), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,255), 1);
    cv::putText(image_range, " ",     cv::Point2f(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,255), 1);
    cv::putText(image_intensity_gradient, " ",     cv::Point2f(5, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,255), 1);
    image_header = msg->header;
  }
}
