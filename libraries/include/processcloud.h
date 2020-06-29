#ifndef PROCESSCLOUD_H
#define PROCESSCLOUD_H

#include "ros/ros.h"

#include <math.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/videoio.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/calib3d.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <string>
#include <iostream>
#include <ros/ros.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <cstdlib>
#include <csignal>
#include <ctime>
#include <math.h>

/// Essa classe tera todas as funçoes para trabalhar a nuvem de pontos, calcular normais, colorir,
/// salvar nuvens e imagens e ate mesmo projetar os pontos para criar a imagem de laser virtual
using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace Eigen;

typedef PointXYZRGB       PointT ;
typedef PointXYZRGBNormal PointTN;

class ProcessCloud
{
public:
  ProcessCloud(string p);
  virtual ~ProcessCloud();
  void calculateNormals(PointCloud<PointT>::Ptr in, PointCloud<PointTN>::Ptr acc_normal);
  void calculateNormals(PointCloud<PointTN>::Ptr acc_normal);
  void colorCloudThroughDistance(PointCloud<PointTN>::Ptr nuvem);
  void transformToCameraFrame(PointCloud<PointTN>::Ptr nuvem);
  void transformToCameraFrame(PointCloud<PointT>::Ptr nuvem);
  void transformCloudServoAngles(PointCloud<PointT>::Ptr cloud, float pan, float tilt, nav_msgs::Odometry &msg, Eigen::Matrix4f &T, Eigen::Vector3f &C);
  void createVirtualLaserImage(PointCloud<PointTN>::Ptr nuvem, std::string nome, int w, int h);
  void saveCloud(PointCloud<PointTN>::Ptr nuvem, string nome);
  void saveCloud(PointCloud<PointT>::Ptr nuvem, string nome);
  void saveImage(cv::Mat img, std::string nome);
  void filterCloudDepthCovariance(PointCloud<PointT >::Ptr cloud, int kn, float thresh);
  void filterCloudDepthCovariance(PointCloud<PointTN>::Ptr cloud, int kn, float thresh);
  void preprocess(PointCloud<PointT>::Ptr cin, PointCloud<PointTN>::Ptr out);

  Mat projectCloudToLaserCenter(PointCloud<PointTN>::Ptr cloud, float fx, float fy, float tx, float ty, Size s);
  void colorCloudWithCalibratedImage(PointCloud<PointTN>::Ptr cloud_in, Mat image, float fx, float fy);
  void colorCloudWithCalibratedImage(PointCloud<PointT>::Ptr cloud_in, Mat image, float fx, float fy);
  void applyPolynomialFilter(vector<PointCloud<PointT>> vetor_nuvens_in, vector<PointCloud<PointTN>> &vetor_nuvens, int grau, double r);

  void writeNVM(std::string nome, std::string nome_imagem, Eigen::VectorXf params);
  void compileFinalNVM(vector<std::string> linhas);
  std::string escreve_linha_imagem(float foco, std::string nome, Eigen::MatrixXf C, Eigen::Quaternion<float> q);


private:
  /// Metodos
  float normaldist(float x, float media, float dev);
  Eigen::Matrix3f euler2matrix(float r, float p, float y);
  Mat correctColorCluster(Mat in);
  Vec3b findPredominantColor(int u, int v, Mat in, int desvio);

  /// Variaveis
  Eigen::Matrix3f K_cam; // Parametros intrinsecos da camera
  int cam_w, cam_h;      // Dimensoes da camera
  std::string pasta;     // Nome da pasta a salvar as coisas

};

#endif // PROCESSCLOUD_H
