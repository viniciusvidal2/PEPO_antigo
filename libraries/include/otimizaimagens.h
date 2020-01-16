#ifndef OTIMIZAIMAGENS_H
#define OTIMIZAIMAGENS_H

#include <string>
#include <math.h>
#include <cstdlib>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
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
#include <pcl/surface/mls.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace cv;
using namespace std;
using namespace pcl;
using namespace pcl::io;

typedef PointXYZRGBNormal PointT;

class OtimizaImagens
{
public:
  OtimizaImagens(std::string p, std::string icam, std::string iclu, std::string id, std::string dist);
  virtual ~OtimizaImagens();

  void calculateEdgesOnImages();
  void saveEdgeImages();
  void calcAndMatchFeatures();
  void adjustImagesKeyboard();


private:
  // Métodos
  Mat correctColorCluster(Mat in);
  Vec3b findPredominantColor(int u, int v, Mat in, int desvio);
  Mat calculateBlobs(Mat in);
  Mat calculateContours(Mat in);
  Mat adjustImageByFocus(Mat in, float fx_r, float fy_r);

  // Arquivos de imagem
  std::string pasta, arquivo_cam, arquivo_clusters, arquivo_depth, arquivo_distancias;
  // Imagens fonte para estudo
  Mat im_cam, im_depth, im_clusters, im_dist;
  // Imagens com arestas
  Mat ed_cam, ed_depth, ed_clusters;
  // Focos para a imagem virtual do laser a serem otimizados a partir do vindo da camera
  float fx_l, fy_l;
};

#endif // OTIMIZAIMAGENS_H
