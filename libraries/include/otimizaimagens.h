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

#include "processcloud.h"

using namespace cv;
using namespace std;
using namespace pcl;
using namespace pcl::io;

typedef PointXYZRGBNormal PointTN;

class OtimizaImagens
{
public:
  OtimizaImagens(std::string p, std::string icam, std::string iclu, std::string id);
  virtual ~OtimizaImagens();

//  void calculateEdgesOnImages();
  Mat calculateEdgeFromOriginalImage(Mat image, string nome);
  Mat calculateHoughTransformFromOriginalImage(Mat in, string nome);
  void saveEdgeImages();
//  void calcAndMatchFeatures();
//  void adjustImagesKeyboard();

  Mat getImage(std::string nome);
  Mat correctColorCluster(Mat in);

  float FOB(Mat rgb, Mat clu);

private:
  // Métodos
  Vec3b findPredominantColor(int u, int v, Mat in, int desvio);
//  Mat calculateBlobs(Mat in);
  Mat calculateContours(Mat in);
//  Mat adjustImageByFocus(Mat in, float fx_r, float fy_r);
//  Mat adjustImageByProjection(Mat in, float fx, float fy, float tx, float ty);
  Mat removeOuterEdges(Mat in);

  // Objeto da classe de trabalho com a nuvem

  // Arquivos de imagem
  std::string pasta, arquivo_cam, arquivo_clusters, arquivo_depth;
  // Imagens fonte para estudo
  Mat im_cam, im_depth, im_clusters;
  // Imagens com arestas
  Mat ed_cam, ed_depth, ed_clusters;
  // Focos para a imagem virtual do laser a serem otimizados a partir do vindo da camera
  float fx_l, fy_l;
  // Parametros e matriz da camera
  Mat params, K;
};

#endif // OTIMIZAIMAGENS_H
