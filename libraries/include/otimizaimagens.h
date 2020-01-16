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

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace cv;
using namespace std;

class OtimizaImagens
{
public:
  OtimizaImagens(std::string p, std::string icam, std::string iclu, std::string id);
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

  // Arquivos de imagem
  std::string pasta, arquivo_cam, arquivo_clusters, arquivo_depth;
  // Imagens fonte para estudo
  Mat im_cam, im_depth, im_clusters;
  // Imagens com arestas
  Mat ed_cam, ed_depth, ed_clusters;
};

#endif // OTIMIZAIMAGENS_H
