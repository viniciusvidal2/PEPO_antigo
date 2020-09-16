/// Includes
///
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <math.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>
#include <pcl/common/common.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "../../libraries/include/processcloud.h"
#include "../../libraries/include/registerobjectoptm.h"

/// Namespaces
///
using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace std;

/// Definiçoes
///
typedef PointXYZRGB       PointT ;
typedef PointXYZRGBNormal PointTN;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "planta_baixa");
  ros::NodeHandle nh;
  ros::NodeHandle n_("~");
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
  ROS_INFO("Iniciando o calculo da planta baixa ...");

  // Parametros
  string nome_param;
  float lado_quadrado;
  n_.param<string>("pasta"   , nome_param   , string("Dados_PEPO"));
  n_.param<float >("resolucao", lado_quadrado, 0.02); // Resolucao da separacao da nuvem em supervoxels quadriculados para formar a imagem

  // Pasta inicial com a nuvem
  char* home;
  home = getenv("HOME");
  string pasta = string(home)+"/Desktop/"+nome_param.c_str()+"/";

  // Lendo a nuvem acumulada
  ROS_INFO("Carregando nuvem acumulada do space ...");
  PointCloud<PointT>::Ptr nuvem_inicial (new PointCloud<PointT>);
  loadPLYFile<PointT>(pasta+"acumulada.ply", *nuvem_inicial);
  ROS_INFO("Nuvem carregada com %zu pontos.", nuvem_inicial->size());

  ros::Time tempo = ros::Time::now();

  /// Separando a nuvem em clusters perpendiculares ao eixo y - y negativo para cima
  ///
  // Filtrando a altura que vai entrar na roda
  ROS_INFO("Filtrando a altura da nuvem ...");
  float metros_altura_acima_pepo = 2; // quantos metros acima do PEPO para fazer a nuvem
  PassThrough<PointT> pass;
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-metros_altura_acima_pepo, 100); // Negativo de tudo no eixo X
  pass.setNegative(false);
  pass.setInputCloud(nuvem_inicial);
  pass.filter(*nuvem_inicial);
  ROS_INFO("Nuvem filtrada com %zu pontos.", nuvem_inicial->size());
  // Pega limites da nuvem
  PointT min_limits, max_limits;
  getMinMax3D(*nuvem_inicial, min_limits, max_limits);
  // Criando vetor de vetores para separar a nuvem de acordo com as dimensoes XZ
  float spanx = int(max_limits.x - min_limits.x), spanz = int(max_limits.z - min_limits.z);
  int w = spanx/lado_quadrado, h = spanz/lado_quadrado;
  vector< vector<PointT> > supervoxels(w);
  for(int i=0; i<supervoxels.size(); i++) supervoxels[i].resize(h);
  // Preenchendo o os supervoxels com um ponto em altura inicial para a comparacao
  PointT pini;
  pini.y = 100;
#pragma omp parallel for
  for(int u=0; u<supervoxels.size(); u++){
      for(int v=0; v<supervoxels[u].size(); v++)
          supervoxels[u][v] = pini;
  }
  // Destinando cada ponto da nuvem original para o seu local no vetor de vetores segundo dimensoes, se for mais alto que o anterior
  ROS_INFO("Comparando os pontos nos supervoxels ...");
  for(size_t i=0; i<nuvem_inicial->size(); i++){
      int u = abs(nuvem_inicial->points[i].x - min_limits.x)/spanx * w, v = abs(nuvem_inicial->points[i].z - min_limits.z)/spanz * h;
      if(u >= w) u = w - 1;
      if(v >= h) v = h - 1;
      if(nuvem_inicial->points[i].y < supervoxels[u][v].y)
          supervoxels[u][v] = nuvem_inicial->points[i];
  }
  nuvem_inicial->clear();

  /// Colorindo a imagem final de acordo com a resolucao da separacao da nuvem
  ///
  // Iniciar a imagem com a quantidade de pixels de acordo com numero de supervoxels
  Mat planta = Mat::zeros(cv::Size(w, h), CV_8UC3);
  // Processando em paralelo, procurar ponto mais alto de cada supervoxel
  ROS_INFO("Colorindo a imagem ...");
#pragma omp parallel for
  for(int u=0; u<supervoxels.size(); u++){
      for(int v=0; v<supervoxels[u].size(); v++){
          PointT p = supervoxels[u][v];
          Vec3b cor;
          // Atribuir cor do ponto mais alto aquele lugar da foto
          cor.val[0] = p.r; cor.val[1] = p.g; cor.val[2] = p.b;
          planta.at<Vec3b>(h-1-v, u) = cor;
      }
  }

  ROS_WARN("Tempo para processamento: %.2f", (ros::Time::now() - tempo).toSec());

  /// Mostrando resultado e salvando
  ///
  ROS_INFO("Salvando e mostrando a planta ...");
  imwrite(pasta+"planta_baixa.png", planta);
  imshow("planta baixa", planta);
  waitKey(0);

  ros::spinOnce();
  return 0;
}
