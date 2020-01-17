#include <ros/ros.h>

#include <iostream>
#include <ros/ros.h>
#include <string>
#include <math.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "../../libraries/include/otimizaimagens.h"

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    /// Inicia o no ROS
    ros::init(argc, argv, "processa_imagens");
    ros::NodeHandle nh;

    /// Iniciar classe de processamento de dados
    std::string pasta  = "/home/grin/Desktop/Dados_B9/";
    std::string im_rgb = "camera_rgb.jpg", im_clu = "imagem_clusters.png", im_dep = "camera_virtual.jpg", im_dist = "distancias.png", im_nuvem = "nuvem_organizada.png";
    OtimizaImagens oi(pasta, im_rgb, im_clu, im_dep, im_dist, im_nuvem);

    /// Criar imagens com arestas resultantes
    ROS_INFO("Calculando arestas nas imagens ...");
    oi.calculateEdgesOnImages();

    /// Salvar imagens na pasta para olhar se tudo bem
    ROS_INFO("Salvando imagens na pasta %s ...", pasta.c_str());
    oi.saveEdgeImages();

    /// Obter features nas imagens com arestas
    ROS_INFO("Obtendo features e matches nas imagens com arestas ...");
//    oi.calcAndMatchFeatures();

    /// Movendo uma imagem sobre a outra com as setas
    ROS_INFO("Movendo uma imagem com arestas sobre a outra com o teclado");
    oi.adjustImagesKeyboard();

    /// Calcular relacao entre as imagens (matriz de transformação homogenea)
    ROS_INFO("Calculando relacao entre as imagens ...");

    /// Colorir nuvem com as cores certas e bora la
    ROS_INFO("Colorindo a nuvem de entrada com cores reais ...");

    ROS_INFO("Processo finalizado.");
    ros::spinOnce();

    return 0;
}
