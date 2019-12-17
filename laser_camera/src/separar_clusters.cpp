/// Includes
///
#include <ros/ros.h>
#include <iostream>
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

#include "../../libraries/include/clusters.h"
#include "../../libraries/include/processcloud.h"

/// Definicoes e namespaces
///
using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace std;
typedef PointXYZRGB       PointT ;
typedef PointXYZRGBNormal PointTN;

/// Main
///
int main(int argc, char **argv)
{
    // Inicia no do
    ros::init(argc, argv, "separar_clusters");
    ros::NodeHandle nh;

    // Inicia variaveis
    PointCloud<PointTN>::Ptr inicial            (new PointCloud<PointTN>);
    PointCloud<PointTN>::Ptr inicial_sem_planos (new PointCloud<PointTN>);
    PointCloud<PointTN>::Ptr final              (new PointCloud<PointTN>);
    vector<PointCloud<PointTN>> vetor_planos;
    vector<PointCloud<PointTN>> vetor_clusters;
    Clusters cl;
    ProcessCloud pc;

    // Le nuvem de pontos
    char* home;
    home = getenv("HOME");
    loadPLYFile(std::string(home)+"/Desktop/Dados_B9/nuvem_final.ply", *inicial);

    // Filtra por outliers
    ROS_INFO("Filtrando por outliers ...");
    StatisticalOutlierRemoval<PointTN> sor;
    sor.setInputCloud(inicial);
    sor.setMeanK(1);
    sor.setStddevMulThresh(0.6);
    sor.filter(*inicial);

    // Extrai um vetor de planos e retorna nuvem sem eles
    cl.obtainPlanes(inicial, vetor_planos, inicial_sem_planos);

    // Extrai clusters da nuvem de pontos que restou
    cl.extractClusters(inicial_sem_planos, vetor_clusters);

    // Colore nuvem de pontos cada qual com sua cor aleatoria
    cl.colorClouds(vetor_planos);
    cl.colorClouds(vetor_clusters);

    // Salva nuvem final
    for(size_t i=0; i < vetor_planos.size(); i++)
        *final += vetor_planos[i];
    for(size_t i=0; i < vetor_clusters.size(); i++)
        *final += vetor_clusters[i];
    savePLYFileASCII(std::string(home)+"/Desktop/Dados_B9/nuvem_clusters.ply", *final);

    // Projeta na imagem virtual a nuvem inteira
    pc.createVirtualLaserImage(final);

    // Cria mesh de Poisson para cada cluster? Possivelmente
}
