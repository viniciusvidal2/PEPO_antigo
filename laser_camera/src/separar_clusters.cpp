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
    ROS_INFO("Carregando a nuvem de pontos ...");
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
    ROS_INFO("Obtendo planos na nuvem ...");
    cl.obtainPlanes(inicial, vetor_planos, inicial_sem_planos);
    ROS_INFO("Foram obtidos %zu planos", vetor_planos.size());

    // Extrai clusters da nuvem de pontos que restou
    ROS_INFO("Obtendo clusters para o restante da nuvem ...");
    cl.extractClustersRegionGrowing(inicial_sem_planos, vetor_clusters);
    ROS_INFO("Foram obtidos %zu clusters", vetor_clusters.size());

    // Colore nuvem de pontos cada qual com sua cor aleatoria
    ROS_INFO("Colorindo planos ...");
    cl.colorCloud(vetor_planos  );
    ROS_INFO("Colorindo clusters ...");
    cl.colorCloud(vetor_clusters);

    // Acumula nuvem final
    ROS_INFO("Acumulando clusters apos processo ...");
    for(size_t i=0; i < vetor_planos.size(); i++)
        *final += vetor_planos[i];
    for(size_t i=0; i < vetor_clusters.size(); i++)
        *final += vetor_clusters[i];

    // Filtra por outliers novamente
//    ROS_INFO("Filtrando por outliers ...");
//    sor.setInputCloud(final);
//    sor.setMeanK(1);
//    sor.setStddevMulThresh(0.4);
//    sor.filter(*final);

    // Salva nuvem final
    ROS_INFO("Salvando nuvem final ...");
    savePLYFileASCII(std::string(home)+"/Desktop/Dados_B9/nuvem_clusters.ply", *final);

    // Projeta na imagem virtual a nuvem inteira
    pc.createVirtualLaserImage(final, "imagem_clusters");

    // Cria mesh de Poisson para cada cluster? Possivelmente

    return 0;
}
