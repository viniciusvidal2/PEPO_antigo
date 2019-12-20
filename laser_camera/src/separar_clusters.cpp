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
    PointCloud<PointTN>::Ptr inicial             (new PointCloud<PointTN>);
    PointCloud<PointTN>::Ptr filtrada            (new PointCloud<PointTN>);
    PointCloud<PointTN>::Ptr filtrada_sem_planos (new PointCloud<PointTN>);
    PointCloud<PointTN>::Ptr final               (new PointCloud<PointTN>);
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
    sor.setMeanK(50);
    sor.setStddevMulThresh(2);
    sor.setNegative(false);
    sor.filter(*filtrada);

    ROS_INFO("Filtrando ruidos radiais ...");
    pc.filterCloudDepthCovariance(filtrada, 50, 1.5);

    // Extrai um vetor de planos e retorna nuvem sem eles
    ROS_INFO("Obtendo planos na nuvem ...");
    cl.obtainPlanes(filtrada, vetor_planos, filtrada_sem_planos);
    cl.separateClustersByDistance(vetor_planos);
    ROS_INFO("Foram obtidos %zu planos.", vetor_planos.size());

    // Extrai clusters da nuvem de pontos que restou
    ROS_INFO("Obtendo clusters para o restante da nuvem ...");
    cl.extractClustersRegionGrowingRGB(filtrada_sem_planos, vetor_clusters);
    cl.separateClustersByDistance(vetor_clusters);
    ROS_INFO("Foram obtidos %zu clusters.", vetor_clusters.size());

    // Definindo paleta de cores de cada plano e cluster
    cl.setColorPallete(vetor_planos.size() + vetor_clusters.size());

    // Colore nuvem de pontos cada qual com sua cor selecionada da paleta
    PointCloud<PointTN>::Ptr temp (new PointCloud<PointTN>);
    ROS_INFO("Colorindo planos ...");
    for(size_t i=0; i < vetor_planos.size(); i++){
        *temp = vetor_planos[i];
        cl.colorCloud(temp, i);
        vetor_planos[i] = *temp;
    }
    ROS_INFO("Colorindo clusters ...");
    for(size_t i=0; i < vetor_clusters.size(); i++){
        *temp = vetor_clusters[i];
        cl.colorCloud(temp, vetor_planos.size()+i);
        vetor_clusters[i] = *temp;
    }

    // Acumula nuvem final
    ROS_INFO("Acumulando clusters apos processo ...");
    for(size_t i=0; i < vetor_planos.size(); i++)
        *final += vetor_planos[i];
    for(size_t i=0; i < vetor_clusters.size(); i++)
        *final += vetor_clusters[i];

    // Salva nuvem final
    ROS_INFO("Salvando nuvem final ...");
    savePLYFileASCII<PointTN>(std::string(home)+"/Desktop/Dados_B9/nuvem_clusters.ply", *final);

    // Projeta na imagem virtual a nuvem inteira
    pc.createVirtualLaserImage(final, "imagem_clusters");

    return 0;
}
