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
    loadPLYFile<PointTN>(std::string(home)+"/Desktop/Dados_B9/nuvem_final.ply", *inicial);

//    // Filtra por outliers
//    ROS_INFO("Filtrando por outliers ...");
//    StatisticalOutlierRemoval<PointTN> sor;
//    sor.setInputCloud(inicial);
//    sor.setMeanK(10);
//    sor.setStddevMulThresh(2.5);
//    sor.setNegative(false);
//    sor.filter(*filtrada);

//    ROS_INFO("Filtrando ruidos radiais ...");
//    pc.filterCloudDepthCovariance(filtrada, 50, 1.5);

//    ROS_INFO("Salvando a nuvem filtrada por covariancia ...");
//    savePLYFileASCII<PointTN>(std::string(home)+"/Desktop/Dados_B9/nuvem_filtrada_covariancia.ply", *filtrada);
    loadPLYFile<PointTN>(std::string(home)+"/Desktop/Dados_B9/nuvem_filtrada_covariancia.ply", *filtrada);

    // Projeta sobre imagem com parametros default para ajudar a separar clusters por cor
    ROS_INFO("Adicionando cor com parametros default ...");
    float fx = 1496.701399, fy = 1475.059238, tx = 2, ty = 9;
    Mat imagem = imread(std::string(home)+"/Desktop/Dados_B9/camera_rgb.png");
    PointCloud<PointTN>::Ptr temp_cor (new PointCloud<PointTN>);
    pc.colorCloudWithCalibratedImage(filtrada, temp_cor, imagem, fx, fy, tx, ty);
    *filtrada = *temp_cor;

    // Extrai um vetor de planos e retorna nuvem sem eles
    ROS_INFO("Obtendo planos na nuvem ...");
    cl.obtainPlanes(filtrada, vetor_planos, filtrada_sem_planos);
    cl.separateClustersByDistance(vetor_planos);
    ROS_INFO("Foram obtidos %zu planos.", vetor_planos.size());

    //////////// Aplicando polinomios sobre os planos ////////////
    omp_set_dynamic(0);
//    #pragma omp parallel for num_threads(vetor_planos.size())
    for(int i = 0; i < vetor_planos.size(); i++){
        // Nuvem atual
        PointCloud<PointTN>::Ptr plane (new PointCloud<PointTN>());
        *plane = vetor_planos[i];
        pcl::search::KdTree<PointT>::Ptr tree_xyzrgb (new pcl::search::KdTree<PointT>());
        // Separando nuvem em nuvem de pontos XYZ, nuvem XYZRGB e so as normais
        PointCloud<PointT>::Ptr cloudxyzrgb (new PointCloud<PointT>());
        cloudxyzrgb->resize(plane->size());
        ROS_INFO("Separando nuvem %d para processar ....", i+1);
        #pragma omp parallel for
        for(size_t i=0; i < plane->size(); i++){
            PointT t;
            t.x = plane->points[i].x; t.y = plane->points[i].y; t.z = plane->points[i].z;
            t.r = plane->points[i].r; t.g = plane->points[i].g; t.b = plane->points[i].b;
            cloudxyzrgb->points[i] = t;
        }
        // Passar filtro polinomial
        ROS_INFO("Aplicando filtro polinomial no plano %d ...", i+1);
        PointCloud<PointTN>::Ptr saida_poli (new PointCloud<PointTN>());
        MovingLeastSquares<PointT, PointTN> mls;
        mls.setComputeNormals(true);
        mls.setInputCloud(cloudxyzrgb);
        mls.setPolynomialOrder(5);
        mls.setSearchMethod(tree_xyzrgb);
        mls.setSearchRadius(0.1);
        mls.process(*saida_poli);
        pc.calculateNormals(saida_poli);
        vetor_planos[i] = *saida_poli;
        ROS_INFO("Plano %d filtrado.", i+1);
    }

    // Extrai clusters da nuvem de pontos que restou
    ROS_INFO("Obtendo clusters para o restante da nuvem ...");
    cl.extractClustersRegionGrowingRGB(filtrada_sem_planos, vetor_clusters);
//    cl.separateClustersByDistance(vetor_clusters);
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

    // Salva cada nuvem de clusters na pasta certa
    for(size_t i=0; i < vetor_planos.size(); i++)
        savePLYFileASCII<PointTN>(std::string(home)+"/Desktop/Dados_B9/Clusters/p_"+std::to_string(i+1)+".ply", vetor_planos[i]);
    for(size_t i=0; i < vetor_clusters.size(); i++)
        savePLYFileASCII<PointTN>(std::string(home)+"/Desktop/Dados_B9/Clusters/o_"+std::to_string(i+1)+".ply", vetor_clusters[i]);

    // Salva nuvem final
    ROS_INFO("Salvando nuvem final ...");
    savePLYFileASCII<PointTN>(std::string(home)+"/Desktop/Dados_B9/nuvem_clusters.ply", *final);

//    loadPLYFile<PointTN>(std::string(home)+"/Desktop/Dados_B9/nuvem_clusters.ply", *final);

    // Projeta na imagem virtual a nuvem inteira
    ROS_INFO("Projetando imagem da camera virtual ...");
    pc.createVirtualLaserImage(final, "imagem_clusters");

    ROS_INFO("Processo finalizado.");

    return 0;
}
