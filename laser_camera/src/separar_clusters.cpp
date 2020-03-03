/// Includes
///
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <math.h>
#include <sys/stat.h>

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
    // Inicia no
    ros::init(argc, argv, "separar_clusters");
    ros::NodeHandle nh;

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    // Inicia variaveis
    PointCloud<PointTN>::Ptr inicial             (new PointCloud<PointTN>);
    PointCloud<PointTN>::Ptr filtrada            (new PointCloud<PointTN>);
    PointCloud<PointTN>::Ptr filtrada_sem_planos (new PointCloud<PointTN>);
    PointCloud<PointTN>::Ptr final               (new PointCloud<PointTN>);    
    PointCloud<PointTN>::Ptr projetar            (new PointCloud<PointTN>);
    vector<PointCloud<PointTN>> vetor_planos  , vetor_planos_filt  ;
    vector<PointCloud<PointTN>> vetor_clusters, vetor_clusters_filt;
    Clusters cl;
    ProcessCloud pc;

    // Define ambiente para ver o tratamento
    int ambiente = 2; // 1 para interno, 2 para externo

    // Le nuvem de pontos
    ROS_INFO("Carregando a nuvem de pontos ...");
    char* home;
    home = getenv("HOME");
    loadPLYFile<PointTN>(std::string(home)+"/Desktop/Dados_B9/acumulada_hd.ply", *inicial);

    // Filtra por outliers
    struct stat buffer;
    std::string nuvem_covariancia_nome = std::string(home)+"/Desktop/Dados_B9/nuvem_filtrada_covariancia.ply";
    if(stat(nuvem_covariancia_nome.c_str(), &buffer) && ambiente == 1){ // Se nao existe o arquivo, calcular somente para interno
        ROS_INFO("Filtrando por outliers ...");
        StatisticalOutlierRemoval<PointTN> sor;
        sor.setInputCloud(inicial);
        sor.setMeanK(10);
        sor.setStddevMulThresh(2.5);
        sor.setNegative(false);
        sor.filter(*filtrada);

        ROS_INFO("Filtrando ruidos radiais ...");
        pc.filterCloudDepthCovariance(filtrada, 50, 1.5);

        ROS_INFO("Salvando a nuvem filtrada por covariancia ...");
        savePLYFileASCII<PointTN>(nuvem_covariancia_nome, *filtrada);
    } else if(ambiente == 1) { // Se o arquivo ja existe, carregar somente
        ROS_INFO("Carregando a nuvem filtrada por covariancia ...");
        loadPLYFile<PointTN>(nuvem_covariancia_nome, *filtrada);
    }
    if(ambiente == 2){ // Se ambiente externo, so tirar mesmo alguns ruidos
        ROS_INFO("Ambiente externo, filtrando somente por outliers ...");
        *filtrada = *inicial;
    }

    // Projeta sobre imagem com parametros default para ajudar a separar clusters por cor
//    ROS_INFO("Adicionando cor com parametros default ...");
//    float fx = 1496.701399, fy = 1475.059238, tx = 2, ty = 9;
//    Mat imagem = imread(std::string(home)+"/Desktop/Dados_B9/camera_rgb.png");
//    PointCloud<PointTN>::Ptr temp_cor (new PointCloud<PointTN>);
//    pc.colorCloudWithCalibratedImage(filtrada, temp_cor, imagem, fx, fy, tx, ty);
//    *filtrada = *temp_cor;

    // Extrai um vetor de planos e retorna nuvem sem eles
    ROS_INFO("Obtendo planos na nuvem ...");
    cl.obtainPlanes(filtrada, vetor_planos, filtrada_sem_planos);
//    cl.separateClustersByDistance(vetor_planos);
    cl.killSmallClusters(vetor_planos, 1);
    ROS_INFO("Foram obtidos %zu planos apos filtragem.", vetor_planos.size());
    vetor_planos_filt = vetor_planos;

    // Aplicando polinomios sobre os planos
    ROS_INFO("Filtrando por polinomio os planos ...");
    pc.applyPolinomialFilter(vetor_planos_filt, 3, 0.1);

    // Extrai clusters da nuvem de pontos que restou
    ROS_INFO("Obtendo clusters para o restante da nuvem ...");
    cl.extractClustersRegionGrowingRGB(filtrada_sem_planos, vetor_clusters);
//    cl.separateClustersByDistance(vetor_clusters);
    cl.killSmallClusters(vetor_clusters, 1);
    ROS_INFO("Foram obtidos %zu clusters apos filtragem.", vetor_clusters.size());
    vetor_clusters_filt = vetor_clusters;

    // Aplicando polinomio sobre clusters
    ROS_INFO("Filtrando por polinomio os clusters ...");
    pc.applyPolinomialFilter(vetor_clusters_filt, 5, 0.15);

    // Definindo paleta de cores de cada plano e cluster
    cl.setColorPallete(vetor_planos.size() + vetor_clusters.size());

    // Colore nuvem de pontos cada qual com sua cor selecionada da paleta
    ROS_INFO("Colorindo planos ...");
    omp_set_dynamic(0);
    #pragma omp parallel for num_threads(vetor_planos.size())
    for(size_t i=0; i < vetor_planos.size(); i++){
        PointCloud<PointTN>::Ptr temp (new PointCloud<PointTN>);
        *temp = vetor_planos[i];
        cl.colorCloud(temp, i);
        vetor_planos[i] = *temp;
        temp->clear();
        *temp = vetor_planos_filt[i];
        cl.colorCloud(temp, i);
        vetor_planos_filt[i] = *temp;
    }
    ROS_INFO("Colorindo clusters ...");
    omp_set_dynamic(0);
    #pragma omp parallel for num_threads(vetor_planos.size())
    for(size_t i=0; i < vetor_clusters.size(); i++){
        PointCloud<PointTN>::Ptr temp (new PointCloud<PointTN>);
        *temp = vetor_clusters[i];
        cl.colorCloud(temp, vetor_planos.size()+i);
        vetor_clusters[i] = *temp;
        temp->clear();
        *temp = vetor_clusters_filt[i];
        cl.colorCloud(temp, vetor_planos.size()+i);
        vetor_clusters_filt[i] = *temp;
    }

    // Acumula nuvem final
    ROS_INFO("Acumulando clusters apos processo ...");
    for(size_t i=0; i < vetor_planos.size(); i++){
        *final += vetor_planos_filt[i];
        *projetar += vetor_planos[i];
    }
    for(size_t i=0; i < vetor_clusters.size(); i++){
        *final += vetor_clusters_filt[i];
        *projetar += vetor_clusters[i];
    }

    // Salva cada nuvem de clusters na pasta certa
    std::string pasta_cluters = std::string(home)+"/Desktop/Dados_B9/Clusters";
    system(("rm -r "+pasta_cluters).c_str());
    if(stat(pasta_cluters.c_str(), &buffer))
        mkdir(pasta_cluters.c_str(), 0777);
    for(size_t i=0; i < vetor_planos_filt.size()  ; i++)
        savePLYFileASCII<PointTN>(pasta_cluters+"/p_"+std::to_string(i+1)+".ply", vetor_planos_filt[i]);
    for(size_t i=0; i < vetor_clusters_filt.size(); i++)
        savePLYFileASCII<PointTN>(pasta_cluters+"/o_"+std::to_string(i+1)+".ply", vetor_clusters_filt[i]);

    // Salva nuvem final
    ROS_INFO("Salvando nuvem final ...");
    savePLYFileASCII<PointTN>(std::string(home)+"/Desktop/Dados_B9/nuvem_clusters_filtrada.ply", *final   );
    savePLYFileASCII<PointTN>(std::string(home)+"/Desktop/Dados_B9/nuvem_clusters.ply"         , *projetar);

    // Projeta na imagem virtual a nuvem inteira
    ROS_INFO("Projetando imagem da camera virtual ...");
    pc.createVirtualLaserImage(projetar, "imagem_clusters");

    ROS_INFO("Processo finalizado.");

    return 0;
}
