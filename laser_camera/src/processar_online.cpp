/// Includes
///
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <math.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

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

#include "../../libraries/include/processcloud.h"

/// Namespaces
///
using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace std;

/// Definiçoes
///
typedef PointXYZRGB       PointT ;

/// Variaveis Globais
///
bool nuvem_pronta = false; // Monitora se ja acumulou o suficiente, para processar e travar o recebimento de imagens
bool fim_processo = false; //
cv_bridge::CvImagePtr image_ptr; // Ponteiro para imagem da camera

pcl::PointCloud<PointXYZ>::Ptr accumulated_cloud; // Nuvem de entrada acumulada

int contador_nuvem = 0, N = 400; // Controle de quantas nuvens absorvidas

ProcessCloud* pc; // Classe com todas as funcoes de trabalho na nuvem separadas e moduladas

ros::Time ini; // Medicao de tempo

/// Callback da camera
///
void camCallback(const sensor_msgs::ImageConstPtr& msg){
    // Se nao temos nuvem total, captura imagem
    if(!nuvem_pronta){
        // Aqui ja temos a imagem em ponteiro de opencv
        image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
}

/// Callback do laser
///
void laserCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // Ler a mensagem e acumular na nuvem total por N vezes
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>());
    pcl::fromROSMsg (*msg, *cloud);
    *accumulated_cloud += *cloud;
    // Se total acumulado, travar o resto e trabalhar
    if(contador_nuvem == N){
        // Vira a variavel de controle de recebimento de imagens
        ROS_WARN("Nuvem foi acumulada, processando ...");
        nuvem_pronta = true;
        // Injetando cor na nuvem
        PointCloud<PointT>::Ptr cloud_color (new PointCloud<PointT>());
        cloud_color->resize(accumulated_cloud->size());
#pragma omp parallel for
        for(size_t i=0; i < cloud_color->size(); i++){
            cloud_color->points[i].r = 0; cloud_color->points[i].g = 0; cloud_color->points[i].b = 0;
            cloud_color->points[i].x = accumulated_cloud->points[i].x;
            cloud_color->points[i].y = accumulated_cloud->points[i].y;
            cloud_color->points[i].z = accumulated_cloud->points[i].z;
        }
        // Transformando nuvem para o frame da camera
        pc->transformToCameraFrame(cloud_color);
        // Colorir pontos com calibracao default para visualizacao rapida
        ROS_WARN("Colorindo nuvem para salvar com parametros default ...");
        PointCloud<PointT>::Ptr cloud_color_image (new PointCloud<PointT>());
        pc->colorCloudWithCalibratedImage(cloud_color, cloud_color_image, image_ptr->image, 1496.701399, 1475.059238, 2, 9);
        // Filtrando por voxels e outliers - essa vai para visualizacao
        ROS_WARN("Filtrando nuvem ...");
        PointCloud<PointT>::Ptr cloud_filter (new PointCloud<PointT>());
        VoxelGrid<PointT> voxel;
        voxel.setInputCloud(cloud_color_image);
        voxel.setLeafSize(0.01, 0.01, 0.01);
        voxel.filter(*cloud_filter);
        StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(cloud_filter);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1);
        sor.filter(*cloud_filter);
        // Salvar dados na pasta Dados_B9, no Desktop
        ROS_WARN("Salvando dados na pasta Dados_B9 ...");
        pc->saveImage(image_ptr->image, "camera_rgb");
        pc->saveCloud(cloud_color_image, "nuvem_online");
        pc->saveCloud(cloud_filter, "nuvem_online_filtrada_rapida");
        // Terminamos o processamento, travar tudo
        ROS_WARN("Tudo terminado, conferir na pasta!");
        fim_processo = true;
        contador_nuvem++;
        ros::Time fin = ros::Time::now();
        ROS_WARN("Tempo gasto para o processo: %.2f", (fin-ini).toSec());
        system("gnome-terminal -x sh -c 'rosnode kill -a'");
    } else {
        contador_nuvem++;
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "processar_online");
  ros::NodeHandle nh;

  // Inicia medidor de tempo
  ini = ros::Time::now();

  // Inicia ponteiros das nuvens
  accumulated_cloud = (PointCloud<PointXYZ>::Ptr) new PointCloud<PointXYZ>;

  // Inicia classes
  pc = new ProcessCloud();

  // Subscribers dessincronizados para mensagens de laser e imagem
  // a principio so precisamos de uma imagem
  ros::Subscriber sub_laser = nh.subscribe("/livox/lidar"      , 100, laserCallback);
  ros::Subscriber sub_cam   = nh.subscribe("/usb_cam/image_raw", 100, camCallback  );

  // O no so funciona uma vez, depois e encerrado
  ros::Rate r(5);
  while(ros::ok()){
      ROS_INFO("................. Processando .................");
      r.sleep();
      ros::spinOnce();
      // Fim do processo cancela o no
      if(fim_processo)
          ros::shutdown();
  }

  return 0;

}
