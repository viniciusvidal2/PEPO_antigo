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
typedef PointXYZRGBNormal PointTN;

/// Variaveis Globais
///
bool nuvem_pronta = false; // Monitora se ja acumulou o suficiente, para processar e travar o recebimento de imagens
bool fim_processo = false; //
cv_bridge::CvImagePtr image_ptr; // Ponteiro para imagem da camera

pcl::PointCloud<PointT>::Ptr accumulated_cloud; // Nuvem de entrada acumulada

int contador_nuvem = 0, N = 400; // Controle de quantas nuvens absorvidas

ProcessCloud* pc; // Classe com todas as funcoes de trabalho na nuvem separadas e moduladas

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
    PointCloud<PointT>::Ptr cloud (new PointCloud<PointT>());
    pcl::fromROSMsg (*msg, *cloud);
    *accumulated_cloud += *cloud;
    // Se total acumulado, travar o resto e trabalhar
    if(contador_nuvem == N){
        // Vira a variavel de controle de recebimento de imagens
        ROS_WARN("Nuvem foi acumulada, processando ...");
        nuvem_pronta = true;
        // Calcular normais da nuvem voltadas para a origem
        ROS_WARN("Calculando normais da nuvem e ajustando sentido ...");
        PointCloud<PointTN>::Ptr cloud_normals (new PointCloud<PointTN>());
        pc->calculateNormals(accumulated_cloud, cloud_normals);
        // Colorir a nuvem segundo a distancia
        ROS_WARN("Colorindo nuvem segundo distancia do ponto ...");
        pc->colorCloudThroughDistance(cloud_normals);
        // Criar imagem projetando com mesma matriz da câmera
        ROS_WARN("Projetando sobre a imagem virtual do laser ...");
        pc->transformToCameraFrame(cloud_normals);
        pc->createVirtualLaserImage(cloud_normals, "imagem_virtual");
        // Salvar dados na pasta Dados_B9, no Desktop
        ROS_WARN("Salvando dados na pasta Dados_B9 ...");
        pc->saveImage(image_ptr->image, "camera_rgb");
        pc->saveCloud(cloud_normals);
        // Terminamos o processamento, travar tudo
        ROS_WARN("Tudo terminado, conferir na pasta!");
        fim_processo = true;
        contador_nuvem++;
        system("gnome-terminal -x sh -c 'rosnode kill -a'");
    } else {
        contador_nuvem++;
    }
}

/// Main
///
int main(int argc, char **argv)
{
    // Inicia no
    ros::init(argc, argv, "processar");
    ros::NodeHandle nh;

    // Inicia ponteiros das nuvens
    accumulated_cloud = (PointCloud<PointT>::Ptr) new PointCloud<PointT>;

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
