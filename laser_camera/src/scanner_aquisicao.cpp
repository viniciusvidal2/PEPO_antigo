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
#include <nav_msgs/Odometry.h>

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

#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <dynamixel_workbench_msgs/DynamixelInfo.h>
#include <dynamixel_workbench_msgs/JointCommand.h>
#include <dynamixel_workbench_msgs/DynamixelState.h>

#include "../../libraries/include/processcloud.h"

/// Namespaces
///
using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace std;

/// Definiçoes
///
typedef PointXYZRGB PointT ;

/// Variaveis Globais
///
cv_bridge::CvImagePtr image_ptr; // Ponteiro para imagem da camera
bool aquisitando = false, nuvem_acumulada = false, aquisitar_imagem = false, fim_processo = false;
int indice_posicao = 0; // Posicao do vetor de posicoes sendo observada no momento
int contador_nuvem = 0, N = 400; // Quantas nuvens aquisitar em cada parcial
vector<int> posicoes_pan, posicoes_tilt; // Posicoes a serem vasculhadas pelos motores
// Inicia variaveis do motor
double raw_min = 50, raw_max = 3988;
double deg_min =  4, deg_max =  350;
double raw_atual = 0; // Seguranca
double deg_raw = (deg_max - deg_min) / (raw_max - raw_min), raw_deg = 1.0 / deg_raw;
double dentro = 2;
// Classe de processamento de nuvens
ProcessCloud *pc;
// Nuvens de pontos e vetor de nuvens parciais
PointCloud<PointXYZ>::Ptr parcial;
PointCloud<PointT>::Ptr acc, acc_filt;
vector<PointCloud<PointT>> parciais;
// Vetor com linhas do arquivo NVM
vector<std::string> linhas_nvm;
// Servico para mover os servos
ros::ServiceClient comando_motor;

///////////////////////////////////////////////////////////////////////////////////////////
double deg2raw(double deg){
    return (deg - deg_min)*raw_deg + raw_min;
}
double raw2deg(double raw){
    return (raw - raw_min)*deg_raw + deg_min;
}
///////////////////////////////////////////////////////////////////////////////////////////

/// Callback da camera
///
void camCallback(const sensor_msgs::ImageConstPtr& msg){
    // Se nao temos nuvem total, captura imagem
    if(aquisitar_imagem){
        // Aqui ja temos a imagem em ponteiro de opencv, depois de pegar uma desabilitar
        image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        aquisitar_imagem = false;
    }
}

/// Callback do laser
///
void laserCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if(aquisitando){
        // Ler a mensagem e acumular na nuvem total por N vezes
        PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>());
        pcl::fromROSMsg (*msg, *cloud);
        *parcial += *cloud;
        // Se total acumulado, travar o resto e trabalhar
        if(contador_nuvem == N){
            ROS_WARN("Nuvem %d foi acumulada, processando ...", indice_posicao+1);
            // Vira a variavel de controle de recebimento de imagens
            aquisitar_imagem = true;
            // Injetando cor na nuvem
            PointCloud<PointT>::Ptr cloud_color (new PointCloud<PointT>());
            cloud_color->resize(parcial->size());
            #pragma omp parallel for
            for(size_t i=0; i < cloud_color->size(); i++){
                cloud_color->points[i].r = 0; cloud_color->points[i].g = 0; cloud_color->points[i].b = 0;
                cloud_color->points[i].x = parcial->points[i].x;
                cloud_color->points[i].y = parcial->points[i].y;
                cloud_color->points[i].z = parcial->points[i].z;
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
            sor.setMeanK(20);
            sor.setStddevMulThresh(2);
            sor.filter(*cloud_filter);
            // Transformar nuvem de acordo com a posicao dos servos
            ROS_WARN("Transformando nuvem segundo angulos dos servos ...");
            pc->transformCloudServoAngles(cloud_color_image, raw2deg(posicoes_pan[indice_posicao]), raw2deg(posicoes_tilt[indice_posicao]));
            pc->transformCloudServoAngles(cloud_filter     , raw2deg(posicoes_pan[indice_posicao]), raw2deg(posicoes_tilt[indice_posicao]));
            // Salvar dados parciais na pasta Dados_B9, no Desktop
            ROS_WARN("Salvando dados de imagem e nuvem da aquisicao %d de %d ...", indice_posicao+1, posicoes_pan.size());
            pc->saveImage(image_ptr->image, "imagem_"+std::to_string(indice_posicao+1));
            pc->saveCloud(cloud_color_image, "p_"+std::to_string(indice_posicao+1));
            pc->saveCloud(cloud_filter, "pf_"+std::to_string(indice_posicao+1));
            // Adcionar ao vetor a linha correspondente do NVM

            // Somar a nuvem filtrada aqui na acumulada filtrada
            *acc_filt += *cloud_filter;
            // Adicionar nuvem original colorida no vetor de nuvens parciais
            parciais[indice_posicao] = *cloud_color_image;
            // Limpar nuvem parcial
            parcial->clear();
            // Zerar contador de nuvens da parcial
            contador_nuvem = 0;
            // Terminamos o processamento, virar flags
            ROS_WARN("Terminada aquisicao da nuvem %d", indice_posicao+1);
            aquisitando = false; nuvem_acumulada = true;
            // Enviar os servos para a proxima posicao, se nao for a ultima ja
            dynamixel_workbench_msgs::JointCommand cmd;
            cmd.request.unit     = "raw";
            if(indice_posicao + 1 < posicoes_pan.size()){
                indice_posicao++;
                cmd.request.pan_pos  = posicoes_pan[indice_posicao];
                cmd.request.tilt_pos = posicoes_tilt[indice_posicao];
                if(comando_motor.call(cmd))
                    ROS_INFO("Indo para a posicao %d de %d totais aquisitar nova nuvem", indice_posicao, posicoes_pan.size());
            } else { // Se for a ultima, finalizar
                // Voltando para o inicio
                cmd.request.pan_pos  = posicoes_pan[0];
                cmd.request.tilt_pos = posicoes_tilt[0];
                if(comando_motor.call(cmd))
                ROS_INFO("Aquisitamos todas as nuvens, salvando tudo e indo para a posicao inicial ...");
                // Somando todas na acumulada cheia e salvando a acumulada
                for(int i=0; i < parciais.size(); i++)
                    *acc += parciais[i];
                pc->saveCloud(acc, "acumulada_hd");
                // Finalizando o no e o ROS
                fim_processo = true;
                ROS_WARN("Finalizando tudo, conferir dados salvos.");
            }
        } else {
            contador_nuvem++;
        }
    }
}

/// Subscriber dos servos
///
void dynCallback(const nav_msgs::OdometryConstPtr& msg){
    // As mensagens trazem angulos em unidade RAW
    double pan = msg->pose.pose.position.x, tilt = msg->pose.pose.position.y;
    // Se estiver perto do valor de posicao atual de gravacao, liberar a aquisicao
    if(abs(pan - posicoes_pan[indice_posicao]) < dentro){
        // Enquanto a nuvem nao estiver acumulada
        while(!nuvem_acumulada)
            aquisitando = true;
        aquisitando = false;
    }
}

/// Main
///
int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanner_aquisicao");
    ros::NodeHandle nh;

    // Preenchendo os vetores de posicao a ser escaneadas
    int step = 30; // [degrees]
    posicoes_pan.resize(int(deg_max - deg_min)/step); posicoes_tilt.resize(posicoes_pan.size());
    for(int i=deg_min; i <= deg_max; i=i+step){
        posicoes_pan[i]  = deg2raw(i);
        posicoes_tilt[i] = 0;
    }

    // Inicia nuvem acumulada final e aloca espaco para a quantidade de parciais
    acc      = (PointCloud<PointT  >::Ptr) new PointCloud<PointT  >();
    acc_filt = (PointCloud<PointT  >::Ptr) new PointCloud<PointT  >();
    parcial  = (PointCloud<PointXYZ>::Ptr) new PointCloud<PointXYZ>();
    parciais.resize(posicoes_pan.size());

    // Inicia o vetor de linhas do arquivo NVM
    linhas_nvm.resize(posicoes_pan.size());

    // Inicia servico para mexer os servos
    comando_motor = nh.serviceClient<dynamixel_workbench_msgs::JointCommand>("/joint_command");

    // Enviando scanner para o inicio
    dynamixel_workbench_msgs::JointCommand cmd;
    cmd.request.unit = "raw";
    cmd.request.pan_pos  = posicoes_pan[0];
    cmd.request.tilt_pos = posicoes_tilt[0];
    if(comando_motor.call(cmd))
        ROS_INFO("Enviando scanner para a posicao inicial ...");

    // Inicia classe de processo de nuvens
    pc = new ProcessCloud();

    // Subscribers dessincronizados para mensagens de laser, imagem e motores
    ros::Subscriber sub_laser = nh.subscribe("/livox/lidar"                    , 100, laserCallback);
    ros::Subscriber sub_cam   = nh.subscribe("/usb_cam/image_raw"              , 100, camCallback  );
    ros::Subscriber sub_dyn   = nh.subscribe("/dynamixel_angulos_sincronizados", 100, dynCallback  );

    // O no so funciona uma vez, depois e encerrado
    while(ros::ok()){
        ros::spinOnce();
        // Fim do processo cancela o no
        if(fim_processo)
            ros::shutdown();
    }

    return 0;
}
