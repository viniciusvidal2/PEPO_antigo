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
#include <pcl/filters/passthrough.h>
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
typedef PointXYZRGB PointT;

/// Variaveis Globais
///
cv_bridge::CvImagePtr image_ptr; // Ponteiro para imagem da camera
bool aquisitando = false, nuvem_acumulada = false, aquisitar_imagem = false, fim_processo = false;
int indice_posicao = 0; // Posicao do vetor de posicoes sendo observada no momento
int contador_nuvem = 0, N = 300; // Quantas nuvens aquisitar em cada parcial
vector<int> posicoes_pan, posicoes_tilt; // Posicoes a serem vasculhadas pelos motores
// Inicia variaveis do motor PAN
double raw_min_pan = 133, raw_max_pan = 3979;
double deg_min_pan =  11, deg_max_pan =  349;
double deg_raw_pan = 1, raw_deg_pan = 1; // Definidas no main
double dentro = 10; // Raio de seguranca que estamos dentro ja [RAW] (pelas contas aproximadas, 1 RAW = 0.08 degrees)
// Inicia variaveis do motor TILT - horizontal da offset para ser o zero
double raw_min_tilt = 1965, raw_hor_tilt = 2200, raw_max_tilt = 2300;
double deg_min_tilt =  172, deg_hor_tilt =  193, deg_max_tilt =  202;
double deg_raw_tilt, raw_deg_tilt; // Definidas no main
// Classe de processamento de nuvens
ProcessCloud *pc;
// Nuvens de pontos e vetor de nuvens parciais
PointCloud<PointXYZ>::Ptr parcial;
PointCloud<PointT  >::Ptr acc, acc_filt;
vector<PointCloud<PointT>> parciais;
// Vetor com linhas do arquivo NVM
vector<std::string> linhas_nvm;
// Servico para mover os servos
ros::ServiceClient comando_motor;
// Publisher para ver a nuvem filtrada que esta acumulando
ros::Publisher cloud_pub;

///////////////////////////////////////////////////////////////////////////////////////////
int deg2raw(double deg, std::string motor){
    if(motor == "pan"){
        return int((deg - deg_min_pan )*raw_deg_pan  + raw_min_pan);
    } else {
        return int((deg - deg_min_tilt)*raw_deg_tilt + raw_min_tilt - raw_hor_tilt);
    }
}
double raw2deg(int raw, std::string motor){
    if(motor == "pan"){
        return (double(raw) - raw_min_pan )*deg_raw_pan  + deg_min_pan;
    } else {
        return (double(raw) - raw_min_tilt)*deg_raw_tilt + deg_min_tilt - deg_hor_tilt;
    }
}
///////////////////////////////////////////////////////////////////////////////////////////

/// Callback da camera
///
void camCallback(const sensor_msgs::ImageConstPtr& msg){
    // Aqui ja temos a imagem em ponteiro de opencv, depois de pegar uma desabilitar
    if(aquisitar_imagem)
        image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
}

/// Callback do laser
///
void laserCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    if(aquisitando){
        // Ler a mensagem e acumular na nuvem total por N vezes
        PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>());
        fromROSMsg (*msg, *cloud);
        *parcial += *cloud;
        // A nuvem ainda nao foi acumulada, frizar isso
        aquisitar_imagem = true;//nuvem_acumulada = false;
        ROS_INFO("Acumulando nuvem %.2f pct ...", float(contador_nuvem)/float(N)*100);
        // Se total acumulado, travar o resto e trabalhar
        if(contador_nuvem == N){
            ROS_WARN("Nuvem %d foi acumulada, processando ...", indice_posicao+1);
            // Vira a variavel de controle de recebimento de imagens e da nuvem
            aquisitar_imagem = false; //nuvem_acumulada = true;
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
            // Filtrar profundidade pra nao vir aquilo tudo de coisa
            PassThrough<PointT> pass;
            pass.setInputCloud(cloud_color);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(0, 15); // Z metros de profundidade
            pass.filter(*cloud_color);
            // Colorir pontos com calibracao default para visualizacao rapida
            ROS_WARN("Colorindo nuvem para salvar com parametros default ...");
            PointCloud<PointT>::Ptr cloud_color_image (new PointCloud<PointT>());
            pc->colorCloudWithCalibratedImage(cloud_color, cloud_color_image, image_ptr->image, 1496.701399, 1475.059238, 2, 9);
            // Filtrando por voxels e outliers - essa vai para visualizacao
            ROS_WARN("Filtrando nuvem ...");
            PointCloud<PointT>::Ptr cloud_filter (new PointCloud<PointT>());
            cloud_filter->header.frame_id = "B9";
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
            ROS_WARN("Transformando nuvem segundo angulos dos servos: pan %.2f e tilt %.2f ...", raw2deg(posicoes_pan[indice_posicao], "pan"), raw2deg(posicoes_tilt[indice_posicao], "tilt"));
            pc->transformCloudServoAngles(cloud_color_image, raw2deg(posicoes_pan[indice_posicao], "pan"), raw2deg(posicoes_tilt[indice_posicao], "tilt"));
            pc->transformCloudServoAngles(cloud_filter     , raw2deg(posicoes_pan[indice_posicao], "pan"), raw2deg(posicoes_tilt[indice_posicao], "tilt"));
            // Salvar dados parciais na pasta Dados_B9, no Desktop
            ROS_WARN("Salvando dados de imagem e nuvem da aquisicao %d de %zu ...", indice_posicao+1, posicoes_pan.size());
            std::string nome_imagem_atual = "imagem_"+std::to_string(indice_posicao+1);
            pc->saveImage(image_ptr->image, nome_imagem_atual);
            pc->saveCloud(cloud_color_image, "p_"+std::to_string(indice_posicao+1));
            pc->saveCloud(cloud_filter, "pf_"+std::to_string(indice_posicao+1));
            /// Adcionar ao vetor a linha correspondente do NVM ///
            // Calcula o centro
            Eigen::Vector3f C {0, 0, 0};
            // Calcula o quaternion correspondente
            Eigen::AngleAxisf pitch(DEG2RAD(raw2deg(posicoes_tilt[indice_posicao], "tilt")), Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf yaw(  DEG2RAD(raw2deg(posicoes_pan[ indice_posicao], "pan" )), Eigen::Vector3f::UnitY());
            Eigen::Quaternion<float> q = yaw*pitch;
            // Escreve a linha e armazena
            linhas_nvm[indice_posicao] = pc->escreve_linha_imagem((1496.701399+1475.059238)/2, nome_imagem_atual, C, q.inverse());
            //////////////////////
            // Somar a nuvem filtrada aqui na acumulada filtrada
            *acc_filt += *cloud_filter;
            // Adicionar nuvem original colorida no vetor de nuvens parciais
            parciais[indice_posicao] = *cloud_color_image;
            // Limpar nuvem parcial
            parcial->clear();
            // Zerar contador de nuvens da parcial
            contador_nuvem = 0;
            // Terminamos o processamento, virar flag
            ROS_WARN("Terminada aquisicao da nuvem %d", indice_posicao+1);
            aquisitando = false;
            // Enviar os servos para a proxima posicao, se nao for a ultima ja
            dynamixel_workbench_msgs::JointCommand cmd;
            cmd.request.unit = "raw";
            if(indice_posicao + 1 < posicoes_pan.size()){
                indice_posicao++; // Proximo ponto de observacao
                cmd.request.pan_pos  = posicoes_pan[indice_posicao];
                cmd.request.tilt_pos = posicoes_tilt[indice_posicao];
                if(comando_motor.call(cmd))
                    ROS_INFO("Indo para a posicao %d de %zu totais aquisitar nova nuvem", indice_posicao+1, posicoes_pan.size());
            } else { // Se for a ultima, finalizar
                // Voltando para o inicio
                cmd.request.pan_pos  = raw_min_pan;
                cmd.request.tilt_pos = posicoes_tilt[indice_posicao]; // Vai deitar mesmo
                if(comando_motor.call(cmd))
                ROS_INFO("Aquisitamos todas as nuvens, salvando tudo e indo para a posicao inicial ...");
                // Somando todas na acumulada cheia e salvando a acumulada
                for(int i=0; i < parciais.size(); i++)
                    *acc += parciais[i];
                pc->saveCloud(acc, "acumulada_hd");
                // Salvando o NVM final
                pc->compileFinalNVM(linhas_nvm);
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
    int pan = int(msg->pose.pose.position.x), tilt = int(msg->pose.pose.position.y);
    // Se estiver perto do valor de posicao atual de gravacao, liberar a aquisicao
    if(abs(pan - posicoes_pan[indice_posicao]) <= dentro && abs(tilt - posicoes_tilt[indice_posicao]) <= dentro){
        aquisitando = true;
        posicoes_pan[indice_posicao] = pan; posicoes_tilt[indice_posicao] = tilt; // Para ficar mais preciso ainda na transformacao
    } else {
        aquisitando = false;
        ROS_INFO("Faltam %d e %d para proxima aquisicao ...", abs(pan - posicoes_pan[indice_posicao]), abs(tilt - posicoes_tilt[indice_posicao]));
    }
    // Se ja tiver no fim do processo, confere se esta chegando no inicio pra dai desligar os motores
    if(fim_processo){
        ROS_WARN("Estamos voltando ao inicio, %d para PAN e %d para TILT ...", int(abs(pan - raw_min_pan)), int(abs(tilt - raw_min_tilt)));
        if(abs(pan - raw_min_pan) <= dentro && abs(tilt - posicoes_tilt[indice_posicao]) <= dentro){
            ROS_WARN("Chegamos ao final, desligando ...");
            system("gnome-terminal -x sh -c 'rosnode kill -a'");
            ros::shutdown();
        }
    }
}

/// Main
///
int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanner_aquisicao");
    ros::NodeHandle nh;
    ROS_INFO("Iniciando o processo do SCANNER - aguardando servos ...");

    // Apagando pasta atual e recriando a mesma na area de trabalho
    char* home;
    home = getenv("HOME");
    std::string pasta = std::string(home)+"/Desktop/Dados_B9";
    system(("rm -r "+pasta).c_str());
    mkdir(pasta.c_str(), 0777);

    sleep(3); // Esperar os motores ligarem e ficarem prontos
    ros::Rate r(2);

    // Definindo as taxas raw - deg dos servos
    deg_raw_pan  = (deg_max_pan  - deg_min_pan ) / (raw_max_pan  - raw_min_pan ); raw_deg_pan  = 1/deg_raw_pan;
    deg_raw_tilt = (deg_max_tilt - deg_min_tilt) / (raw_max_tilt - raw_min_tilt); raw_deg_tilt = 1/deg_raw_tilt;

    // Preenchendo os vetores de posicao a ser escaneadas - A FAZER inserir posicoes de tilt aqui
    int step = 30; // [degrees]
    vector<double> tilts = {raw_hor_tilt, raw_max_tilt, raw_min_tilt}; // Tres tilts que vao rolar
    int vistas = int(deg_max_pan - deg_min_pan)/step + 1; // Vistas na horizontal
    posicoes_pan.resize(vistas * tilts.size());
    posicoes_tilt.resize(posicoes_pan.size());
    omp_set_dynamic(0);
    #pragma omp parallel for num_threads(tilts.size())
    for(int j=0; j < tilts.size(); j++){
        for(int i=0; i < vistas; i++){
            if(remainder(j, 2) == 0) // Define assim um vai e volta no scanner, ao inves de voltar ao inicio
                posicoes_pan[i + j*vistas] = deg2raw(deg_min_pan + double(i*step), "pan");
            else
                posicoes_pan[i + j*vistas] = deg2raw(deg_max_pan - double(i*step), "pan");
            posicoes_tilt[i + j*vistas] = int(tilts[j]);
//            cout << posicoes_pan[i + j*vistas] << "  " << posicoes_tilt[i + j*vistas] << endl;
        }
    }

    // Inicia nuvem acumulada final e aloca espaco para a quantidade de parciais
    acc      = (PointCloud<PointT  >::Ptr) new PointCloud<PointT  >();
    acc_filt = (PointCloud<PointT  >::Ptr) new PointCloud<PointT  >();
    parcial  = (PointCloud<PointXYZ>::Ptr) new PointCloud<PointXYZ>();
    acc->header.frame_id      = "B9";
    acc_filt->header.frame_id = "B9";
    parcial->header.frame_id  = "B9";
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

    while(!comando_motor.call(cmd)){
        ROS_INFO("Iniciando comunicacao com os servos ...");
        r.sleep();
    }
    ROS_INFO("Servos comunicando e indo para a posicao inicial ...");

    // Inicia classe de processo de nuvens
    pc = new ProcessCloud();

    // Subscribers dessincronizados para mensagens de laser, imagem e motores
    ros::Subscriber sub_laser = nh.subscribe("/livox/lidar"                    , 100, laserCallback);
    ros::Subscriber sub_cam   = nh.subscribe("/usb_cam/image_raw"              ,  10, camCallback  );
    ros::Subscriber sub_dyn   = nh.subscribe("/dynamixel_angulos_sincronizados",  10, dynCallback  );

    // Publisher da nuvem para ver ao vivo
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/B9/acc_filt", 100);

    ROS_INFO("Comecando a aquisicao ...");

    // O no so funciona uma vez, depois e encerrado
    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = "B9";
    while(ros::ok()){
        if(acc_filt->size() > 0){
            toROSMsg(*acc_filt, cloud_msg);
            cloud_msg.header.stamp = ros::Time::now();
            cloud_pub.publish(cloud_msg);
            r.sleep();
        }
        ros::spinOnce();
    }

    return 0;
}
