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
#include <geometry_msgs/Twist.h>

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

#include "../../libraries/include/processcloud.h"
#include "servos60kg/Position.h"

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
bool aquisitando = false, transicao = false, aquisitar_imagem = false, fim_processo = false;
int indice_posicao = 0; // Posicao do vetor de posicoes sendo observada no momento
int contador_nuvem = 0, N = 100; // Quantas nuvens aquisitar em cada parcial
vector<int> posicoes_pan, posicoes_tilt; // Posicoes a serem vasculhadas pelos motores [DEGREES]
// Inicia variaveis do motor PAN
double deg_min_pan = 0, deg_max_pan = 360;
// Inicia variaveis do motor TILT - horizontal da offset para ser o zero
double deg_min_tilt = 0, deg_hor_tilt = 45, deg_max_tilt = 90; // AJUSTAR !!
// Raio de seguranca que estamos dentro ja [DEGREES]
double dentro = 2;
// Classe de processamento de nuvens
ProcessCloud *pc;
// Nuvens de pontos e vetor de nuvens parciais
PointCloud<PointXYZ>::Ptr parcial;
PointCloud<PointT  >::Ptr acc, acc_filt;
vector<PointCloud<PointT>> parciais;
// Vetor com linhas do arquivo NVM
vector<std::string> linhas_nvm;
// Servico para mover os servos
ros::ServiceClient comando_servos;
// Publisher para ver a nuvem filtrada que esta acumulando
ros::Publisher cloud_pub;
// Publisher para a odometria que estamos no momento
ros::Publisher odom_pub;
nav_msgs::Odometry odom_msg;
// Testando sincronizar subscribers por mutex
Mutex m;

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
        aquisitar_imagem = true;
        // Se total acumulado, travar o resto e trabalhar
        if(contador_nuvem == N){
            m.lock();
            ROS_WARN("Nuvem %d foi acumulada, processando ...", indice_posicao+1);
            // Vira a variavel de controle de recebimento de imagens e da nuvem
            aquisitar_imagem = false;
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
            pass.setFilterLimits(0, 55); // Z metros de profundidade
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
            float p = posicoes_pan[indice_posicao], t = posicoes_tilt[indice_posicao];
            Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
            Eigen::Vector3f C;
            pc->transformCloudServoAngles(cloud_color_image, p, t, odom_msg, T, C);
            pc->transformCloudServoAngles(cloud_filter     , p, t, odom_msg, T, C);
            // Salvar dados parciais na pasta Dados_B9, no Desktop
            ROS_WARN("Salvando dados de imagem e nuvem da aquisicao %d de %zu ...", indice_posicao+1, posicoes_pan.size());
            std::string nome_imagem_atual = "imagem_"+std::to_string(indice_posicao+1);
            pc->saveImage(image_ptr->image, nome_imagem_atual);
            pc->saveCloud(cloud_color_image, "p_"+std::to_string(indice_posicao+1));
            pc->saveCloud(cloud_filter, "pf_"+std::to_string(indice_posicao+1));
            /// Adcionar ao vetor a linha correspondente do NVM ///
            Eigen::Matrix4f T_ = T.inverse(); // Inverte para escrever a camera;
            // Calcula o quaternion correspondente
            Eigen::Matrix3f rot_cam;
            rot_cam << T_.block<3,3>(0, 0);
            Eigen::Quaternion<float> q(rot_cam);
            // Calcula o centro
//            Eigen::MatrixXf C(3, 1);
//            C << -rot_cam*T.block<3,1>(0, 3);
            cout << "\nCentro oficial:\n" <<  -C.transpose() << endl << endl;
            // Escreve a linha e armazena
            linhas_nvm[indice_posicao] = pc->escreve_linha_imagem((1496.701399+1475.059238)/2, nome_imagem_atual, C, q);
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
            servos60kg::Position cmd;
            if(indice_posicao + 1 < posicoes_pan.size()){
                indice_posicao++; // Proximo ponto de observacao
                cmd.request.pan  = posicoes_pan[indice_posicao];
                cmd.request.tilt = posicoes_tilt[indice_posicao];
                if(comando_servos.call(cmd) == 1)
                    ROS_INFO("Indo para a posicao %d de %zu totais aquisitar nova nuvem", indice_posicao+1, posicoes_pan.size());
                if(posicoes_tilt[indice_posicao] != posicoes_tilt[indice_posicao-1])
                    sleep(2); // Seguranca para nao capturar nada balancando
                sleep(6); // Precisa segurar porque nao conseguimos ainda ler o angulo
            } else { // Se for a ultima, finalizar
                // Voltando para o inicio
                cmd.request.pan  = posicoes_pan[indice_posicao]; // Onde terminou e onde deve parar de uma vez
                cmd.request.tilt = deg_min_tilt; // Vai deitar mesmo
                if(comando_servos.call(cmd) == 1)
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
            m.unlock();
        } else {
            contador_nuvem++;
        }
    }
}

/// Subscriber dos servos
///
void servosCallback(const geometry_msgs::TwistConstPtr& msg){
    // As mensagens trazem angulos em unidade ja de DEGREES
    float pan = msg->linear.x, tilt = msg->linear.y;
    // Se estiver perto do valor de posicao atual de gravacao, liberar a aquisicao
    if(abs(pan - posicoes_pan[indice_posicao]) <= dentro && abs(tilt - posicoes_tilt[indice_posicao]) <= dentro){
        aquisitando = true;
        posicoes_pan[indice_posicao] = pan; posicoes_tilt[indice_posicao] = tilt; // Para ficar mais preciso ainda na transformacao
    } else {
        aquisitando = false;
        ROS_INFO("Aguardando proxima posicao de aquisicao ...");
    }
    // Se ja tiver no fim do processo, confere se esta chegando no inicio pra dai desligar os motores
    if(fim_processo){
        ROS_WARN("Estamos voltando ao inicio ...");
        if(abs(pan - deg_min_pan) <= dentro && abs(tilt - deg_min_tilt) <= dentro){
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

    // Preenchendo os vetores de posicao a ser escaneadas
    int step = 15; // [degrees]
    vector<double> tilts = {deg_min_tilt, deg_hor_tilt, deg_max_tilt};
    int vistas = int(deg_max_pan - deg_min_pan)/step + 1; // Vistas na horizontal
    posicoes_pan.resize(vistas * tilts.size());
    posicoes_tilt.resize(posicoes_pan.size());
    omp_set_dynamic(0);
    #pragma omp parallel for num_threads(tilts.size())
    for(int j=0; j < tilts.size(); j++){
        for(int i=0; i < vistas; i++){
            if(remainder(j, 2) == 0) // Define assim um vai e volta no scanner, ao inves de voltar ao inicio
                posicoes_pan[i + j*vistas] = deg_min_pan + double(i*step);
            else
                posicoes_pan[i + j*vistas] = deg_max_pan - double(i*step);
            posicoes_tilt[i + j*vistas] = tilts[j];
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
    comando_servos = nh.serviceClient<servos60kg::Position>("/mover_servos");

    // Enviando scanner para o inicio
    servos60kg::Position cmd;
    cmd.request.pan = 0;
    cmd.request.tilt = 0;

    while(!comando_servos.call(cmd)){
        ROS_ERROR("Falha na comunicacao com os servos !! ...");
        r.sleep();
    }
    ROS_INFO("Servos comunicando e indo para a posicao inicial ...");
    sleep(10); // Esperar os servos pararem de balancar

    // Inicia classe de processo de nuvens
    pc = new ProcessCloud();

    // Publisher da nuvem para ver ao vivo
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/B9/acc_filt", 100);

    // Publisher para odometria atual do robo
    odom_pub  = nh.advertise<nav_msgs::Odometry>("B9/odom", 100);
    odom_msg.header.frame_id = "B9";
    odom_msg.child_frame_id  = "B9";

    // Subscribers dessincronizados para mensagens de laser, imagem e motores
    ros::Subscriber sub_laser  = nh.subscribe("/livox/lidar"      , 10, laserCallback );
    ros::Subscriber sub_cam    = nh.subscribe("/usb_cam/image_raw", 10, camCallback   );
    ros::Subscriber sub_servos = nh.subscribe("/servos60kg/estado",  1, servosCallback);

    ROS_INFO("Comecando a aquisicao ...");

    // Vai publicando as mensagens nos topicos
    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = "B9";
    while(ros::ok()){
        if(acc_filt->size() > 0){
            toROSMsg(*acc_filt, cloud_msg);
            cloud_msg.header.stamp = ros::Time::now();
            odom_msg.header.stamp = cloud_msg.header.stamp;
            cloud_pub.publish(cloud_msg);
            odom_pub.publish(odom_msg);
            r.sleep();
        }
        ros::spinOnce();
    }

    return 0;
}
