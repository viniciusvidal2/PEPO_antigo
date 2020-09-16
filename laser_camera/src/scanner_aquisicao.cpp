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
#include <std_msgs/Bool.h>

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
#include "servos60kg/LED.h"

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
int contador_nuvem = 0, N = 150; // Quantas nuvens aquisitar em cada parcial
vector<int> posicoes_pan, posicoes_tilt; // Posicoes a serem vasculhadas pelos motores [DEGREES]
vector<float> tilts_tf; // Angulos convertidos entre valor do servo e frame do robo
// Inicia variaveis do motor PAN
float deg_min_pan = 0, deg_max_pan = 360;
// Inicia variaveis do motor TILT - horizontal da offset para ser o zero
float deg_min_tilt = 120, deg_hor_tilt = 90, deg_max_tilt = 45;
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
ros::ServiceClient comando_leds;
// Publisher para ver a nuvem filtrada que esta acumulando
ros::Publisher cloud_pub;
// Publisher para a odometria que estamos no momento
ros::Publisher odom_pub;
nav_msgs::Odometry odom_msg;
// Testando sincronizar subscribers por mutex
Mutex m;
// Flag sobre o estado dos drivers dos servos
bool drivers_iniciados = false;

/// Caminho controlado do comando total dos servos
///
void moveServoSteps(float pan0, float pan1, float tilt0, float tilt1){
  float step_pan = 1, step_tilt = 1; // passo maximo de movimento [DEGREES]
  // Iniciar comando de envio das coordenadas aos servos
  servos60kg::Position cmd;
  float ang_temp;
  /// Por seguranca, enviar primeiro pan e depois tilt
  if(abs(pan1 - pan0) > abs(step_pan)){
    step_pan = (pan1 > pan0) ? step_pan : -step_pan;
    ang_temp = pan0 + step_pan;
    ang_temp = ang_temp >=   0 ? ang_temp :   0;
    ang_temp = ang_temp <= 360 ? ang_temp : 360;
    cmd.request.pan  = ang_temp;
    cmd.request.tilt = tilt0;
    comando_servos.call(cmd);
    while(abs(pan1 - ang_temp) >= 0.5){
      ang_temp += step_pan;
      if(pan1 > pan0)
        ang_temp = (ang_temp > pan1) ? pan1 : ang_temp;
      else
        ang_temp = (ang_temp < pan1) ? pan1 : ang_temp;
      ang_temp = ang_temp >=   0 ? ang_temp :   0;
      ang_temp = ang_temp <= 360 ? ang_temp : 360;
      cmd.request.pan  = ang_temp;
      cmd.request.tilt = tilt0;
      comando_servos.call(cmd);
    }
  }
  
  if(abs(tilt1 - tilt0) > abs(step_tilt)){
    step_tilt = (tilt1 > tilt0) ? step_tilt : -step_tilt;
    ang_temp = tilt0 + step_tilt;
    cmd.request.pan  = pan1;
    cmd.request.tilt = ang_temp;
    comando_servos.call(cmd);
    while(abs(tilt1 - ang_temp) >= 0.5){
      ang_temp += step_tilt;
      if(tilt1 > tilt0)
        ang_temp = (ang_temp > tilt1) ? tilt1 : ang_temp;
      else
        ang_temp = (ang_temp < tilt1) ? tilt1 : ang_temp;
      ang_temp = ang_temp >=   0 ? ang_temp :   0;
      ang_temp = ang_temp <= 180 ? ang_temp : 180;
      cmd.request.pan  = pan1;
      cmd.request.tilt = ang_temp;
      comando_servos.call(cmd);
    }
  }

  sleep(2);
}

/// Callback drivers do motor
///
void motCallback(const std_msgs::BoolConstPtr& msg){
    // Sabemos aqui se os drivers dos servos ja foram iniciados no outro no e assim podemos iniciar todo o movimento
    drivers_iniciados = msg->data;
}

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
            omp_set_dynamic(0);
            #pragma omp parallel for num_threads(50)
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
            pass.setFilterLimits(0, 70); // Z metros de profundidade
            pass.filter(*cloud_color);
            // Colorir pontos com calibracao default para visualizacao rapida
            ROS_WARN("Colorindo nuvem para salvar com parametros default ...");
            PointCloud<PointT>::Ptr cloud_color_image (new PointCloud<PointT>());
            pc->colorCloudWithCalibratedImage(cloud_color, cloud_color_image, image_ptr->image, 1133.3, 1121.6, 2, 7); // Brio
//            pc->colorCloudWithCalibratedImage(cloud_color, cloud_color_image, image_ptr->image, 1496.701399, 1475.059238, 2, 9); // Logitech antiga
            // Filtrando por voxels e outliers - essa vai para visualizacao
            ROS_WARN("Filtrando nuvem ...");
            PointCloud<PointT>::Ptr cloud_filter (new PointCloud<PointT>());
            cloud_filter->header.frame_id = "pepo";
            VoxelGrid<PointT> voxel;
            voxel.setInputCloud(cloud_color_image);
            voxel.setLeafSize(0.01, 0.01, 0.01);
            voxel.filter(*cloud_filter);
//            StatisticalOutlierRemoval<PointT> sor;
//            sor.setInputCloud(cloud_filter);
//            sor.setMeanK(20);
//            sor.setStddevMulThresh(2);
//            sor.filter(*cloud_filter);
            // Transformar nuvem de acordo com a posicao dos servos
            float p = posicoes_pan[indice_posicao], t = tilts_tf[indice_posicao];
            Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
            Eigen::Vector3f C{0, 0, 0};
//            pc->transformCloudServoAngles(cloud_color_image, p, t, odom_msg, T, C);
            pc->transformCloudServoAngles(cloud_filter     , p, t, odom_msg, T, C);
            // Salvar dados parciais na pasta Dados_pepo, no Desktop
            ROS_WARN("Salvando dados de imagem e nuvem da aquisicao %d de %zu ...", indice_posicao+1, posicoes_pan.size());
            std::string nome_imagem_atual = "imagem_"+std::to_string(indice_posicao+1);
            pc->saveImage(image_ptr->image, nome_imagem_atual);
//            pc->saveCloud(cloud_color_image, "p_"+std::to_string(indice_posicao+1));
            pc->saveCloud(cloud_filter, "pf_"+std::to_string(indice_posicao+1));
            /// Adcionar ao vetor a linha correspondente do NVM ///
            Eigen::Matrix4f T_ = T.inverse(); // Inverte para escrever a camera;
            // Calcula o quaternion correspondente
            Eigen::Matrix3f rot_cam;
            rot_cam << T_.block<3,3>(0, 0);
            Eigen::Quaternion<float> q(rot_cam);
            // Escreve a linha e armazena
            linhas_nvm[indice_posicao] = pc->escreve_linha_imagem((2182.371971/2 + 2163.57254/2)/2, nome_imagem_atual, C, q); // Brio
//            linhas_nvm[indice_posicao] = pc->escreve_linha_imagem((1496.701399+1475.059238)/2, nome_imagem_atual, C, q); // Logitech antiga
            //////////////////////
            // Somar a nuvem filtrada aqui na acumulada filtrada
//            *acc_filt += *cloud_filter;
            // Adicionar nuvem original colorida no vetor de nuvens parciais -> melhor na acumulada de uma vez para memoria RAM
            //*acc += *cloud_filter; //*cloud_color_image;
            //parciais[indice_posicao] = *cloud_color_image;
            // Limpar nuvem parcial
            parcial->clear();
            cloud_color_image->clear();
            // Zerar contador de nuvens da parcial
            contador_nuvem = 0;
            // Terminamos o processamento, virar flag
            ROS_WARN("Terminada aquisicao da nuvem %d", indice_posicao+1);
            aquisitando = false;
            // Enviar os servos para a proxima posicao, se nao for a ultima ja
            servos60kg::Position cmd;
            if(indice_posicao + 1 < posicoes_pan.size()){
                indice_posicao++; // Proximo ponto de observacao
                ROS_INFO("Movendo servos para a proxima posicao de aquisicao ...");
                moveServoSteps(posicoes_pan[indice_posicao-1], posicoes_pan[indice_posicao], posicoes_tilt[indice_posicao-1], posicoes_tilt[indice_posicao]);
                ROS_INFO("Chegou na posicao %d de %zu totais para aquisitar nova nuvem ...", indice_posicao+1, posicoes_pan.size());
		sleep(2); // Para a camera calibrar
                aquisitando = true;
                aquisitar_imagem = true; // Garante que ja podemos aceitar novas imagens agora
            } else { // Se for a ultima, finalizar
                // Voltando para o inicio, pelo menos em tilt
                ROS_INFO("Movendo servos para a posicao final e finalizando ...");
                moveServoSteps(posicoes_pan[indice_posicao], posicoes_pan[0], posicoes_tilt[indice_posicao], posicoes_tilt[indice_posicao]);
                // Salvando o NVM final
                pc->compileFinalNVM(linhas_nvm);
                // Somando todas na acumulada cheia e salvando a acumulada
                ROS_WARN("Salvando nuvem final ...");
                //auto par = parciais.begin();
                //while(par != parciais.end()){
                //    *acc += *par;
                //    par = parciais.erase(par); // Assim vai limpando a memoria RAM enquanto acumula tudo para salvar
                //}
//                for(int i=0; i < parciais.size(); i++)
//                    *acc += parciais[i];
                //parciais.clear();
                //pc->saveCloud(acc, "acumulada_hd");
                //acc->clear();
                // Colocando LED de forma continua
                // Ligar o LED piscando rapido
                servos60kg::LED cmd_led;
                cmd_led.request.led = 1;
                comando_leds.call(cmd_led);
                // Finalizando o no e o ROS
                ROS_WARN("Nuvem final salva, conferir dados salvos.");
                ROS_WARN("Ao observar o LED em brilho continuo, foi finalizado o processo.");
                fim_processo = true;
            }
            m.unlock();
        } else {
            contador_nuvem++;
        }
    }
}

/// Main
///
int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanner_aquisicao");
    ros::NodeHandle nh;
    ros::NodeHandle n_("~");
    ROS_INFO("Iniciando o processo do SCANNER - aguardando servos ...");

    // Pegando o nome da pasta por parametro
    string nome_param;
    n_.param("pasta", nome_param, string("Dados_PEPO"));

    // Apagando pasta atual e recriando a mesma na area de trabalho
    char* home;
    home = getenv("HOME");
    std::string pasta = std::string(home)+"/Desktop/"+nome_param.c_str()+"/";
    system(("rm -r "+pasta).c_str());
    mkdir(pasta.c_str(), 0777);

    sleep(5); // Esperar os motores ligarem e ficarem prontos

    // Preenchendo os vetores de posicao a ser escaneadas
    int step = 20; // [degrees]
    vector<double> tilts = {135, 115, 95, 75, 55, 35};
    int vistas = int(deg_max_pan - deg_min_pan)/step + 1; // Vistas na horizontal
    posicoes_pan.resize(vistas * tilts.size());
    posicoes_tilt.resize(posicoes_pan.size());
    omp_set_dynamic(0);
    #pragma omp parallel for num_threads(tilts.size())
    for(int j=0; j < tilts.size(); j++){
        for(int i=0; i < vistas; i++){
            if(remainder(j, 2) == 0){ // Define assim um vai e volta no scanner, ao inves de voltar ao inicio
                posicoes_pan[i + j*vistas] = deg_min_pan + double(i*step);
            } else {
                posicoes_pan[i + j*vistas] = deg_max_pan - double(i*step);
            }
            posicoes_pan[i + j*vistas] = (posicoes_pan[i + j*vistas] >= 0  ) ? posicoes_pan[i + j*vistas] :   0;
            posicoes_pan[i + j*vistas] = (posicoes_pan[i + j*vistas] <= 360) ? posicoes_pan[i + j*vistas] : 360;
            posicoes_tilt[i + j*vistas] = tilts[j];
        }
    }

    // Converte os tilts para frame do robo - horizontal vale 0, cresce da vista de baixo para a de cima
    tilts_tf.resize(posicoes_tilt.size());
    for(int i=0; i < tilts_tf.size(); i++)
        tilts_tf[i] = deg_hor_tilt - double(posicoes_tilt[i]);

    // Inicia nuvem acumulada final e aloca espaco para a quantidade de parciais
    acc      = (PointCloud<PointT  >::Ptr) new PointCloud<PointT  >();
    acc_filt = (PointCloud<PointT  >::Ptr) new PointCloud<PointT  >();
    parcial  = (PointCloud<PointXYZ>::Ptr) new PointCloud<PointXYZ>();
    acc->header.frame_id      = "pepo";
    acc_filt->header.frame_id = "pepo";
    parcial->header.frame_id  = "pepo";
    parciais.resize(posicoes_pan.size());

    // Inicia o vetor de linhas do arquivo NVM
    linhas_nvm.resize(posicoes_pan.size());

    // Inicia servico para mexer os servos
    comando_servos = nh.serviceClient<servos60kg::Position>("/mover_servos");

    // Inicia servico para acionar o LED
    comando_leds = nh.serviceClient<servos60kg::LED>("/controle_led");

    // Subscriber para aguardar os drivers dos servos no outro no
    ros::Subscriber sub_mot = nh.subscribe("/servos60kg/inicio", 10, motCallback);
    ros::Rate rd(1);
    while(!drivers_iniciados){
         ROS_INFO("Aguardando drivers dos motores iniciarem ...");
         ros::spinOnce();
         rd.sleep();
    }

    // Ligar o LED piscando rapido
    servos60kg::LED cmd_led;
    cmd_led.request.led = 2;
    comando_leds.call(cmd_led);

    // Enviando scanner para o inicio    
    ROS_INFO("Servos comunicando e indo para a posicao inicial ...");
    servos60kg::Position cmd_servo;
    cmd_servo.request.pan  = posicoes_pan[0];
    cmd_servo.request.tilt = posicoes_tilt[0];
    comando_servos.call(cmd_servo);
    sleep(2); // Esperar os servos pararem de balancar
    aquisitando = true;
    aquisitar_imagem = true;
    ROS_INFO("Posicao inicial confirmada. Iniciando ...");

    // Inicia classe de processo de nuvens com o nome da pasta que sera tudo gravado
    pc = new ProcessCloud(pasta);

    // Publisher da nuvem para ver ao vivo
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/pepo/acc_filt", 100);

    // Publisher para odometria atual do robo
    odom_pub = nh.advertise<nav_msgs::Odometry>("pepo/odom", 100);
    odom_msg.header.frame_id = "pepo";
    odom_msg.child_frame_id  = "pepo";

    // Subscribers dessincronizados para mensagens de laser, imagem
    ros::Subscriber sub_laser  = nh.subscribe("/livox/lidar"      , 10, laserCallback );
    ros::Subscriber sub_cam    = nh.subscribe("/camera/image_raw", 10, camCallback   );

    ROS_INFO("Comecando a aquisicao ...");

    // Ligar o LED piscando lentamente para aquisitar
    cmd_led.request.led = 3;
    comando_leds.call(cmd_led);

    // Vai publicando as mensagens nos topicos
    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header.frame_id = "pepo";
    ros::Rate r(2);
    while(ros::ok()){
//        if(acc_filt->size() > 0){
//            toROSMsg(*acc_filt, cloud_msg);
//            cloud_msg.header.stamp = ros::Time::now();
//            odom_msg.header.stamp = cloud_msg.header.stamp;
//            cloud_pub.publish(cloud_msg);
//            odom_pub.publish(odom_msg);
//        }
        r.sleep();
        ros::spinOnce();

        if(fim_processo){
          ros::shutdown();
          break;
        }
    }

    return 0;
}
