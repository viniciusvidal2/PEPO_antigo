/// Includes
///
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <math.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree.h>

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

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "../../libraries/include/processcloud.h"
#include "../../libraries/include/registerobjectoptm.h"

/// Namespaces
///
using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace std;
using namespace message_filters;

/// Definiçoes
///
typedef PointXYZRGB       PointT ;
typedef PointXYZRGBNormal PointTN;
typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;

/// Variaveis globais
///
// Limites em raw e deg para os servos de pan e tilt
double raw_min_pan = 35, raw_max_pan = 4077;
double deg_min_pan =  3, deg_max_pan =  358;
float raw_min_tilt = 2595.0 , raw_hor_tilt = 2280.0, raw_max_tilt = 1595.0 ;
float deg_min_tilt =   27.68, deg_hor_tilt =    0.0, deg_max_tilt =  -60.20;
float raw_deg_pan, deg_raw_pan, raw_deg_tilt, deg_raw_tilt;
// Publicador de imagem, nuvem parcial e odometria
ros::Publisher cl_pub;
ros::Publisher od_pub;
// Classe de processamento de nuvens
ProcessCloud *pc;
RegisterObjectOptm *roo;
// Nuvens de pontos acumulada e anterior
PointCloud<PointTN>::Ptr acc;
PointCloud<PointTN>::Ptr anterior;
PointCloud<PointTN>::Ptr parcial;
PointCloud<PointTN>::Ptr parcial_esq_anterior; // Parte esquerda ja filtrada e armazenada para otimizar o algoritmo
// Valor do servo naquele instante em variavel global para ser acessado em varios callbacks
int pan, tilt;
// Nome da pasta que vamos trabalhar
string pasta;
// Poses das cameras para aquela aquisicao [DEG]
vector<float> pan_cameras, tilt_cameras;
// Transformacoes otimizadas por ICP para cada vista pan
vector<Matrix4f> otimizacoes_icp;
// Vetor com linhas do arquivo NVM
vector<string> linhas_sfm;
// Quantos tilts estao ocorrendo por pan, e contador de quantos ja ocorreram
int ntilts = 4, contador_nuvens = 0;
// Parametros para filtros
float voxel_size, depth;
int filter_poli;
// Braco do centro ao laser
Vector3f off_laser{0, 0, 0.04};

///////////////////////////////////////////////////////////////////////////////////////////
int deg2raw(float deg, string motor){
    if(motor == "pan")
        return int(deg*raw_deg_pan);// int((deg - deg_min_pan )*raw_deg_pan  + raw_min_pan);
    else
        return int((deg - deg_min_tilt)*raw_deg_tilt + raw_min_tilt);
}
float raw2deg(int raw, string motor){
    if(motor == "pan")
        return float(raw*deg_raw_pan);// (float(raw) - raw_min_pan )*deg_raw_pan  + deg_min_pan;
    else
        return (float(raw) - raw_max_tilt)*deg_raw_tilt + deg_max_tilt;
}
///////////////////////////////////////////////////////////////////////////////////////////

/// Callback das poses
///
void posesCallback(const nav_msgs::OdometryConstPtr& msg){
    // As mensagens trazem angulos em unidade RAW - salvar em DEG
    pan_cameras.push_back(raw2deg(int(msg->pose.pose.position.x), "pan" ));
    tilt_cameras.push_back(raw2deg(int(msg->pose.pose.position.y), "tilt"));
    ROS_INFO("Poses recebidas: %zu.", pan_cameras.size());
    // Atualiza a quantidade de tilts que estamos esperando
    ntilts = int(msg->pose.pose.orientation.x);
}

/// Callback do laser e odometria sincronizado
///
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg_cloud, const nav_msgs::OdometryConstPtr& msg_angle){
    //cout << "Tempo para a transmissao da nuvem entre edge e fog:  " << ros::Time::now() - msg_cloud->header.stamp << endl;
    ros::Time tempo = ros::Time::now();
    // As mensagens trazem angulos em unidade RAW
    pan = int(msg_angle->pose.pose.position.x), tilt = int(msg_angle->pose.pose.position.y);
    int cont_aquisicao = msg_angle->pose.pose.position.z + 1;

    // Atualiza contagem de nuvens que chegaram na vista parcial de pan
    contador_nuvens++;

    // Converter mensagem em nuvem
    PointCloud<PointT>::Ptr cloud (new PointCloud<PointT>);
    fromROSMsg(*msg_cloud, *cloud);
    size_t tamanho_chegada = cloud->size();
    ROS_INFO("Recebendo nuvem %d com %zu pontos, contador %d e filtrando ...", cont_aquisicao, cloud->size(), contador_nuvens);
    // Realizar pre-processamento
    PointCloud<PointTN>::Ptr cloud_normals (new PointCloud<PointTN>);
    pc->preprocess(cloud, cloud_normals, voxel_size/100.0f, depth, filter_poli);
    cout << "Tempo para PREPROCESSAR: " << ros::Time::now() - tempo << endl;
    tempo = ros::Time::now();
    cloud->clear();

    // Adiciona somente uma parcial daquela vista em pan
    *parcial += *cloud_normals;

    if(contador_nuvens == ntilts){

        // Transformacao inicial, antes de sofrer otimizacao, devido aos angulos de servo em PAN
        Matrix3f R = pc->euler2matrix(0, 0, -DEG2RAD(raw2deg(pan, "pan") - 0*int(cont_aquisicao/ntilts)));
        Matrix4f Tini = Matrix4f::Identity();
        Tini.block<3,3>(0, 0) = R;
        Tini.block<3,1>(0, 3) = R*off_laser;
        transformPointCloudWithNormals<PointTN>(*parcial, *parcial, Tini);

        ///// ADICIONANDO CENTRO DA CAMERA EM PAN NA NUVEM PARA TESTE
//        PointTN pteste;
//        pteste.x = off(0); pteste.y = off(1); pteste.z = off(2);
//        pteste.r =     0 ; pteste.g =    250; pteste.b =     0 ;
//        parcial->push_back(pteste);

        // Filtro de regioes da nuvem
        PassThrough<PointTN> pass;

        ////////////////
        // Se for primeira nuvem, guardar e iniciar
        ROS_INFO("Acumulando parcial com %zu pontos ...", parcial->size());
        size_t tamanho_filtrada = parcial->size();
        PointCloud<PointTN>::Ptr parcial_pontos_novos (new PointCloud<PointTN>);
        *parcial_pontos_novos = *parcial;
        if(acc->size() < 10){
            // Inicia nuvem acumulada
            *acc = *parcial;
            *anterior = *parcial;
//            // Inicia na origem a otimizacao das nuvens e cameras
//            for(int i=0; i<ntilts; i++) otimizacoes_icp.emplace_back(Matrix4f::Identity());
        } else {
//            // Separar intersecao das nuvens para usar no ICP
//            PointCloud<PointTN>::Ptr anterior_icp (new PointCloud<PointTN>);
//            PointCloud<PointTN>::Ptr cloud_icp    (new PointCloud<PointTN>);
//            if(abs(cont_aquisicao - msg_angle->pose.pose.orientation.w) > ntilts)
//                *anterior_icp = *parcial_esq_anterior;
//            else
//                *anterior_icp = *acc;
//            *cloud_icp = *parcial;
//            ROS_INFO("Filtrando area comum ...");
//            transformPointCloudWithNormals(*cloud_icp, *cloud_icp, Tini.inverse());
//            pass.setFilterFieldName("x");
//            pass.setFilterLimits(0, 100); // Negativo de tudo no eixo X
//            pass.setNegative(false);
//            pass.setInputCloud(cloud_icp);
//            pass.filter(*cloud_icp);
//            transformPointCloudWithNormals(*cloud_icp, *cloud_icp, Tini);
//            cout << "Tempo para AREA COMUM: " << ros::Time::now() - tempo << endl;
//            tempo = ros::Time::now();
//            if(anterior_icp->size() > 200 && cloud_icp->size() > 200){
//                // Aproximar nuvem por ICP da nuvem inicial
//                ROS_INFO("Refinando registro por ICP ...");
//                Matrix4f Ticp = roo->icp(anterior_icp, cloud_icp, 0.03, 15);
//                cout << "Tempo para ICP: " << ros::Time::now() - tempo << endl;
//                tempo = ros::Time::now();
//                // Transformar nuvem completa com o resultado do ICP
//                transformPointCloudWithNormals<PointTN>(*parcial, *parcial, Ticp);
//                // Adicionar no vetor de otimizacoes a transformacao ajuste fino, depois colocar a da camera inicial
//                for(int i=0; i<ntilts; i++) otimizacoes_icp.emplace_back(Ticp);
//            } else { // Nao houve nuvem relevante aqui, adiciona entao identidade como otimizacao (nao ha otimizacao)
//                for(int i=0; i<ntilts; i++) otimizacoes_icp.emplace_back(Matrix4f::Identity());
//            }
//            anterior_icp->clear(); cloud_icp->clear();
            // Procurar por pontos ja existentes e retirar nesse caso
            // Se nao e a ultima
            float raio_vizinhos = (voxel_size > 0) ? 5*voxel_size/100.0f : 0.03;
            if(abs(cont_aquisicao - msg_angle->pose.pose.orientation.w) > ntilts)
                roo->searchNeighborsKdTree(parcial_pontos_novos, parcial_esq_anterior, raio_vizinhos, 130.0); // quanto maior o ultimo valor, maior o raio que eu aceito ter vizinhos
            else // Se for, comparar com a acumulada pra nao repetir pontos do inicio tambem
                roo->searchNeighborsKdTree(parcial_pontos_novos, acc                 , raio_vizinhos,  30.0);
            size_t tamanho_acc = parcial_pontos_novos->size();
//            cout << "Tempo para KDTREE FINAL: " << ros::Time::now() - tempo << endl;
            tempo = ros::Time::now();
            // Acumulando pontos novos
            *acc += *parcial_pontos_novos;
            ROS_WARN("Nuvem acumulada com %zu pontos.", acc->size());
            // Salvando nuvem atual para a proxima iteracao
            *anterior = *parcial_pontos_novos;
//            ROS_WARN("RELACAO DE PONTOS: entrada: %zu \tfiltragem: %zu (%.1f)\tkdtree: %zu (%.1f)", tamanho_chegada, tamanho_filtrada, 100.0f*float(tamanho_filtrada)/float(tamanho_chegada), tamanho_acc, 100.0f*float(tamanho_acc)/float(tamanho_chegada));
        }

        // Salvar a vista da esquerda para a proxima iteracao
        transformPointCloudWithNormals(*parcial, *parcial_esq_anterior, Tini.inverse());
        pass.setFilterFieldName("x");
        pass.setFilterLimits(0, 100); // Negativo de tudo no eixo X
        pass.setNegative(true);
        pass.setInputCloud(parcial_esq_anterior);
        pass.filter(*parcial_esq_anterior);
        transformPointCloudWithNormals(*parcial_esq_anterior, *parcial_esq_anterior, Tini);

        // Salvar nuvem parcia somente considerando os pontos novos
        ROS_INFO("Salvando nuvem de pontos %d ...", cont_aquisicao);
        if(cont_aquisicao < 10){
            pc->saveCloud(parcial_pontos_novos, "pf_00"+std::to_string(cont_aquisicao));
        } else if(cont_aquisicao < 100) {
            pc->saveCloud(parcial_pontos_novos, "pf_0" +std::to_string(cont_aquisicao));
        } else {
            pc->saveCloud(parcial_pontos_novos, "pf_"  +std::to_string(cont_aquisicao));
        }
        // Limpa a parcial atual vista em PAN
        parcial->clear();
        // Reseta o contador de nuvens
        contador_nuvens = 0;

    } // Fim do if else de contador nuvens

    // Publicar nuvem acumulada
    sensor_msgs::PointCloud2 msg_out;
    toROSMsg(*acc, msg_out);
    msg_out.header.stamp = ros::Time::now();
    msg_out.header.frame_id = "map";
    cl_pub.publish(msg_out);

    // Fazendo processos finais
    if(cont_aquisicao >= msg_angle->pose.pose.orientation.w){
        ROS_INFO("Processando todo o SFM otimizado ...");
        for(int i=0; i<pan_cameras.size(); i++){
            // Calcula a matriz de rotacao da camera
            float p = pan_cameras[i], t = tilt_cameras[i];
            Matrix3f Rt = pc->euler2matrix(0, -DEG2RAD(t), 0);
            Matrix3f Rp = pc->euler2matrix(0, 0, -DEG2RAD(p));
            Matrix3f Rcam = (Rp*Rt).transpose();

            // Calcula centro da camera
            Vector3f C = Rp*off_laser;
            C = Rp*Rt*pc->gettCam() + C;

            // Calcula vetor de translacao da camera por t = -R'*C
            Vector3f tcam = -Rcam*C;

            ///// ADICIONANDO CENTRO DA CAMERA NA NUVEM PARA TESTE
            //            PointTN pteste;
            //            pteste.x = C(0); pteste.y = C(1); pteste.z = C(2);
            //            pteste.r =   0 ; pteste.g =   0; pteste.b =   250;
            //            acc->push_back(pteste);

            // Escreve a linha e anota no vetor de linhas SFM
            string nome_imagem;
            if(i+1 < 10)
                nome_imagem = "imagem_00"+std::to_string(i+1)+".png";
            else if(i+1 < 100)
                nome_imagem = "imagem_0" +std::to_string(i+1)+".png";
            else
                nome_imagem = "imagem_"  +std::to_string(i+1)+".png";
            linhas_sfm.push_back(pc->escreve_linha_sfm(nome_imagem, Rcam, tcam));
        }
        ROS_INFO("Salvando nuvem acumulada final ...");
        pc->saveCloud(acc, "acumulada");
        ROS_INFO("Salvando SFM final ...");
        pc->compileFinalSFM(linhas_sfm);
        ROS_INFO("Processo finalizado.");
        ros::shutdown();
    }
    ////////////////
}

/// Main
///
int main(int argc, char **argv)
{
    ros::init(argc, argv, "acc_space2");
    ros::NodeHandle nh;
    ros::NodeHandle n_("~");
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    ROS_INFO("Iniciando o processo em FOG de registro de nuvens ...");

    // Pegando o nome da pasta por parametro
    string nome_param;
    n_.param<string>("pasta", nome_param , string("Dados_PEPO"));
    n_.param<float >("vs"   , voxel_size , 2    );
    n_.param<float >("df"   , depth      , 10   );
    n_.param<int   >("fp"   , filter_poli, 1    );

    // Apagando pasta atual e recriando a mesma na area de trabalho
    char* home;
    home = getenv("HOME");
    pasta = string(home)+"/Desktop/"+nome_param.c_str()+"/";
    int retorno = system(("rm -r "+pasta).c_str());
    mkdir(pasta.c_str(), 0777);

    // Inicia classe de processo de nuvens
    pc  = new ProcessCloud(pasta);
    roo = new RegisterObjectOptm();

    // Calculando taxas exatas entre deg e raw para os motores de pan e tilt
    deg_raw_pan  = 0.08789062;
    deg_raw_tilt = deg_raw_pan; //(deg_max_tilt - deg_min_tilt)/(raw_max_tilt - raw_min_tilt);
    raw_deg_pan  = 1.0/deg_raw_pan ;
    raw_deg_tilt = 1.0/deg_raw_tilt;

    // Iniciando a nuvem parcial acumulada de cada pan
    acc = (PointCloud<PointTN>::Ptr) new PointCloud<PointTN>();
    acc->header.frame_id = "map";
    anterior = (PointCloud<PointTN>::Ptr) new PointCloud<PointTN>();
    anterior->header.frame_id = "map";
    parcial = (PointCloud<PointTN>::Ptr) new PointCloud<PointTN>();
    parcial->header.frame_id = "map";
    parcial_esq_anterior = (PointCloud<PointTN>::Ptr) new PointCloud<PointTN>();
    parcial_esq_anterior->header.frame_id = "map";

    // Publicadores
    cl_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_user" , 50);
    od_pub = nh.advertise<nav_msgs::Odometry      >("/vista_space", 50);

    // Subscriber de poses das cameras
    ros::Subscriber poses_sub = nh.subscribe("/poses_space", 100, &posesCallback);

    // Iniciar subscritor da nuvem sincronizado com odometria
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh, "/cloud_space", 100);
    message_filters::Subscriber<nav_msgs::Odometry      > angle_sub(nh, "/angle_space", 100);
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), cloud_sub, angle_sub);
    sync.registerCallback(boost::bind(&cloudCallback, _1, _2));

    // Esperando ligar a camera e vir imagens - poses
    ros::Rate r(10);
    ROS_INFO("Aguardando vinda de poses ...");
    while(poses_sub.getNumPublishers() == 0){
        r.sleep();
        ros::spinOnce();
    }
    ROS_INFO("Comecando a reconstrucao do space ...");

    // Aguardar todo o processamento e ir publicando
    while(ros::ok()){
        ros::spinOnce();
    }

    return 0;
}
