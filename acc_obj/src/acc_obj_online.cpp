#include <ros/ros.h>
#include <iostream>
#include <string>
#include <math.h>
#include <sys/stat.h>
#include <ostream>
#include <iterator>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "../../libraries/include/processcloud.h"
#include "../../libraries/include/registerobjectoptm.h"
#include "acc_obj/comandoAquisitar.h"

using namespace pcl;
using namespace pcl::io;
using namespace Eigen;
using namespace std;
using namespace cv;
using namespace message_filters;

typedef PointXYZRGB       PointT;
typedef PointXYZRGBNormal PointTN;
typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> syncPolicy;

// Controle de inicio de aquisicao
bool primeira_vez = true;
// Controle de processando dados que chegam ou nao
bool aceita_dados = false;
// O que fazer com a nova nuvem apos o processo
int nuvem_aceita = 0;
// Nuvem acumulada e todas as outras do processo
PointCloud<PointTN>::Ptr cobj;
PointCloud<PointTN>::Ptr cnow;
PointCloud<PointTN>::Ptr cref1;
PointCloud<PointTN>::Ptr cpixnow;
PointCloud<PointTN>::Ptr cpixref;
// Matrizes com os indices da nuvem de cada pixel
MatrixXi imrefpix(1080, 1920), imnowpix(1080, 1920);
// Nome da nossa pasta que estamos trabalhando
string pasta;
// Imagem de agora e da referencia
Mat imnow, imref;
// Pose da nuvem agora e na referencia
Matrix4f Tnow, Tref;
// Centro e orientacao da camera naquela aquisicao
Vector3f C;
Quaternion<float> q;
// Vetor de offset entre centro do laser e da camera - desenho solid, e foco
Vector3f t_off_lc(0.00, 0.0443, 0.023);
float f = 1130;
// Vetor de linhas para NVM
vector<string> linhas_nvm;
// Ponteiro para imagem que chega
cv_bridge::CvImagePtr image_ptr;
// Objetos de classe usados no processo
ProcessCloud *pc;
RegisterObjectOptm *roo;
// Contador de aquisicoes
int cont_aquisicao = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Subscriber sincronizado para os dois dados
void callback(const sensor_msgs::ImageConstPtr &msg_im, const sensor_msgs::PointCloud2ConstPtr &msg_cloud){
    // Se vamos aceitar os dados que chegam para acumular
    if(aceita_dados){
        // Volta a flag
        aceita_dados = false;
        // Converte imagem e nuvem
        PointCloud<PointT>::Ptr cloud (new PointCloud<PointT>);
        fromROSMsg(*msg_cloud, *cloud);
        image_ptr = cv_bridge::toCvCopy(msg_im, sensor_msgs::image_encodings::BGR8);
        // Como proceder se primeira vez ou seguindo o processo
        if(!primeira_vez){
            ////////////////////////////////////
            ////////////////////////////////////
            // Atualiza contagem aquisicao
            cont_aquisicao++;
            // Le dados, filtra e guarda na memoria
            ROS_INFO("Pre-processando nuvem %d...", cont_aquisicao);
            pc->preprocess(cloud, cnow);
            pc->calculateNormals(cnow);
            pc->filterCloudDepthCovariance(cnow, 20, 1);
            image_ptr->image.copyTo(imnow);
            imnowpix = MatrixXi::Constant(imnow.rows, imnow.cols, -1);

            // Projetar nuvem e salvar nuvem auxiliar de pixels
            ROS_INFO("Projetando para otimizar cores e anotar os pixels ...");
            roo->projectCloudAndAnotatePixels(cnow, imnow, cpixnow, f, t_off_lc, imnowpix);

            // Levar aonde paramos no processo de reconstrucao do objeto
            transformPointCloudWithNormals<PointTN>(*cnow, *cnow, Tref);

            // Calcular features e matches e retornar os N melhores e seus pontos em 3D
            // correspondentes a partir da nuvem auxiliar de pixels
            ROS_INFO("Match de features 2D e obtendo correspondencias em 3D ...");
            vector<Point2d> matches3Dindices;
            roo->matchFeaturesAndFind3DPoints(imref, imnow, cpixref, cpixnow, 70, matches3Dindices, imrefpix, imnowpix);
            ROS_INFO("Foram obtidas %zu correspondencias 3D.", matches3Dindices.size());

            // Continuamos somente se a imagem forneceu matches
            if(matches3Dindices.size() > 0){
                // Rodar a otimizacao da transformada por SVD
                ROS_INFO("Otimizando a transformacao relativa das nuvens por SVD ...");
                Tnow = roo->optmizeTransformSVD(cref1, cnow, matches3Dindices);

                // Transforma a nuvem atual com a transformacao encontrada
                transformPointCloudWithNormals<PointTN>(*cnow, *cnow, Tnow);

                // Refina a transformacao por ICP com poucas iteracoes
                ROS_INFO("Refinando registro por ICP ...");
                Matrix4f Ticp = roo->icp(cobj, cnow, 20);

                // Soma a nuvem transformada e poe no lugar certo somente pontos "novos"
                ROS_INFO("Registrando nuvem atual no objeto final ...");
                PointCloud<PointTN>::Ptr cnowtemp (new PointCloud<PointTN>);
                PointCloud<PointTN>::Ptr cobjmem  (new PointCloud<PointTN>);
                *cobjmem = *cobj; // Caso nao seja aceita a mudanca
                *cnowtemp = *cnow;
                roo->searchNeighborsKdTree(cnowtemp, cobj, 0.6);
                *cobj += *cnowtemp;

                ros::Rate r1(0.5);
                while(nuvem_aceita != 2 && nuvem_aceita != 3){
                    ROS_WARN("Aguardando comando de aceite ou nao para a captura ...");
                    r1.sleep();
                }
                if(nuvem_aceita == 2){ // Pode passar a nova aquisicao
                    // Pose atual para a nova nuvem
                    Matrix4f Tobj = Ticp*Tnow*Tref;
                    // StatisticalOutlierRemoval<PointTN> sor;
                    // sor.setInputCloud(cobj);
                    // sor.setMeanK(30);
                    // sor.setStddevMulThresh(2);
                    // sor.filter(*cobj);

                    // Calcula a pose da camera e escreve no NVM
                    ROS_INFO("Escrevendo no NVM ...");
                    Matrix4f Tcam = Matrix4f::Identity();
                    Tcam.block<3,1>(0, 3) = t_off_lc;
                    Tcam = Tobj*Tcam;
                    C = Tcam.block<3,1>(0, 3);
                    q = Tcam.block<3,3>(0, 0).transpose();
                    linhas_nvm.push_back(pc->escreve_linha_imagem(f, pasta+"imagem"+to_string(cont_aquisicao)+".png", C, q));
                    pc->compileFinalNVM(linhas_nvm);
                    ROS_INFO("Salvando a imagem na pasta ...");
                    imwrite(pasta+"imagem"+to_string(cont_aquisicao)+".png", imnow);

                    // Atualiza as referencias e parte para a proxima aquisicao
                    *cref1 = *cnow;
                    *cpixref = *cpixnow; // Nuvem de indices ja esta referenciado a origem
                    imrefpix = imnowpix;
                    Tref = Tobj;
                    imnow.copyTo(imref);
                    ROS_INFO("Tudo feito e pronto para proxima captura.");
                } else if(nuvem_aceita == 3){ // Nao foi boa a aquisicao, nao aceita e reseta a nuvem obj
                    ROS_INFO("Nao funcionou a nuvem, retorne a cena pois nao passou a aproximacao.");
                    *cobj = *cobjmem;
                } else if(nuvem_aceita == 4){ // Resetando a aquisicao
                    ROS_INFO("Resetando toda a aquisicao.");
                    cnow->clear(); cref1->clear(); cobj->clear(); cpixnow->clear(); cpixref->clear();
                    linhas_nvm.clear();
                    primeira_vez = true;
                }
                // Retorna o flag para o neutro
                nuvem_aceita = 0;
            } // Fim do if de matches
            ////////////////////////////////////
            ////////////////////////////////////
        } else { // Se na primeira vez que roda o subscriber
            ROS_INFO("Iniciando os dados de referencia ...");
            cont_aquisicao++;
            pc->preprocess(cloud, cref1);
            pc->calculateNormals(cref1);
            pc->filterCloudDepthCovariance(cref1, 15, 1);
            image_ptr->image.copyTo(imref);
            imrefpix = MatrixXi::Constant(imref.rows, imref.cols, -1);
            // Projetar a nuvem na imagem de forma calibrada, otimizar a cor e salvar em matriz de
            // pontos auxiliar os pixels correspondentes a cada ponto
            roo->projectCloudAndAnotatePixels(cref1, imref, cpixref, f, t_off_lc, imrefpix);
            *cobj = *cref1;
            // Inicia pose e escreve para o NVM
            Tref = Matrix4f::Identity();
            Tnow = Tref;
            C = -t_off_lc;
            q = Quaternion<float>::Identity();
            linhas_nvm.push_back(pc->escreve_linha_imagem(f, pasta+"imagem"+to_string(cont_aquisicao)+".png", C, q));
            pc->compileFinalNVM(linhas_nvm);
            // Salva a imagem
            imwrite(pasta+"imagem"+to_string(cont_aquisicao)+".png", imref);
            // Fim da primeira vez
            primeira_vez = false;
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Servico para o que fazer quando processar os dados
bool comando_aquisitar(acc_obj::comandoAquisitar::Request &req, acc_obj::comandoAquisitar::Response &res){
    switch(req.comando){
    case 1: // Aquisicao foi chamada
        aceita_dados = true;
        cont_aquisicao++;
        res.result = 1;
        ROS_INFO("Realizando aquisicao na posicao %d ...", cont_aquisicao);
        break;
    case 2: // Aquisicao vai passar
        nuvem_aceita = 2;
        res.result = 1;
        ROS_INFO("Aquisicao foi boa, ajustando ...");
        break;
    case 3: // Aquisicao nao vai passar
        nuvem_aceita = 3;
        ROS_INFO("Aquisicao nao foi boa, desfazendo ...");
        res.result = 1;
        break;
    case 4: // Resetando
        nuvem_aceita = 4;
        ROS_INFO("Resetando toda a aquisicao ...");
        res.result = 1;
        break;
    case 5:
        ROS_INFO("Finalizando o processo e salvando tudo ...");
        savePLYFileBinary<PointTN>(pasta+"objeto.ply", *cobj);
        ROS_INFO("Tudo finalizado.");
        break;
    default:
        break;
    }

    return true;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Parametro de iteracoes do icp reconfiguravel

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
    ros::init(argc, argv, "acc_obj_online");
    ros::NodeHandle nh;
    ros::NodeHandle n_("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    // Inicia nuvens
    cobj    = (PointCloud<PointTN>::Ptr) new PointCloud<PointTN>;
    cnow    = (PointCloud<PointTN>::Ptr) new PointCloud<PointTN>;
    cref1    = (PointCloud<PointTN>::Ptr) new PointCloud<PointTN>;
    cpixnow = (PointCloud<PointTN>::Ptr) new PointCloud<PointTN>;
    cpixref = (PointCloud<PointTN>::Ptr) new PointCloud<PointTN>;
    cobj->header.frame_id = "obj";

    // Nome da pasta a gravar tudo
    string nome_param;
    n_.param("pasta", nome_param, string("Dados_PEPO"));

    // Criando pasta na area de trabalho
    char* home;
    home = getenv("HOME");
    pasta = std::string(home)+"/Desktop/"+nome_param.c_str()+"/";
    system(("rm -r "+pasta).c_str());
    mkdir(pasta.c_str(), 0777);

    // Objetos de classe usados no processo
    pc  = new ProcessCloud(pasta);
    roo = new RegisterObjectOptm();

    // Inicia os subscribers da imagem e nuvem atuais
    message_filters::Subscriber<sensor_msgs::Image      > subim(nh, "/imagem"     , 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> subpc(nh, "/nuvem_atual", 10);
    Synchronizer<syncPolicy> sync(syncPolicy(10), subim, subpc);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // Inicia o servico de aquisicao
    ros::ServiceServer procedimento = nh.advertiseService("/capturar", comando_aquisitar);

    // Inicia o publicador da nuvem objeto atual
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/obj", 10);
    sensor_msgs::PointCloud2 pc_msg;
    pc_msg.header.frame_id = "obj";

    // Roda o no
    ros::Rate r(1);
    while(ros::ok()){
        if(cobj->size() > 0){
            toROSMsg(*cobj, pc_msg);
            pub.publish(pc_msg);
        }
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
