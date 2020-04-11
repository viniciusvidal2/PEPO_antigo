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

#include "../../libraries/include/otimizaimagens.h"
#include "../../libraries/include/clusters.h"
#include "../../libraries/include/processcloud.h"

using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace std;
typedef PointXYZRGBNormal PointTN;

ProcessCloud *pc;
OtimizaImagens *oi;

/////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf fromNormalizedToReal(Eigen::VectorXf in, Eigen::MatrixXf r){
    // Aplica regra de 3 segundo os limites das restriçoes
    // Variaveis escaladas de -1 a 1
    // Valores reais estao para as restricoes definidas no main
    Eigen::VectorXf saida(in.rows());
    omp_set_dynamic(1);
    #pragma omp parallel for
    for(int i=0; i<saida.rows(); i++)
        saida(i) = (r(1,i) - r(0,i))*(in(0) + 1)/2 + r(0, i);

    return saida;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXf bat(float fx, float fy, float tx, float ty,
                    PointCloud<PointTN>::Ptr nuvem_clusters, Mat edges_rgb){
    // Variavel de saida
    Eigen::VectorXf saida(4);

    ROS_INFO("Iniciando variaveis de otimizacao ...");

    // Restricoes no espaco de busca para foco e translacao
    Eigen::MatrixXf rest(2, 4);
    float libt = 5.0, libf = 50.0; // Centimetros de liberdade em translaçao / unidade de foco, para espaço de busca
    rest << fx-libf, fy-libf, tx-libt, ty-libt,
            fx+libf, fy-libf, tx+libt, ty+libt;

    // Parametros do algoritmo de bats
    int nbats = 30;
    float alfa = 0.5, lambda = 0.6, beta = 0.2, e = -0.1;

    // Velocidades, taxa de emissao e amplitudes sonoras
    Eigen::MatrixXf v(nbats, rest.cols()), r(nbats, rest.cols());
    Eigen::MatrixXf As = Eigen::MatrixXf::Constant(nbats, rest.cols(), 1);

    // Vetor de fobs e controle de iteracoes
    Eigen::VectorXf F(nbats); // Vetor contendo a FOB de cada morcego
    int t = 0, t_max = 10, t_lim = 7;

    // Rolando uma vez a fob de cada bat para inicio de conversa
    ROS_INFO("Iniciando bats e respectivas fobs ...");
    ros::Time ini = ros::Time::now();
    Eigen::MatrixXf bats(nbats, rest.cols());
    bats = Eigen::MatrixXf::Random(nbats, bats.cols()); // Linha de cada morcego com cada parametro em uma coluna
    omp_set_dynamic(0);
    #pragma omp parallel for num_threads(bats.rows())
    for(int i=0; i<bats.rows(); i++){
        // Traz valores de volta ao real
        Eigen::VectorXf vr = fromNormalizedToReal(bats.row(i), rest);
        // Projeta a nuvem de entrada na camera virtual
        Mat nuvem_projetada_raw   = pc->projectCloudToLaserCenter(nuvem_clusters, vr(0), vr(1), vr(2), vr(3), edges_rgb.size());
        Mat imagem_camera_virtual = oi->correctColorCluster(nuvem_projetada_raw);
        // Calculado arestas na imagem virtual
        Mat edges_clusters        = oi->calculateEdgeFromOriginalImage(imagem_camera_virtual, "clusters");
        // Calcula a FOB para essa imagem com arestas e a rgb inicial
        F(i) = oi->FOB(edges_rgb, edges_clusters);
    }

    ROS_INFO("Demorou pra iniciar os bats um total de %.2f segundos.", (ros::Time::now()-ini).toSec());

    // Indice do menor valor da fob
    Eigen::VectorXf::Index indice_melhor_bat;
    float melhor_valor = F.minCoeff(&indice_melhor_bat);

    //// Etapa iterativa do algoritmo ////
    ROS_INFO("Inicio do processo iterativo ...");
    ini = ros::Time::now();
    float valor_anterior = melhor_valor;
    int contador_repeticoes = 0;
    while(t < t_max){

        ros::Time iniit = ros::Time::now();
        // Controle de repeticao
        if(valor_anterior - melhor_valor <= 1e-2){
            contador_repeticoes += 1;
        } else {
            contador_repeticoes = 0;
        }
        valor_anterior = melhor_valor;

        // Se nao estourou repeticao, rodam os morcegos
        if(contador_repeticoes < t_lim){
            ROS_INFO("Iteracao %d ...", t+1);
            omp_set_dynamic(0);
            #pragma omp parallel for num_threads(bats.rows())
            for(int i=0; i<bats.rows(); i++){
                // Calculo da velocidade do morcego
                v.row(i) << v.row(i) + (bats.row(indice_melhor_bat)-bats.row(i))*beta;
                Eigen::VectorXf bat_temp(bats.cols());
                bat_temp << bats.row(i).transpose() + v.row(i).transpose();
                // Etapa de busca local
                if((double)rand()/(RAND_MAX) < r(i))
                    bat_temp << bats.row(indice_melhor_bat).transpose() + Eigen::MatrixXf::Constant(bat_temp.rows(), bat_temp.cols(), e*As.mean());
                // Etapa de avaliacao da fob
                for(int j=0; j<bat_temp.rows(); j++){ // Controle de limites
                    if(bat_temp(j) < -1) bat_temp(j) = -1;
                    if(bat_temp(j) >  1) bat_temp(j) =  1;
                }
                // Traz valores de volta ao real
                Eigen::VectorXf vr = fromNormalizedToReal(bat_temp, rest);
                // Projeta a nuvem de entrada na camera virtual
                Mat nuvem_projetada_raw   = pc->projectCloudToLaserCenter(nuvem_clusters, vr(0), vr(1), vr(2), vr(3), edges_rgb.size());
                Mat imagem_camera_virtual = oi->correctColorCluster(nuvem_projetada_raw);
                // Calculando arestas na imagem virtual
                Mat edges_clusters = oi->calculateEdgeFromOriginalImage(imagem_camera_virtual, "clusters");
                // Calcula a FOB para essa imagem com arestas e a rgb inicial
                float fob_temp = oi->FOB(edges_rgb, edges_clusters);
                // Atualizando o morcego ou nao por busca global
                if((double)rand()/(RAND_MAX) < As(i) || fob_temp < F(i)){
                    bats.row(i) << bat_temp.transpose();
                    r(i)  = 1 - exp(-lambda*t);
                    As(i) = alfa*As(i);
                    F(i)  = fob_temp;
                }
//                // Busca novamente pelo melhor valor dentre os morcegos pela fob
//                melhor_valor = F.minCoeff(&indice_melhor_bat);
            } // Fim do for de busca dos morcegos
            // Busca novamente pelo melhor valor dentre os morcegos pela fob
            melhor_valor = F.minCoeff(&indice_melhor_bat);
            // Atualiza iteracao
            t++;
            ROS_INFO("Tempo dessa iteracao: %.2f.", (ros::Time::now()-iniit).toSec());
        } else {
            break; // Ja acabou a busca aqui entao
        }

    } // fim do while t_max
    //////////////////////////////////////
    ROS_INFO("Tempo decorrido do processo iterativo: %.2f.", (ros::Time::now()-ini).toSec());

    // Tira os valores de normalizados para reais novamente
    saida = fromNormalizedToReal(bats.row(indice_melhor_bat), rest);

    return saida;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    // Inicia o no
    ros::init(argc, argv, "otimiza_bat");
    ros::NodeHandle nh;
    char* home;
    home = getenv("HOME");

    // Inicia classes de funcoes para imagens, otimizacao e nuvem
    ROS_INFO("Carregando dados para processar ...");
    pc = new ProcessCloud();

    std::string im_rgb = "camera_rgb.png", im_clu = "imagem_clusters.png", im_dep = "imagem_virtual.png";
    oi = new OtimizaImagens(std::string(home)+"/Desktop/Dados_B9/", im_rgb, im_clu, im_dep);

    // Le nuvem clusterizada
    PointCloud<PointTN>::Ptr nuvem_clusters (new PointCloud<PointTN>);
    loadPLYFile<PointTN>(std::string(home)+"/Desktop/Dados_B9/nuvem_clusters.ply", *nuvem_clusters);
    vector<int> indicesnan;
    removeNaNFromPointCloud(*nuvem_clusters, indicesnan);

    // Calcula arestas na imagem original e retorna o resultado
    ROS_INFO("Calculando arestas imagem RGB ...");
    Mat rgb, edges_rgb;
    rgb = oi->getImage("rgb");
    edges_rgb = oi->calculateEdgeFromOriginalImage(rgb, "rgb");
    //    edges_rgb = oi.calculateHoughTransformFromOriginalImage(edges_rgb, "rgb");

    ////////////////////////////////////////////
    /// Inicia otimizacao por metodo de Bats ///
    ////////////////////////////////////////////

    // Parametros para a funcao de projecao - intrinsecos da camera a principio
    float fx = 1496.701399, fy = 1475.059238, tx = 2, ty = 9;
    //    float fx = 1484.701399, fy = 1477.059238, tx = 0, ty = 0;

    // Projeta a nuvem com a transformacao passada sobre camera virtual
    ROS_INFO("Projetando camera virtual ...");
    Mat nuvem_projetada_raw;
    nuvem_projetada_raw   = pc->projectCloudToLaserCenter(nuvem_clusters, fx, fy, tx, ty, rgb.size());

    // Ajusta imagem e calcula arestas na camera virtual
    ROS_INFO("Calculando imagem e arestas da camera virtual ...");
    Mat edges_clusters, imagem_camera_virtual;
    imagem_camera_virtual = oi->correctColorCluster(nuvem_projetada_raw);
    edges_clusters        = oi->calculateEdgeFromOriginalImage(imagem_camera_virtual, "clusters");
    //    edges_clusters        = oi.calculateHoughTransformFromOriginalImage(edges_clusters, "clusters");

//    // Soma as duas imagens de arestas e mostra
//    Mat soma;
//    addWeighted(edges_rgb, 1.0, edges_clusters, 1.0, 0.0, soma);
//    namedWindow("Inicial");
//    imshow("Inicial", soma);
//    waitKey(0);
//    cvDestroyAllWindows();

    // Chama metodo de bats e retorna vetor Eigen com os valores ordenados da seguinte forma
    // fx - fy - tx - ty
    ROS_INFO("Comecando o metodo de otimizacao por BATS ...");
    Eigen::VectorXf opt(4);
//    opt = bat(fx, fy, tx, ty, nuvem_clusters, edges_rgb);
    opt << fx, fy, tx, ty;

    ////////////////////////////////////////////

    // Se deu certo, colore nuvem com a foto entregue
    ROS_INFO("Projetando nuvem na imagem colorida com valores calibrados ...");

    PointCloud<PointTN>::Ptr nuvem_calibrada (new PointCloud<PointTN>);
    pc->colorCloudWithCalibratedImage(nuvem_clusters, nuvem_calibrada, rgb, opt(0), opt(1), opt(2), opt(3));

    cout << "\nParametros Otimizados na ordem como estao:\n\n" << opt << endl << endl;

    // Salvar arquivo NVM para o MART
    ROS_INFO("Salvando arquivo NVM ...");
    std::string nvm = std::string(home)+"/Desktop/Dados_B9/camera.nvm";
    pc->writeNVM(nvm, std::string(home)+"/Desktop/Dados_B9/"+im_rgb, opt);

    // Salvando a nuvem final para dar uma ideia
    ROS_INFO("Salvando nuvem colorida e finalizando ...");
    savePLYFileASCII<PointTN>(std::string(home)+"/Desktop/Dados_B9/nuvem_colorida_calibrada.ply", *nuvem_calibrada);

    ros::spinOnce();

    return 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
