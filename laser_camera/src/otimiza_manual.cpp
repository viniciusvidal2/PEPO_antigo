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

int main(int argc, char **argv)
{
    // Inicia o no
    ros::init(argc, argv, "otimiza_manual");
    ros::NodeHandle nh;
    char* home;
    home = getenv("HOME");

    // Inicia classes de funcoes para imagens, otimizacao e nuvem
    ROS_INFO("Carregando dados para processar ...");
    ProcessCloud pc;

    std::string im_rgb = "camera_rgb.png", im_clu = "imagem_clusters.png", im_dep = "imagem_virtual.png";
    OtimizaImagens oi(std::string(home)+"/Desktop/Dados_B9/", im_rgb, im_clu, im_dep);

    // Le nuvem clusterizada
    PointCloud<PointTN>::Ptr nuvem_clusters (new PointCloud<PointTN>);
    loadPLYFile<PointTN>(std::string(home)+"/Desktop/Dados_B9/nuvem_clusters.ply", *nuvem_clusters);
    vector<int> indicesnan;
    removeNaNFromPointCloud(*nuvem_clusters, indicesnan);

    // Calcula arestas na imagem original e retorna o resultado
    ROS_INFO("Calculando arestas imagem RGB ...");
    Mat rgb, edges_rgb;
    rgb = oi.getImage("rgb");
    edges_rgb = oi.calculateEdgeFromOriginalImage(rgb, "rgb");
//    edges_rgb = oi.calculateHoughTransformFromOriginalImage(edges_rgb, "rgb");

    ////////////////////////////////////////////////////////
    /// Loop com o teclado sobre o resultado da projecao ///
    ////////////////////////////////////////////////////////

    // Funcao objeto
    float fob = 0;

    // Parametros para a funcao de projecao - intrinsecos da camera a principio
    float fx = 1496.701399, fy = 1475.059238, tx = 2, ty = 9, passo = 5;
//    float fx = 1484.701399, fy = 1477.059238, tx = 0, ty = 0, passo = 5;

    // Projeta a nuvem com a transformacao passada sobre camera virtual
    ROS_INFO("Projetando camera virtual ...");
    Mat nuvem_projetada_raw;
    nuvem_projetada_raw   = pc.projectCloudToLaserCenter(nuvem_clusters, fx, fy, tx, ty, rgb.size());

    // Ajusta imagem e calcula arestas na camera virtual
    ROS_INFO("Calculando imagem e arestas da camera virtual ...");
    Mat edges_clusters, imagem_camera_virtual;
    imagem_camera_virtual = oi.correctColorCluster(nuvem_projetada_raw);
    edges_clusters        = oi.calculateEdgeFromOriginalImage(imagem_camera_virtual, "clusters");
//    edges_clusters        = oi.calculateHoughTransformFromOriginalImage(edges_clusters, "clusters");

    // Soma as duas imagens de arestas e mostra
    Mat soma;
    addWeighted(edges_rgb, 1.0, edges_clusters, 1.0, 0.0, soma);
    namedWindow("Ajuste teclado");
    imshow("Ajuste teclado", soma);

    // Repetir tudo com loop sobre as teclas
    int t;
    while(t != 32){
        t = waitKey(0);
        switch(t){
        case 99: // letra c, para esquerda
            tx -= passo;
            break;
        case 102: // letra f, para cima
            ty -= passo;
            break;
        case 98: // letra b, para direita
            tx += passo;
            break;
        case 118: // letra v, para baixo
            ty += passo;
            break;
        case 97:  // letra a, reduz passo
            if(passo > 1)
                passo -= 1;
            break;
        case 115: // letra s, aumenta passo
            passo += 1;
            break;
        case 105: // letra i, diminui foco em X
            fx -= passo;
            break;
        case 111: // letra o, aumenta o foco em X
            fx += passo;
            break;
        case 107: // letra k, diminui o foco em Y
            fy -= passo;
            break;
        case 108: // letra l, aumenta o foco em Y
            fy += passo;
            break;
        default:
            break;
        }
        ROS_INFO("Valores atuais. tx: %.1f   ty: %.1f   fx: %.1f   fy:%.1f   passo: %.0f", tx, ty, fx, fy, passo);
        if( t != 97 && t != 115 && t!= 32){ // Se alterar o passo so, nao calcula
            // Projeta a nuvem com a transformacao passada sobre camera virtual
            ROS_INFO("Projetando camera virtual ...");
            nuvem_projetada_raw   = pc.projectCloudToLaserCenter(nuvem_clusters, fx, fy, tx, ty, rgb.size());

            // Ajusta imagem e calcula arestas na camera virtual
            ROS_INFO("Calculando imagem e arestas da camera virtual ...");
            imagem_camera_virtual = oi.correctColorCluster(nuvem_projetada_raw);
            edges_clusters        = oi.calculateEdgeFromOriginalImage(imagem_camera_virtual, "clusters");
//            edges_clusters        = oi.calculateHoughTransformFromOriginalImage(edges_clusters, "clusters");

            // Aplicando a fob aqui para ver como sai
            ROS_INFO("Calculando a FOB ...");
            fob = oi.FOB(edges_rgb, edges_clusters);
            ROS_INFO("Valor da FOB na iteracao: %.2f", fob);

            ROS_INFO("Pronto, proximo ajuste ...");
            // Soma as duas imagens de arestas e mostra
            addWeighted(edges_rgb, 1.0, edges_clusters, 1.0, 0.0, soma);
            imshow("Ajuste teclado", soma);
        }
    }
    ////////////////////////////////////////////////////////

    // Se deu certo, colore nuvem com a foto entregue
    ROS_INFO("Projetando nuvem na imagem colorida com valores calibrados ...");

    // Le nuvem inicial, sem clusters, com mais informacao
    PointCloud<PointTN>::Ptr nuvem_final (new PointCloud<PointTN>);
    loadPLYFile<PointTN>(std::string(home)+"/Desktop/Dados_B9/nuvem_final.ply", *nuvem_final);

    PointCloud<PointTN>::Ptr nuvem_calibrada (new PointCloud<PointTN>);
    pc.colorCloudWithCalibratedImage(nuvem_final, nuvem_calibrada, rgb, fx, fy, tx, ty);

    // Salvando a nuvem final para dar uma ideia
    ROS_INFO("Salvando e finalizando ...");
    savePLYFileASCII<PointTN>(std::string(home)+"/Desktop/Dados_B9/nuvem_colorida_calibrada.ply", *nuvem_calibrada);

    ros::spinOnce();

    return 0;
}
