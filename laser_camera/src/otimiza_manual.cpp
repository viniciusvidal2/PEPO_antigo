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
    Clusters cl;
    ProcessCloud pc;

    std::string im_rgb = "camera_rgb.jpg", im_clu = "imagem_clusters.png", im_dep = "camera_virtual.jpg", im_dist = "distancias.png", im_nuvem = "nuvem_organizada.png";
    OtimizaImagens oi(std::string(home)+"/Desktop/Dados_B9/", im_rgb, im_clu, im_dep, im_dist, im_nuvem);


    // Le nuvem clusterizada
    PointCloud<PointTN>::Ptr nuvem_clusters (new PointCloud<PointTN>);
    loadPLYFile<PointTN>(std::string(home)+"/Desktop/Dados_B9/nuvem_clusters.ply", *nuvem_clusters);

    // Calcula arestas na imagem original e retorna o resultado
    Mat rgb, edges_rgb;
    rgb = oi.getImage("rgb");
    edges_rgb = oi.calculateEdgeFromOriginalImage(rgb, "rgb");

    ////////////////////////////////////////////////////////
    /// Loop com o teclado sobre o resultado da projecao ///
    ////////////////////////////////////////////////////////

    // Projeta a nuvem com a transformacao passada sobre camera virtual

    // Ajusta imagem e calcula arestas na camera virtual

    // Soma as duas imagens de arestas e mostra
    Mat soma;

    namedWindow("Ajuste teclado");
    imshow("Ajuste teclado", soma);
    // Repetir tudo com loop sobre as teclas
    int t;
    while(t != 32){
        t = waitKey(0);
        switch(t){
        case 99:
            break;
        }

        // Projeta a nuvem com a transformacao passada sobre camera virtual

        // Ajusta imagem e calcula arestas na camera virtual

        // Soma as duas imagens de arestas e mostra

        imshow("Ajuste teclado", soma);
    }
    ////////////////////////////////////////////////////////

}
