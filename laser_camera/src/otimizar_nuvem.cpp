/// Includes
///
#include <ros/ros.h>

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
#include <pcl/kdtree/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "../../libraries/include/processcloud.h"

/// Namespaces
///
using namespace pcl;
using namespace pcl::io;
using namespace pcl::search;
using namespace pcl::visualization;
using namespace std;

/// Definiçoes
///
typedef PointXYZRGBNormal PointTN;
typedef PointXYZRGB       PointT ;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "otimizar_nuvem");
    ros::NodeHandle nh("~");

    // Inicia variaveis
    std::string nome_nuvem;
    double mean, deviation;
    ProcessCloud pc;

    // Receber parametros de arquivo launch
    nh.param("nome_nuvem"  , nome_nuvem, std::string("nuvem.ply"));
    nh.param("mean_or"     , mean      , 1.0                     );
    nh.param("deviation_or", deviation , 1.0                     );

    // Carregar a nuvem - criar backup
    PointCloud<PointTN>::Ptr original (new PointCloud<PointTN>());
    PointCloud<PointTN>::Ptr modif    (new PointCloud<PointTN>());
    if(loadPLYFile<PointTN>(nome_nuvem, *original) == -1)
        ROS_ERROR("Nao encontrou a nuvem na pasta %s", nome_nuvem.c_str());
    *modif = *original;

    // Cria a KD-Tree para usar nos metodos
    pcl::search::KdTree<PointT>::Ptr tree_xyzrgb (new pcl::search::KdTree<PointT>());

    // Filtrar por outliers
    ROS_INFO("Filtrando por outliers ...");
    StatisticalOutlierRemoval<PointTN> sor;
    sor.setInputCloud(modif);
    sor.setMeanK(mean);
    sor.setStddevMulThresh(deviation);
    sor.filter(*modif);

    // Separando nuvem em nuvem de pontos XYZ, nuvem XYZRGB e so as normais
    PointCloud<PointXYZ   >::Ptr cloudxyz    (new PointCloud<PointXYZ   >());
    PointCloud<PointT     >::Ptr cloudxyzrgb (new PointCloud<PointT     >());
    PointCloud<PointNormal>::Ptr cloudnormal (new PointCloud<PointNormal>());

    ROS_INFO("Separando nuvem para processar ....");
    #pragma omp parallel for num_threads(50)
    for (size_t i=0; i < modif->size(); i++) {
        PointXYZ  xyz;
        PointT      t;
        PointNormal n;
        // Preenchendo cada ponto para a respectiva nuvem
        xyz.x = modif->points[i].x; xyz.y = modif->points[i].y; xyz.z = modif->points[i].z;
        cloudxyz->push_back(xyz);
        t.x = modif->points[i].x; t.y = modif->points[i].y; t.z = modif->points[i].z;
        t.r = modif->points[i].r; t.g = modif->points[i].g; t.b = modif->points[i].b;
        cloudxyzrgb->push_back(t);
        n.normal_x = modif->points[i].normal_x; n.normal_y = modif->points[i].normal_y; n.normal_z = modif->points[i].normal_z;
        n.x        = modif->points[i].x       ; n.y        = modif->points[i].y       ; n.z        = modif->points[i].z       ;
        cloudnormal->push_back(n);
    }

    // Passar filtro polinomial
    ROS_INFO("Aplicando filtro polinomial ...");
    PointCloud<PointTN>::Ptr temp (new PointCloud<PointTN>());
    MovingLeastSquares<PointT, PointTN> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(cloudxyzrgb);
    mls.setPolynomialOrder(5);
    mls.setSearchMethod(tree_xyzrgb);
    mls.setSearchRadius(0.12);
    mls.process(*temp);
    *modif = *temp;

    // Corrigir normais da nuvem caso necessario
    ROS_INFO("Corrigindo normais defeituosas ....");
//    pc.calculateNormals(modif);

    // Mostrar no visualizador da PCL o inicio e o resultado
    PCLVisualizer::Ptr viewer_o (new PCLVisualizer ("Original"));
    viewer_o->setBackgroundColor(0, 0, 0);
    PointCloudColorHandlerRGBField<PointTN> rgbo(original);
    viewer_o->addPointCloud<PointTN>(original, rgbo, "original");
    viewer_o->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2, "original");
    viewer_o->addCoordinateSystem(1.0);
    viewer_o->initCameraParameters();
    viewer_o->spin();
    PCLVisualizer::Ptr viewer_m (new PCLVisualizer ("Modificada"));
    viewer_m->setBackgroundColor(0, 0, 0);
    PointCloudColorHandlerRGBField<PointTN> rgbm(modif);
    viewer_m->addPointCloud<PointTN>(modif, rgbm, "modificada");
    viewer_m->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2, "modificada");
    viewer_m->addCoordinateSystem(1.0);
    viewer_m->initCameraParameters();
    viewer_m->spin();

    // Finaliza com uma rodada somente
    ros::spinOnce();
    return 0;
}
