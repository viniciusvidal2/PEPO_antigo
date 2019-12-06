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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "otimizar_nuvem");
    ros::NodeHandle nh;

    // Inicia variaveis
    std::string nome_nuvem;
    double mean, deviation;

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
    pcl::search::KdTree<PointTN>::Ptr tree (new pcl::search::KdTree<PointTN>());

    // Filtrar por outliers
    StatisticalOutlierRemoval<PointTN> sor;
    sor.setInputCloud(modif);
    sor.setMeanK(mean);
    sor.setStddevMulThresh(deviation);
    sor.filter(*modif);

    // Passar filtro polinomial
    PointCloud<PointTN> temp;
    MovingLeastSquares<PointTN, PointNormal> mls;
    mls.setComputeNormals(false);
    mls.setInputCloud(modif);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.02);
    mls.process(temp);

    // Mostrar no visualizador da PCL o inicio e o resultado
    PCLVisualizer::Ptr viewer_o (new PCLVisualizer ("Original"));
    viewer_o->setBackgroundColor(0, 0, 0);
    PointCloudColorHandlerRGBField<PointTN> rgbo(original);
    viewer_o->addPointCloud<PointTN>(original, rgbo, "original");
    viewer_o->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2, "original");
//    viewer_o->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
    viewer_o->addCoordinateSystem(1.0);
    viewer_o->initCameraParameters();
    PCLVisualizer::Ptr viewer_m (new PCLVisualizer ("Modificada"));
    viewer_m->setBackgroundColor(0, 0, 0);
    PointCloudColorHandlerRGBField<PointTN> rgbm(modif);
    viewer_m->addPointCloud<PointTN>(modif, rgbm, "modificada");
    viewer_m->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2, "modificada");
//    viewer_m->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
    viewer_m->addCoordinateSystem(1.0);
    viewer_m->initCameraParameters();

    // Finaliza com uma rodada somente
    ros::spinOnce();
    return 0;
}
