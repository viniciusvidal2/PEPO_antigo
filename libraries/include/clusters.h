#ifndef CLUSTERS_H
#define CLUSTERS_H

#include <string>
#include <math.h>
#include <cstdlib>

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
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "../../libraries/include/clusters.h"
#include "../../libraries/include/processcloud.h"

/// Definicoes e namespaces
///
using namespace pcl;
using namespace pcl::io;
using namespace pcl::search;
using namespace cv;
using namespace std;
typedef PointXYZRGB       PointT ;
typedef PointXYZRGBNormal PointTN;


class Clusters
{
public:
    Clusters();
    virtual ~Clusters();
    void obtainPlanes(PointCloud<PointTN>::Ptr in, vector<PointCloud<PointTN>> &planos, PointCloud<PointTN>::Ptr out);
    void extractClustersRegionGrowing(PointCloud<PointTN>::Ptr in, vector<PointCloud<PointTN>> &clust);
    void extractClustersRegionGrowingRGB(PointCloud<PointTN>::Ptr in, vector<PointCloud<PointTN>> &clust);
    void extractClustersEuclidian(PointCloud<PointTN>::Ptr in, vector<PointCloud<PointTN>> &clust);
    void separateClustersByDistance(vector<PointCloud<PointTN>> &clust);
    void setColorPallete(size_t l);
    void colorCloud(PointCloud<PointTN>::Ptr cloud, size_t i);

private:
    vector<int> pal_r, pal_g, pal_b;
};

#endif // CLUSTERS_H
