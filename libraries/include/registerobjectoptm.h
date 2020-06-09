#ifndef REGISTEROBJECTOPTM_H
#define REGISTEROBJECTOPTM_H

#include <string>
#include <math.h>
#include <cstdlib>
#include <boost/accumulators/accumulators.hpp>

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
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

/// Definicoes e namespaces
///
using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace std;
using namespace Eigen;

typedef PointXYZRGB       PointT ;
typedef PointXYZRGBNormal PointTN;

class RegisterObjectOptm
{
public:
    RegisterObjectOptm();
    virtual ~RegisterObjectOptm();
    void readCloudAndPreProcess(string name, PointCloud<PointTN>::Ptr cloud);
    void projectCloudAndAnotatePixels(PointCloud<PointTN>::Ptr cloud, Mat im, PointCloud<PointTN>::Ptr cloud_pix, float f, Vector3f t);
    Matrix4f icp(PointCloud<PointTN>::Ptr ctgt, PointCloud<PointTN>::Ptr csrc, int its);
    void matchFeaturesAndFind3DPoints(Mat imref, Mat imnow, PointCloud<PointTN>::Ptr cref, PointCloud<PointTN>::Ptr cnow, int npontos3d, vector<Point2d> &matches3d);
    Matrix4f optmizeTransformLeastSquares(PointCloud<PointTN>::Ptr cref, PointCloud<PointTN>::Ptr cnow, vector<Point2d> matches3d);
    Matrix4f optmizeTransformSVD(PointCloud<PointTN>::Ptr cref, PointCloud<PointTN>::Ptr cnow, vector<Point2d> matches3d);
    Matrix4f optmizeTransformP2P(PointCloud<PointTN>::Ptr cref, PointCloud<PointTN>::Ptr cnow, vector<Point2d> matches3d);

private:
    void plotDebug(Mat imref, Mat imnow, PointCloud<PointTN>::Ptr cref, PointCloud<PointTN>::Ptr cnow, vector<Point2f> pref, vector<Point2f> pnow, vector<Point2d> match);
    void filterMatchesLineCoeff(vector<DMatch> &matches, vector<KeyPoint> kpref, vector<KeyPoint> kpnow, float width, float n);
};

#endif // REGISTEROBJECTOPTM_H
