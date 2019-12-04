//Includes

#include "std_msgs/String.h"
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>
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


using namespace std;


//Definitions
typedef pcl::PointXYZRGB PointT;

//Global vars
std::string filename_atual = "/home/joaomarcos/Desktop/atual.ply";
std::string filename_anterior = "/home/joaomarcos/Desktop/anterior.ply";

pcl::PointCloud<PointT>::Ptr accumulated_cloud;
pcl::PointCloud<PointT>::Ptr point_cloud_anterior;
pcl::PointCloud<PointT>::Ptr point_cloud_atual;
ros::Publisher pub_cloud_atual;
ros::Publisher pub_cloud_anterior;

pcl::PointCloud<PointT> atual, anterior;

int contador = 1;
int first = true;
int N = 10;
int tamanho = 0;


void filter_grid(pcl::PointCloud<PointT>::Ptr cloud, float leaf_size){
    pcl::VoxelGrid<PointT> grid;
    grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    grid.setInputCloud(cloud);
    grid.filter(*cloud);
}

void removeOutlier(pcl::PointCloud<PointT>::Ptr in, float mean, float deviation){
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(in);
    sor.setMeanK(mean);
    sor.setStddevMulThresh(deviation);
    sor.filter(*in);
}


sensor_msgs::PointCloud2 msg_atual;
sensor_msgs::PointCloud2 msg_anterior;

void cloud_open_target(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    //declare variables
    pcl::PointCloud<PointT>::Ptr cloud;
    sensor_msgs::PointCloud2 msg_out;
    
    msg_atual.header.frame_id    = "livox_frame";
    msg_anterior.header.frame_id = "livox_frame";
    
    
    ros::Time tic = ros::Time::now();
    ros::Time t = msg->header.stamp;
    
    //allocate objects for pointers
    cloud = (pcl::PointCloud<PointT>::Ptr) (new pcl::PointCloud<PointT>);
    
    msg_out.header.stamp = t;
    
    //Convert the ros message to pcl point cloud
    pcl::fromROSMsg (*msg, *cloud);
    
    *accumulated_cloud += *cloud;
    
    if(contador > N)
    {
        
        // Filtra antes
        float leaf_size = 0.05;
        //        filter_grid(accumulated_cloud, leaf_size);
        removeOutlier(accumulated_cloud, 1, 0.5);

        if(first)
        {
            (*point_cloud_anterior) += (*accumulated_cloud);// nuvem de pontos anterior recebe atual
            anterior = *point_cloud_anterior;
            first = false;
            //N=1;
            
        }
        
        if (!first)
        {
            
            point_cloud_atual->clear();
            (*point_cloud_atual) = (*accumulated_cloud);// nuvem de pontos anterior recebe atual
            
            // faz comparação e pintura
            // Verifica o menor tamanho para seguranca
            if (point_cloud_atual->points.size() >= point_cloud_anterior->points.size())
            {
                tamanho = point_cloud_anterior->points.size();
            }
            else
            {
                tamanho = point_cloud_atual->points.size();
            }

            pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;

            kdtree.setInputCloud(point_cloud_anterior);

            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            float radius = 0.1;
             #pragma omp parallel for
            for(int var = 0; var <= tamanho; var++)
            {
                if ( kdtree.radiusSearch (point_cloud_atual->points[var], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) == 0 )
                {
                    point_cloud_atual->points[var].r = 250; point_cloud_atual->points[var].g = 250; point_cloud_atual->points[var].b = 250;
                }
            }

            // (*point_cloud_anterior) = (*point_cloud_atual);// nuvem de pontos anterior recebe atual

            // Para publicacao
            atual    = *point_cloud_atual;
//            anterior = *point_cloud_anterior;
            
        }
        accumulated_cloud->clear();
        contador = 1;
        
    }

    contador++;
    
}




int main (int argc, char** argv)
{
    
    // Initialize ROS
    ros::init (argc, argv, "accumulatepointcloud");
    ros::NodeHandle nh;
    
    //Initialize accumulated cloud variable
    accumulated_cloud = (pcl::PointCloud<PointT>::Ptr) new pcl::PointCloud<PointT>;
    point_cloud_atual = (pcl::PointCloud<PointT>::Ptr) new pcl::PointCloud<PointT>;
    point_cloud_anterior= (pcl::PointCloud<PointT>::Ptr) new pcl::PointCloud<PointT>;
    
    accumulated_cloud->header.frame_id    = "livox_frame";
    point_cloud_atual->header.frame_id    = "livox_frame";
    point_cloud_anterior->header.frame_id = "livox_frame";
    
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub_target = nh.subscribe ("/livox/lidar", 10, cloud_open_target);
    
    pub_cloud_atual  = nh.advertise<sensor_msgs::PointCloud2>("msg_atual", 10);
    pub_cloud_anterior  = nh.advertise<sensor_msgs::PointCloud2>("msg_anterior", 10);

    
    //Loop infinetly
    ros::Rate r(10);
    while (ros::ok())
    {
        

        pcl::toROSMsg (atual, msg_atual);
        pcl::toROSMsg (anterior, msg_anterior);
        pub_cloud_atual.publish(msg_atual);
        pub_cloud_anterior.publish(msg_anterior);
        
        // Spin
        r.sleep();
        ros::spinOnce();
    }
    
}
