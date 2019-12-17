#include "../include/clusters.h"

/////////////////////////////////////////////////////////////////////////////////////////////////
Clusters::Clusters()
{

}
/////////////////////////////////////////////////////////////////////////////////////////////////
Clusters::~Clusters(){

}
/////////////////////////////////////////////////////////////////////////////////////////////////
void Clusters::obtainPlanes(PointCloud<PointTN>::Ptr in, vector<PointCloud<PointTN>> &planos, PointCloud<PointTN>::Ptr out){
    // Cria coeficientes do modelo
    ModelCoefficients::Ptr coefficients (new ModelCoefficients);
    // Indices dos pontos que estao dentro
    PointIndices::Ptr inliers (new PointIndices);
    // Objeto para realizar segmentacao
    SACSegmentation<PointTN> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(SACMODEL_PLANE);
    seg.setMethodType(SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold (0.1);
    // Processar planos ate cansar
    PointCloud<PointTN>::Ptr temp (new PointCloud<PointTN>), plane (new PointCloud<PointTN>), cloud_f (new PointCloud<PointTN>);
    *temp = *in;
    int nr_points = (int) temp->points.size();
    while(temp->size() > 0.5*nr_points){ // Ainda podem haver planos significativos
        seg.setInputCloud(temp);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0){
            ROS_WARN("Could not estimate a planar model for the given dataset, continuing...");
            break;
        }
        // Extract the planar inliers from the input cloud
        ExtractIndices<PointTN> extract;
        extract.setInputCloud(temp);
        extract.setIndices(inliers);
        extract.setNegative(false);
        // Get the points associated with the planar surface
        extract.filter(*plane);
        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *temp = *cloud_f;
        // Adiciona ao vetor o plano obtido
        planos.push_back(*plane);
    }
    // Passar o que sobrou sem planos para a funcao principal
    *out = *temp;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void Clusters::extractClusters(PointCloud<PointTN>::Ptr in, vector<PointCloud<PointTN>> &clust){

}
/////////////////////////////////////////////////////////////////////////////////////////////////
void Clusters::colorCloud(PointCloud<PointTN>::Ptr cloud){
    // Definir cor de forma aleatoria
    int int_r = int(int((rand()) % (250 - 40)) + 40);
    int int_g = int(int((rand()) % (250 - 40)) + 40);
    int int_b = int(int((rand()) % (250 - 40)) + 40);
    #pragma omp for num_threads(int(cloud.size()/10))
    for(size_t j=0; j < cloud->size(); j++){
        cloud->points[j].r = int_r;
        cloud->points[j].g = int_g;
        cloud->points[j].b = int_b;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
