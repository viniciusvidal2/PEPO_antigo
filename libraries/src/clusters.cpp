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
    seg.setDistanceThreshold (0.02);
    // Processar planos ate cansar
    PointCloud<PointTN>::Ptr temp (new PointCloud<PointTN>), plane (new PointCloud<PointTN>), cloud_f (new PointCloud<PointTN>);
    *temp = *in;
    int nr_points = (int) temp->points.size();
    while(temp->size() > 0.3*nr_points){ // Ainda podem haver planos significativos
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
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void Clusters::extractClusters(PointCloud<PointTN>::Ptr in, vector<PointCloud<PointTN>> &clust){

}
/////////////////////////////////////////////////////////////////////////////////////////////////
void Clusters::colorClouds(vector<PointCloud<PointTN>> &clouds){
    #pragma omp parallel for
    for(size_t i=0; i < clouds.size(); i++){
        // Definir cor de forma aleatoria
        int intensidade = (rand() % (230 - 40)) + 40;
        #pragma omp for num_threads(int(clouds[i].size()/10))
        for(size_t j=0; j < clouds[i].size(); j++){
            clouds[i].points[j].r = intensidade;
            clouds[i].points[j].g = intensidade;
            clouds[i].points[j].b = intensidade;
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
