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
void Clusters::extractClustersRegionGrowing(PointCloud<PointTN>::Ptr in, vector<PointCloud<PointTN>> &clust){
    // Criando a KdTree pra todos os metodos
    search::KdTree<PointTN>::Ptr tree (new search::KdTree<PointTN>);
    // Separando as normais de entrada da nuvem de uma vez
    PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
    NormalEstimation<PointTN, Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(in);
    normal_estimator.setKSearch(10);
    normal_estimator.compute(*normals);
    // Iniciando o objeto de calculo da regiao e inserindo parametros
    RegionGrowing<PointTN, Normal> reg;
    reg.setSearchMethod(tree);
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(10000000);
    reg.setNumberOfNeighbours(50);
    reg.setInputCloud(in);
    reg.setInputNormals(normals);
    reg.setCurvatureThreshold(60.0);
    reg.setSmoothnessThreshold(30.0 / 180.0 * M_PI);
    // Inicia vetor de clusters - pelo indice na nuvem
    vector<PointIndices> clusters_ind;
    reg.extract(clusters_ind);
    // Passa para o vetor de nuvens da rotina principal
    clust.resize(clusters_ind.size());
    ExtractIndices<PointTN> extract;
    extract.setInputCloud(in);
    extract.setNegative(false);
    PointIndices::Ptr temp (new PointIndices);
    #pragma omp parallel for
    for(size_t i=0; i<clusters_ind.size(); i++){
        *temp = clusters_ind[i];
        extract.setIndices(temp);
        extract.filter(clust[i]);
        temp->indices.clear();
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void Clusters::colorCloud(vector<PointCloud<PointTN>> &clouds){
    for(size_t j=0; j<clouds.size(); j++){
        // Definir cor de forma aleatoria
        int int_r = int(int((rand()) % (250 - 40)) + 40);
        int int_g = int(int((rand()) % (250 - 40)) + 40);
        int int_b = int(int((rand()) % (250 - 40)) + 40);
        #pragma omp for num_threads(int(cloud.size()/10))
        for(size_t j=0; j < clouds.size(); j++){
            clouds[j].points[j].r = int_r;
            clouds[j].points[j].g = int_g;
            clouds[j].points[j].b = int_b;
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
