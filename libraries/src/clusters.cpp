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
    seg.setMaxIterations(250);
    seg.setDistanceThreshold (0.03);
    // Processar planos ate cansar
    PointCloud<PointTN>::Ptr temp (new PointCloud<PointTN>), plane (new PointCloud<PointTN>), cloud_f (new PointCloud<PointTN>);
    *temp = *in;
    int nr_points = (int) temp->points.size();
    while(temp->size() > 0.6*nr_points){ // Ainda podem haver planos significativos
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
    normal_estimator.setKSearch(20);
    normal_estimator.compute(*normals);
    // Forcar virar as normais na marra para a origem
    Eigen::Vector3f C = Eigen::Vector3f::Zero();
    for(size_t i=0; i < normals->size(); i++){
        Eigen::Vector3f normal, cp;
        normal << normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z;
        cp     << C(0) - in->points[i].x     , C(1) - in->points[i].y     , C(2) - in->points[i].z     ;
        float cos_theta = (normal.dot(cp))/(normal.norm()*cp.norm());
        if(cos_theta <= 0){ // Esta apontando errado, deve inverter
            normals->points[i].normal_x = -normals->points[i].normal_x;
            normals->points[i].normal_y = -normals->points[i].normal_y;
            normals->points[i].normal_z = -normals->points[i].normal_z;
        }
    }
    // Iniciando o objeto de calculo da regiao e inserindo parametros
    RegionGrowing<PointTN, Normal> reg;
    reg.setSearchMethod(tree);
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(10000000);
    reg.setNumberOfNeighbours(50);
    reg.setInputCloud(in);
    reg.setInputNormals(normals);
    reg.setCurvatureThreshold(6.0);
    reg.setSmoothnessThreshold(20.0 / 180.0 * M_PI);
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
void Clusters::extractClustersRegionGrowingRGB(PointCloud<PointTN>::Ptr in, vector<PointCloud<PointTN>> &clust){
    // Criando a KdTree pra todos os metodos
    search::KdTree<PointTN>::Ptr tree (new search::KdTree<PointTN>);
    // Separando as normais de entrada da nuvem de uma vez
    PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
    NormalEstimation<PointTN, Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(in);
    normal_estimator.setKSearch(20);
    normal_estimator.compute(*normals);
    // Forcar virar as normais na marra para a origem
    Eigen::Vector3f C = Eigen::Vector3f::Zero();
    for(size_t i=0; i < normals->size(); i++){
        Eigen::Vector3f normal, cp;
        normal << normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z;
        cp     << C(0) - in->points[i].x     , C(1) - in->points[i].y     , C(2) - in->points[i].z     ;
        float cos_theta = (normal.dot(cp))/(normal.norm()*cp.norm());
        if(cos_theta <= 0){ // Esta apontando errado, deve inverter
            normals->points[i].normal_x = -normals->points[i].normal_x;
            normals->points[i].normal_y = -normals->points[i].normal_y;
            normals->points[i].normal_z = -normals->points[i].normal_z;
        }
    }
    // Iniciando o objeto de calculo da regiao e inserindo parametros
    RegionGrowingRGB<PointTN, Normal> reg;
    reg.setSearchMethod(tree);
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(10000000);
    reg.setNumberOfNeighbours(50);
    reg.setInputCloud(in);
    reg.setInputNormals(normals);
    reg.setCurvatureThreshold(1.0);
    reg.setSmoothnessThreshold(5.0 / 180.0 * M_PI);
    reg.setPointColorThreshold(5);
    reg.setRegionColorThreshold(5);
    reg.setDistanceThreshold(0.04);
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
void Clusters::extractClustersEuclidian(PointCloud<PointTN>::Ptr in, vector<PointCloud<PointTN>> &clust){
    // Cria arvore de busca e objeto de clusters por distancia euclidiana
    search::KdTree<PointTN>::Ptr tree (new search::KdTree<PointTN>);
    EuclideanClusterExtraction<PointTN> eucl;
    eucl.setInputCloud(in);
    eucl.setMaxClusterSize(int(in->size()*2));
    eucl.setMinClusterSize(int(in->size()/4));
    eucl.setClusterTolerance(0.05);
    eucl.setSearchMethod(tree);
    // Inicia vetor de clusters - pelo indice na nuvem
    vector<PointIndices> clusters_ind;
    eucl.extract(clusters_ind);
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
void Clusters::separateClustersByDistance(vector<PointCloud<PointTN> > &clust){
    // Criar vetor de nuvens interno para cada nuvem em cluster, aplicar o metodo
    vector<PointCloud<PointTN>> local, tempv;
    PointCloud<PointTN>::Ptr tempc (new PointCloud<PointTN>);
    ROS_INFO("A entrada possui %zu clusters.", clust.size());
    for(size_t i=0; i<clust.size(); i++){
        // Passa para a funcao de euclidean cluster a nuvem corespondente
        *tempc = clust[i];
        this->extractClustersEuclidian(tempc, tempv);
        ROS_INFO("O cluster %zu virou %zu clusters.", i+1, tempv.size());
        // Adiciona ao novo vetor local os resultados
        local.insert(local.end(), tempv.begin(), tempv.end());
    }
    ROS_INFO("Local saiu com %zu clusters.", local.size());
    // Forca o vetor global ser igual ao vetor local que foi separado
    clust.clear(); clust = local;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void Clusters::setColorPallete(size_t l){
    // De acordo com o tamanho, dividir o range de cores de 250 e colocar nos vetores
    pal_r.resize(l); pal_g.resize(l); pal_b.resize(l);
    for(size_t i=0; i < l; i++)
        pal_r[i] = pal_g[i] = pal_b[i] = int(20 + 230/l*i);
    // Bagunçar aqui para nao ficar tudo escala de cinza
    random_shuffle(pal_r.begin(), pal_r.end());
    random_shuffle(pal_g.begin(), pal_g.end());
    random_shuffle(pal_b.begin(), pal_b.end());
}

/////////////////////////////////////////////////////////////////////////////////////////////////
void Clusters::colorCloud(PointCloud<PointTN>::Ptr cloud, size_t i){
    // Colorir agora com a cor certa na paleta de cores
    #pragma omp for num_threads(int(cloud.size()/10))
    for(size_t j=0; j < cloud->size(); j++){
        cloud->points[j].r = pal_r[i];
        cloud->points[j].g = pal_g[i];
        cloud->points[j].b = pal_b[i];
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
