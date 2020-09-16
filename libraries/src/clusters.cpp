#include "../include/clusters.h"

/////////////////////////////////////////////////////////////////////////////////////////////////
Clusters::Clusters()
{

}
/////////////////////////////////////////////////////////////////////////////////////////////////
Clusters::~Clusters(){

}
/////////////////////////////////////////////////////////////////////////////////////////////////
void Clusters::obtainPlanes(PointCloud<PointT>::Ptr in, vector<PointCloud<PointT>> &planos, PointCloud<PointT>::Ptr out){
    // Cria coeficientes do modelo
    ModelCoefficients::Ptr coefficients (new ModelCoefficients);
    // Indices dos pontos que estao dentro
    PointIndices::Ptr inliers (new PointIndices);
    // Objeto para realizar segmentacao
    SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(SACMODEL_PLANE);
    seg.setMethodType(SAC_RANSAC);
    seg.setMaxIterations(150);
    seg.setDistanceThreshold(0.10);
    // Processar planos ate cansar
    PointCloud<PointT>::Ptr temp (new PointCloud<PointT>), plane (new PointCloud<PointT>), cloud_f (new PointCloud<PointT>);
    *temp = *in;
    int nr_points = (int) temp->points.size();
    int contador_iteracoes = 0;
    while(temp->size() > 0.6*nr_points && contador_iteracoes < 30){ // Ainda podem haver planos significativos
        seg.setInputCloud(temp);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0){
            ROS_WARN("Could not estimate a planar model for the given dataset, continuing...");
            break;
        }
        // Extract the planar inliers from the input cloud
        ExtractIndices<PointT> extract;
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
        contador_iteracoes++;
        ROS_INFO("Passou a iteracao %d na busca por planos ...", contador_iteracoes);
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
    #pragma omp parallel for
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
    reg.setMinClusterSize(in->size()/10);
    reg.setMaxClusterSize(10000000);
    reg.setNumberOfNeighbours(15);
    reg.setInputCloud(in);
    reg.setInputNormals(normals);
    reg.setCurvatureThreshold(5);
    reg.setSmoothnessThreshold(15.0 / 180.0 * M_PI);
    // Inicia vetor de clusters - pelo indice na nuvem
    vector<PointIndices> clusters_ind;
    reg.extract(clusters_ind);
    // Passa para o vetor de nuvens da rotina principal
    clust.resize(clusters_ind.size());
    #pragma omp parallel for
    for(size_t i=0; i<clusters_ind.size(); i++){
        PointIndices::Ptr temp (new PointIndices);
        *temp = clusters_ind[i];
        ExtractIndices<PointTN> extract;
        extract.setInputCloud(in);
        extract.setNegative(false);
        extract.setIndices(temp);
        extract.filter(clust[i]);
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void Clusters::extractClustersRegionGrowingRGB(PointCloud<PointT>::Ptr in, vector<PointCloud<PointT>> &clust){
    // Criando a KdTree pra todos os metodos
    search::KdTree<PointT>::Ptr tree (new search::KdTree<PointT>);
    // Separando as normais de entrada da nuvem de uma vez
    PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
    NormalEstimation<PointT, Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(in);
    normal_estimator.setKSearch(30);
    normal_estimator.compute(*normals);
    // Forcar virar as normais na marra para a origem
    Eigen::Vector3f C = Eigen::Vector3f::Zero();
    #pragma omp parallel for
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
    RegionGrowingRGB<PointT, Normal> reg;
    reg.setSearchMethod(tree);
    reg.setMinClusterSize(5);
    reg.setMaxClusterSize(10000000);
    reg.setNumberOfNeighbours(30);
    reg.setInputCloud(in);
    reg.setInputNormals(normals);
    reg.setCurvatureThreshold(0.5);
    reg.setSmoothnessThreshold(5.0 / 180.0 * M_PI);
    reg.setPointColorThreshold(40);
    reg.setRegionColorThreshold(50);
    reg.setDistanceThreshold(0.05);
    // Inicia vetor de clusters - pelo indice na nuvem
    vector<PointIndices> clusters_ind;
    reg.extract(clusters_ind);
    // Passa para o vetor de nuvens da rotina principal
    clust.resize(clusters_ind.size());
    #pragma omp parallel for
    for(size_t i=0; i<clusters_ind.size(); i++){
        PointIndices::Ptr temp (new PointIndices);
        *temp = clusters_ind[i];
        ExtractIndices<PointT> extract;
        extract.setInputCloud(in);
        extract.setNegative(false);
        extract.setIndices(temp);
        extract.filter(clust[i]);
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void Clusters::extractClustersEuclidian(PointCloud<PointTN>::Ptr in, vector<PointCloud<PointTN>> &clust){
    // Cria arvore de busca e objeto de clusters por distancia euclidiana
    search::KdTree<PointTN>::Ptr tree (new search::KdTree<PointTN>);
    EuclideanClusterExtraction<PointTN> eucl;
    eucl.setInputCloud(in);
    eucl.setMaxClusterSize(int(in->size()*10));
    eucl.setMinClusterSize(int(in->size()/10));
    eucl.setClusterTolerance(0.09);
    eucl.setSearchMethod(tree);
    // Inicia vetor de clusters - pelo indice na nuvem
    vector<PointIndices> clusters_ind;
    eucl.extract(clusters_ind);
    // Passa para o vetor de nuvens da rotina principal
    clust.resize(clusters_ind.size());
    #pragma omp parallel for
    for(size_t i=0; i<clusters_ind.size(); i++){
        PointIndices::Ptr temp (new PointIndices);
        *temp = clusters_ind[i];
        ExtractIndices<PointTN> extract;
        extract.setInputCloud(in);
        extract.setNegative(false);
        extract.setIndices(temp);
        extract.filter(clust[i]);
        temp->indices.clear();
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void Clusters::separateClustersByDistance(vector<PointCloud<PointT> > &clust){
    // Criar vetor de nuvens interno para cada nuvem em cluster, aplicar o metodo
    vector<PointCloud<PointT>> local, tempv;
    PointCloud<PointT>::Ptr tempc (new PointCloud<PointT>);
    for(size_t i=0; i<clust.size(); i++){
        // Passa para a funcao de euclidean cluster a nuvem corespondente
        *tempc = clust[i];
        this->extractClustersRegionGrowingRGB(tempc, tempv);
        // Adiciona ao novo vetor local os resultados
        local.insert(local.end(), tempv.begin(), tempv.end());
    }
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
    #pragma omp parallel for
    for(size_t j=0; j < cloud->size(); j++){
        cloud->points[j].r = pal_r[i];
        cloud->points[j].g = pal_g[i];
        cloud->points[j].b = pal_b[i];
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//void Clusters::adjustSmallClusters(vector<PointCloud<PointTN>> &clusters){
//    // Variaveis
//    vector<int> grande_ou_pequeno(clusters.size()); // 1 para grande, 0 para pequeno
//    float thresh = 0.1; // Limite entre o total da nuvem e o tamanho do clusters para ser grande
//    vector<Eigen::Vector4f> centroides(clusters.size());
//    size_t tamanho_total = 0;
//    // Calcular o centroide de cada cluster e somar o tamanho total da nuvem que deu origem
//    for (int i=0; i<grande_ou_pequeno.size(); i++) {
//        compute3DCentroid(clusters[i], centroides[i]);
//        tamanho_total += clusters[i].size();
//    }
//    // Segundo porcentagem, alterar e marcar como pequeno
//#pragma omp parallel for
//    for (int i=0; i < grande_ou_pequeno.size(); i++) {
//        if( float(clusters[i].size())/float(tamanho_total) > thresh )
//            grande_ou_pequeno[i] = 1;
//    }
//    // Calcular distancia do cluster mais proximo que nao e pequeno e adicionar
//    for (int i=0; i < grande_ou_pequeno.size(); i++) {
//        int indice_escolhido;
//        float dist_min = 1000000;
//        // Se for cluster pequeno
//        if(grande_ou_pequeno[i] == 0){
//            // Varre o vetor de novo
//            for(int j=0; j < grande_ou_pequeno.size(); j++){
//                // Se for cluster grande, calcula distancia
//                if(grande_ou_pequeno[j] == 1){
//                    float dist = sqrt( pow(centroides[i](0) - centroides[j](0), 2) +
//                                       pow(centroides[i](1) - centroides[j](1), 2) +
//                                       pow(centroides[i](2) - centroides[j](2), 2) );
//                    if(dist < dist_min){
//                        dist_min = dist;
//                        indice_escolhido = j;
//                    }
//                }
//            }
//            // Adicionar no indice j encontrado como indice_escolhido
//            clusters[indice_escolhido] += clusters[i];
//        }
//    }
//    // Salva todos os clusters grandes no vetor de saida e substitui o vetor de entrada
//    vector<PointCloud<PointTN>> temp;
//    for(int i=0; i<grande_ou_pequeno.size(); i++){
//        if(grande_ou_pequeno[i] == 1)
//            temp.push_back(clusters[i]);
//    }

//    clusters.clear(); clusters = temp;
//}
/////////////////////////////////////////////////////////////////////////////////////////////////
void Clusters::killSmallClusters(vector<PointCloud<PointT>> &clusters, float pct_over_mean){
    // Variaveis
    vector<Eigen::Vector4f> centroides(clusters.size());
    float media, soma = 0;
    vector<int> grande_ou_pequeno(clusters.size()); // 1 para grande, 0 para pequeno
    // Tira media e desvio padrao do tamanho dos clusters, calcula centroide tambem
    for (int i=0; i<clusters.size(); i++) {
        compute3DCentroid(clusters[i], centroides[i]);
        soma += float(clusters[i].size());
    }
    media = soma/float(clusters.size());
    // Se estiver menor que media menos tantos desvios marca
    #pragma omp parallel for
    for(int i=0; i < clusters.size(); i++){
        if(clusters[i].size() > media*pct_over_mean)
            grande_ou_pequeno[i] = 1;
    }
    // Calcular distancia do cluster mais proximo que nao e pequeno e adicionar
    for (int i=0; i < grande_ou_pequeno.size(); i++) {
        int indice_escolhido;
        float dist_min = 1000000;
        // Se for cluster pequeno
        if(grande_ou_pequeno[i] == 0){
            // Varre o vetor de novo
            for(int j=0; j < grande_ou_pequeno.size(); j++){
                // Se for cluster grande, calcula distancia
                if(grande_ou_pequeno[j] == 1){
                    float dist = sqrt( pow(centroides[i](0) - centroides[j](0), 2) +
                                       pow(centroides[i](1) - centroides[j](1), 2) +
                                       pow(centroides[i](2) - centroides[j](2), 2) );
                    if(dist < dist_min){
                        dist_min = dist;
                        indice_escolhido = j;
                    }
                }
            }
            // Adicionar no indice j encontrado como indice_escolhido
            clusters[indice_escolhido] += clusters[i];
        }
    }
    // Salva todos os clusters grandes no vetor de saida e substitui o vetor de entrada
    vector<PointCloud<PointT>> temp;
    for(int i=0; i<grande_ou_pequeno.size(); i++){
        if(grande_ou_pequeno[i] == 1)
            temp.push_back(clusters[i]);
    }

    clusters.clear(); clusters = temp;
}
