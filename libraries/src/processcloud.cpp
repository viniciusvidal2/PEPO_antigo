#include "../include/processcloud.h"

/////////////////////////////////////////////////////////////////////////////////////////////////
ProcessCloud::ProcessCloud(string p):pasta(p)
{
    // Matriz intrinseca com imagem em Full HD
    K1 << 1427.1  ,   -0.063, 987.9,
             0.041, 1449.4  , 579.4,
             0    ,    0    ,   1  ;
    // Matriz intrinseca com imagem em HD
    K2 << 951.30,   0   , 1280.0/2,
            0   , 966.26,  720.0/2,
            0   ,   0   ,    1    ;
    // Matriz intrinseca com imagem em resolucao simplificada por 4
    K4 << 375.29  ,   0.0472, 241.18,
            0.0157, 374.50  , 137.36,
            0     ,   0     ,   1   ;
    // Matriz extrinseca com imagem em Full HD
    Rt1.resize(3, 4);
    Rt1 << 1, 0, 0,  0.0077,
           0, 1, 0,  0.0329,
           0, 0, 1,  0.0579;
    // Matriz extrinseca com imagem em resolucao simplificada por 4
    Rt4.resize(3, 4);
    Rt4 << 1, 0, 0,  0.0226,
           0, 1, 0,  0.0938,
           0, 0, 1,  0.0221;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
ProcessCloud::~ProcessCloud(){
    ros::shutdown();
    ros::waitForShutdown();
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::calculateNormals(PointCloud<PointT>::Ptr in, PointCloud<PointTN>::Ptr acc_normal){
    // Vetor de zeros simbolizando a origem
    Eigen::Vector3f C = Eigen::Vector3f::Zero();
    // Remove pontos nan aqui para nao correr risco de derrubar o calculo das normais
    std::vector<int> indicesnan;
    removeNaNFromPointCloud<PointTN>(*acc_normal, *acc_normal, indicesnan);
    // Inicia estimador de normais
    search::KdTree<PointT>::Ptr tree (new search::KdTree<PointT>());
    NormalEstimationOMP<PointT, Normal> ne;
    ne.setInputCloud(in);
    ne.setSearchMethod(tree);
    ne.setKSearch(100);
    ne.setNumberOfThreads(20);
    // Nuvem de normais calculada
    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>());
    ne.compute(*cloud_normals);
    // Adiciona saida na nuvem concatenada PointTN
    concatenateFields(*in, *cloud_normals, *acc_normal);
    // Filtra por normais problematicas
    removeNaNNormalsFromPointCloud(*acc_normal, *acc_normal, indicesnan);

    // Forcar virar as normais na marra para a origem
#pragma omp parallel for
    for(size_t i=0; i < acc_normal->size(); i++){
        Eigen::Vector3f normal, cp;
        normal << acc_normal->points[i].normal_x, acc_normal->points[i].normal_y, acc_normal->points[i].normal_z;
        cp     << C(0)-acc_normal->points[i].x  , C(1)-acc_normal->points[i].y  , C(2)-acc_normal->points[i].z  ;
        float cos_theta = (normal.dot(cp))/(normal.norm()*cp.norm());
        if(cos_theta <= 0){ // Esta apontando errado, deve inverter
            acc_normal->points[i].normal_x = -acc_normal->points[i].normal_x;
            acc_normal->points[i].normal_y = -acc_normal->points[i].normal_y;
            acc_normal->points[i].normal_z = -acc_normal->points[i].normal_z;
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::calculateNormals(PointCloud<PointTN>::Ptr acc_normal){
    // Vetor de zeros simbolizando a origem
    Eigen::Vector3f C = Eigen::Vector3f::Zero();
    // Remove pontos nan aqui para nao correr risco de derrubar o calculo das normais
    std::vector<int> indicesnan;
    removeNaNFromPointCloud<PointTN>(*acc_normal, *acc_normal, indicesnan);
    removeNaNNormalsFromPointCloud<PointTN>(*acc_normal, *acc_normal, indicesnan);
    // Inicia estimador de normais
    search::KdTree<PointTN>::Ptr tree (new search::KdTree<PointTN>());
    NormalEstimationOMP<PointTN, PointTN> ne;
    ne.setInputCloud(acc_normal);
    ne.setSearchMethod(tree);
    if(acc_normal->size() > 50) ne.setKSearch(100); else ne.setKSearch(3);
    ne.setNumberOfThreads(20);
    ne.compute(*acc_normal);
    removeNaNNormalsFromPointCloud<PointTN>(*acc_normal, *acc_normal, indicesnan);

    // Forcar virar as normais na marra para a origem
    #pragma omp parallel for
    for(size_t i=0; i < acc_normal->size(); i++){
        Eigen::Vector3f normal, cp;
        normal << acc_normal->points[i].normal_x, acc_normal->points[i].normal_y, acc_normal->points[i].normal_z;
        cp     << C(0)-acc_normal->points[i].x  , C(1)-acc_normal->points[i].y  , C(2)-acc_normal->points[i].z  ;
        float cos_theta = (normal.dot(cp))/(normal.norm()*cp.norm());
        if(cos_theta <= 0){ // Esta apontando errado, deve inverter
            acc_normal->points[i].normal_x = -acc_normal->points[i].normal_x;
            acc_normal->points[i].normal_y = -acc_normal->points[i].normal_y;
            acc_normal->points[i].normal_z = -acc_normal->points[i].normal_z;
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::transformToCameraFrame(PointCloud<PointTN>::Ptr nuvem){
    // Rotacionar a nuvem para cair no frame da câmera (laser tem X para frente, câmera deve ter
    // Z para frente e X para o lado
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitY());
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();     // Matriz de transformaçao homogenea
    T.block<3, 3>(0, 0) = R;                             // Adiciona a rotacao onde deve estar
    transformPointCloudWithNormals(*nuvem, *nuvem, T);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::transformToCameraFrame(PointCloud<PointT>::Ptr nuvem){
    // Rotacionar a nuvem para cair no frame da câmera (laser tem X para frente, câmera deve ter
    // Z para frente e X para o lado
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitY());
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();     // Matriz de transformaçao homogenea
    T.block<3, 3>(0, 0) = R;                             // Adiciona a rotacao onde deve estar
    transformPointCloud(*nuvem, *nuvem, T);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::colorCloudWithCalibratedImage(PointCloud<PointT>::Ptr cloud_in, Mat image, float scale){
    // Matriz da camera segundo escala da imagem
    MatrixXf P(3, 4);
    if(scale == 1)
        P = K1*Rt1;
    else if(scale == 4)
        P = K4*Rt4;

    for(size_t i = 0; i < cloud_in->size(); i++){
        // Pegar ponto em coordenadas homogeneas
        MatrixXf X_(4, 1);
        X_ << cloud_in->points[i].x,
              cloud_in->points[i].y,
              cloud_in->points[i].z,
                        1          ;
        MatrixXf X(3, 1);
        X = P*X_;
        if(X(2, 0) > 0){
            X = X/X(2, 0);
            // Colorindo ponto se for o caso de projetado corretamente
            if(floor(X(0,0)) > 0 && floor(X(0,0)) < image.cols && floor(X(1,0)) > 0 && floor(X(1,0)) < image.rows){
                cv::Vec3b cor = image.at<Vec3b>(Point(X(0,0), X(1,0)));
                PointT point = cloud_in->points[i];
                point.b = cor.val[0]; point.g = cor.val[1]; point.r = cor.val[2];
                cloud_in->points[i] = point;
            }
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::colorCloudWithCalibratedImage(PointCloud<PointTN>::Ptr cloud_in, Mat image, float scale){
    // Matriz da camera segundo escala da imagem
    MatrixXf P(3, 4);
    if(scale == 1)
        P = K1*Rt1;
    else if(scale == 4)
        P = K4*Rt4;

    for(size_t i = 0; i < cloud_in->size(); i++){
        // Pegar ponto em coordenadas homogeneas
        MatrixXf X_(4, 1);
        X_ << cloud_in->points[i].x,
              cloud_in->points[i].y,
              cloud_in->points[i].z,
                        1          ;
        MatrixXf X(3, 1);
        X = P*X_;
        if(X(2, 0) > 0){
            X = X/X(2, 0);
            // Adicionando ponto na imagem se for o caso de projetado corretamente
            if(floor(X(0,0)) > 0 && floor(X(0,0)) < image.cols && floor(X(1,0)) > 0 && floor(X(1,0)) < image.rows){
                cv::Vec3b cor = image.at<Vec3b>(Point(X(0,0), X(1,0)));
                PointTN point = cloud_in->points[i];
                point.b = cor.val[0]; point.g = cor.val[1]; point.r = cor.val[2];
                cloud_in->points[i] = point;
            }
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::filterCloudDepthCovariance(PointCloud<PointT>::Ptr cloud, int kn, float thresh, float depth){
    // Definir area a filtrar pela distancia dos pontos
    PointCloud<PointT>::Ptr close_area (new PointCloud<PointT>);
    PointCloud<PointT>::Ptr far_area   (new PointCloud<PointT>);
    for(size_t i=0; i<cloud->size(); i++){
        if(sqrt(pow(cloud->points[i].x, 2) + pow(cloud->points[i].y, 2) + pow(cloud->points[i].z, 2)) > depth)
            far_area->push_back(cloud->points[i]);
        else
            close_area->push_back(cloud->points[i]);
    }
    // Se houver area proxima relevante, calcular
    if(close_area->size() > 300){
        // Nuvens para filtrar
        PointCloud<PointT>::Ptr temp_out        (new PointCloud<PointT>);
        PointCloud<PointT>::Ptr marcar_outliers (new PointCloud<PointT>);
        marcar_outliers->resize(close_area->size());
        // Objetos para procurar na nuvem o centroide e a covariancia
        KdTreeFLANN<PointT>::Ptr tree (new KdTreeFLANN<PointT>);
        tree->setInputCloud(close_area);
        // Varrer todos os pontos atras da covariancia
#pragma omp parallel for
        for(size_t i=0; i<close_area->size(); i++){
            // Cria variaveis aqui dentro pelo processo ser paralelizado
            Eigen::Vector4f centroide;
            Eigen::Matrix4f rotacao_radial;
            Eigen::Matrix3f covariancia;
            rotacao_radial = Matrix4f::Identity();
            // Calcula angulo para rotacionar a nuvem e cria matriz de rotacao (yaw em torno de Y, pitch em torno de X)
            float yaw_y   = atan2( close_area->points[i].x, close_area->points[i].z);
            float pitch_x = atan2(-close_area->points[i].y, close_area->points[i].z);
            Eigen::Matrix3f rot = this->euler2matrix(0, pitch_x, yaw_y);
            rotacao_radial.block<3,3>(0, 0) << rot;
            // Calcula vizinhos mais proximos aqui por raio ou K neighbors
            vector<int> indices_vizinhos;
            vector<float> distancias_vizinhos;
            tree->nearestKSearch(int(i), kn, indices_vizinhos, distancias_vizinhos);
            // Separa nuvem com esses vizinhos
            PointCloud<PointT>::Ptr temp (new PointCloud<PointT>);
            temp->resize(indices_vizinhos.size());
#pragma omp parallel for
            for(size_t j=0; j<indices_vizinhos.size(); j++)
                temp->points[j] = close_area->points[ indices_vizinhos[j] ];
            // Rotaciona a nuvem separada segundo o raio que sai do centro do laser (origem)
            transformPointCloud(*temp, *temp, rotacao_radial);
            // Calcula centroide e covariancia da nuvem
            compute3DCentroid(*temp, centroide);
            computeCovarianceMatrix(*temp, centroide, covariancia);
            // Se for muito maior em z que em x e y, considera ruim e marca na nuvem
            if(covariancia(2, 2) > thresh*covariancia(0, 0) && covariancia(2, 2) > thresh*covariancia(1, 1))
                marcar_outliers->points[i].x = 1;
            else
                marcar_outliers->points[i].x = 0;
        }
        // Passa rapidamente para nuvem de saida
        for(size_t i=0; i<marcar_outliers->size(); i++){
            if(marcar_outliers->points[i].x == 0)
                temp_out->push_back(close_area->points[i]);
        }
        marcar_outliers->clear();
        *close_area = *temp_out;
        temp_out->clear();
        *cloud = *close_area + *far_area;
        close_area->clear(); far_area->clear();
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::filterCloudDepthCovariance(PointCloud<PointTN>::Ptr cloud, int kn, float thresh, float depth){
    // Definir area a filtrar pela distancia dos pontos
    PointCloud<PointTN>::Ptr close_area (new PointCloud<PointTN>);
    PointCloud<PointTN>::Ptr far_area   (new PointCloud<PointTN>);
    for(size_t i=0; i<cloud->size(); i++){
        if(sqrt(pow(cloud->points[i].x, 2) + pow(cloud->points[i].y, 2) + pow(cloud->points[i].z, 2)) > depth)
            far_area->push_back(cloud->points[i]);
        else
            close_area->push_back(cloud->points[i]);
    }
    // Se houver area proxima relevante, calcular
    if(close_area->size() > 300){
        // Nuvens para filtrar
        PointCloud<PointTN>::Ptr temp_out        (new PointCloud<PointTN>);
        PointCloud<PointTN>::Ptr marcar_outliers (new PointCloud<PointTN>);
        marcar_outliers->resize(close_area->size());
        // Objetos para procurar na nuvem o centroide e a covariancia
        KdTreeFLANN<PointTN>::Ptr tree (new KdTreeFLANN<PointTN>);
        tree->setInputCloud(close_area);
        // Varrer todos os pontos atras da covariancia
#pragma omp parallel for
        for(size_t i=0; i<close_area->size(); i++){
            // Cria variaveis aqui dentro pelo processo ser paralelizado
            Eigen::Vector4f centroide;
            Eigen::Matrix4f rotacao_radial;
            Eigen::Matrix3f covariancia;
            rotacao_radial = Matrix4f::Identity();
            // Calcula angulo para rotacionar a nuvem e cria matriz de rotacao (yaw em torno de Y, pitch em torno de X)
            float yaw_y   = atan2( close_area->points[i].x, close_area->points[i].z);
            float pitch_x = atan2(-close_area->points[i].y, close_area->points[i].z);
            Eigen::Matrix3f rot = this->euler2matrix(0, pitch_x, yaw_y);
            rotacao_radial.block<3,3>(0, 0) << rot;
            // Calcula vizinhos mais proximos aqui por raio ou K neighbors
            vector<int> indices_vizinhos;
            vector<float> distancias_vizinhos;
            tree->nearestKSearch(int(i), kn, indices_vizinhos, distancias_vizinhos);
            // Separa nuvem com esses vizinhos
            PointCloud<PointTN>::Ptr temp (new PointCloud<PointTN>);
            temp->resize(indices_vizinhos.size());
#pragma omp parallel for
            for(size_t j=0; j<indices_vizinhos.size(); j++)
                temp->points[j] = close_area->points[ indices_vizinhos[j] ];
            // Rotaciona a nuvem separada segundo o raio que sai do centro do laser (origem)
            transformPointCloud(*temp, *temp, rotacao_radial);
            // Calcula centroide e covariancia da nuvem
            compute3DCentroid(*temp, centroide);
            computeCovarianceMatrix(*temp, centroide, covariancia);
            // Se for muito maior em z que em x e y, considera ruim e marca na nuvem
            if(covariancia(2, 2) > thresh*covariancia(0, 0) && covariancia(2, 2) > thresh*covariancia(1, 1))
                marcar_outliers->points[i].x = 1;
            else
                marcar_outliers->points[i].x = 0;
        }
        // Passa rapidamente para nuvem de saida
        for(size_t i=0; i<marcar_outliers->size(); i++){
            if(marcar_outliers->points[i].x == 0)
                temp_out->push_back(close_area->points[i]);
        }
        marcar_outliers->clear();
        *close_area = *temp_out;
        temp_out->clear();
        *cloud = *close_area + *far_area;
        close_area->clear(); far_area->clear();
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::preprocess(PointCloud<PointT>::Ptr cin, PointCloud<PointTN>::Ptr out, float vs, float d, int fp,
                              float &tempo_cor, float &tempo_octree, float &tempo_demaisfiltros, float &tempo_normais){
    // Retirando pontos que vem do edge nao projetados ou com erro
    ros::Time tempo = ros::Time::now();
    this->removeNotProjectedThroughDefinedColor(cin, 200, 200, 200);
    std::vector<int> indicesnan;
    removeNaNFromPointCloud<PointT>(*cin, *cin, indicesnan);
    // Filtro de voxels para aliviar a entrada
    if(vs > 0){
        VoxelGrid<PointT> voxel;
        voxel.setLeafSize(vs, vs, vs);
        voxel.setInputCloud(cin);
        voxel.filter(*cin);
    }
    // Filtro de profundidade para nao pegarmos muito fundo
    PassThrough<PointT> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, d); // Z metros de profundidade
    pass.setInputCloud(cin);
    pass.filter(*cin);
    tempo_cor = (ros::Time::now() - tempo).toSec();
    tempo = ros::Time::now();
    // Separando a nuvem de entrada em clusters
    vector<PointCloud<PointT>> cin_clusters;
    this->divideInOctreeLevels(cin, cin_clusters, 2);
    // Continuar separando enquanto houver algum maior que 30 mil pontos
    bool so_pequenos = false;
    size_t indice_grande;
    PointCloud<PointT>::Ptr nuvem_grande (new PointCloud<PointT>);
    vector<PointCloud<PointT>> big_clusters;
    while(!so_pequenos){
        so_pequenos = true;
        // Se achar algum cluster grande ainda, separar indice e nuvem correspondente
        for(size_t i=0; i<cin_clusters.size(); i++){
            if(cin_clusters[i].size() > 20000){
                so_pequenos = false;
                indice_grande = i;
                *nuvem_grande = cin_clusters[i];
                break;
            }
            // Matar cluster pequeno
            if(cin_clusters[i].size() < 200) cin_clusters[i].clear();
        }
        // Se foi achado algum cluster grande, processar ele, substituir o original com o primeiro cluster obtido e adicionar
        // o restante dos clusters ao final da nuvem de clusters
        if(!so_pequenos && nuvem_grande->size() > 0){
            this->divideInOctreeLevels(nuvem_grande, big_clusters, 2);
            cin_clusters[indice_grande] = big_clusters[0];
            cin_clusters.insert(cin_clusters.end(), big_clusters.begin()+1, big_clusters.end());
            big_clusters.clear(); nuvem_grande->clear();
        }
    }
    tempo_octree = (ros::Time::now() - tempo).toSec();
    // Para cada cluster, filtrar e retornar no vetor de filtradas
    vector<PointCloud<PointTN>> out_clusters(cin_clusters.size());
    vector<float> tempos_demaisfiltros(cin_clusters.size()), tempos_normais(cin_clusters.size());
#pragma omp parallel for
    for(size_t i=0; i<out_clusters.size(); i++){
        ros::Time tempo2 = ros::Time::now();
        PointCloud<PointT>::Ptr temp (new PointCloud<PointT>);
        *temp = cin_clusters[i];
        // Filtro de ruidos aleatorios
        StatisticalOutlierRemoval<PointT> sor;
        sor.setMeanK(30);
        sor.setStddevMulThresh(2);
        sor.setNegative(false);
        sor.setInputCloud(temp);
        sor.filter(*temp);
        // Calcular filtro de covariancia na regiao mais proxima
        float depth_cov_filter = 5;
        this->filterCloudDepthCovariance(temp, 100, 2.0, depth_cov_filter);
        tempos_demaisfiltros[i] = (ros::Time::now() - tempo2).toSec();
        tempo2 = ros::Time::now();
        // Polinomio
        PointCloud<PointTN>::Ptr filt_normal (new PointCloud<PointTN>);
        if(fp == 1){
            // Passando polinomio pra suavizar a parada
            search::KdTree<PointT>::Ptr tree (new search::KdTree<PointT>);
            MovingLeastSquares<PointT, PointTN> mls;
            mls.setComputeNormals(true);
            mls.setInputCloud(temp);
            mls.setPolynomialOrder(1);
            mls.setSearchMethod(tree);
            mls.setSearchRadius(0.05);
            mls.process(*filt_normal);
        } else {
            PointCloud<Normal>::Ptr normals (new PointCloud<Normal>);
            normals->resize(temp->size());
            concatenateFields(*temp, *normals, *filt_normal);
        }
        // Normais apos tudo filtrado
        this->calculateNormals(filt_normal);        
        out_clusters[i] = *filt_normal;
        tempos_normais[i] = (ros::Time::now() - tempo2).toSec();
    }
    // Somar todos os clusters filtrados de volta na nuvem de saida
    for(size_t i=0; i<out_clusters.size(); i++)
        *out += out_clusters[i];

    tempo_demaisfiltros = *max_element(tempos_demaisfiltros.begin(), tempos_demaisfiltros.end());
    tempo_normais = *max_element(tempos_normais.begin(), tempos_normais.end());
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::divideInOctreeLevels(PointCloud<PointT>::Ptr cloud, vector<PointCloud<PointT>> &leafs, float level){
    /// Obter os limites de dimensao da nuvem de entrada
    PointT min_limits, max_limits;
    getMinMax3D(*cloud, min_limits, max_limits);
    // Se a nuvem variar consideravelmente em todas as dimensoes, aumentar o level automaticamente
    float dl = 5; // [m]
    if(abs(max_limits.x - min_limits.x) > dl && abs(max_limits.x - min_limits.x) > dl && abs(max_limits.x - min_limits.x) > dl)
        level *= 2;
    /// De acordo com a quantidade de niveis da octree, calcular os centroides das folhas da arvore
    // Dimensoes da caixa que corresponde a folha
    float stepx, stepy, stepz;
    stepx = abs(max_limits.x - min_limits.x)/level;
    stepy = abs(max_limits.y - min_limits.y)/level;
    stepz = abs(max_limits.z - min_limits.z)/level;
    // Centros em que vamos caminhar naquela dimensao para cada folha
    vector<float> centros_x, centros_y, centros_z;
    // Se temos bastante variacao, dividir, senao mantem somente uma divisao ali na dimensao
    float tol = 0.1; // [m]
    if(stepx > tol)
        centros_x.resize(size_t(level));
    else
        centros_x.resize(size_t(level/2));
    if(stepy > tol)
        centros_y.resize(size_t(level));
    else
        centros_y.resize(size_t(level/2));
    if(stepz > tol)
        centros_z.resize(size_t(level));
    else
        centros_z.resize(size_t(level/2));
    for(int i=0; i<centros_x.size(); i++)
        centros_x[i] = min_limits.x + stepx/2 + float(i)*stepx;
    for(int i=0; i<centros_y.size(); i++)
        centros_y[i] = min_limits.y + stepy/2 + float(i)*stepy;
    for(int i=0; i<centros_z.size(); i++)
        centros_z[i] = min_limits.z + stepz/2 + float(i)*stepz;
    // Montar a nuvem de pontos com os centroides combinados
    PointCloud<PointT>::Ptr centroides (new PointCloud<PointT>);
    for(int i=0; i<centros_x.size(); i++){
        for(int j=0; j<centros_y.size(); j++){
            for(int k=0; k<centros_z.size(); k++){
                PointT c;
                c.x = centros_x[i]; c.y = centros_y[j]; c.z = centros_z[k];
                c.r = 250; c.b = 0; c.g = 0;
                centroides->push_back(c);
            }
        }
    }

    // Colocar o numero de folhas como o tamanho do vetor de saida
    leafs.resize(centroides->size());
    /// Iterar sobre a nuvem para colocar cada ponto na sua folha
    /// Descobrir qual centroide esta mais perto por KdTree
    KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(centroides);
    vector<int> indices;
    vector<float> distances;
    for(size_t i=0; i<cloud->size(); i++){
        kdtree.nearestKSearch(cloud->points[i], 1, indices, distances);
        if(indices.size() == 1 && indices[0] >= 0 && indices[0] < centroides->size())
            leafs[indices[0]].push_back(cloud->points[i]);
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::removeNotProjectedThroughDefinedColor(PointCloud<PointT>::Ptr cloud, int r, int g, int b){
    ExtractIndices<PointT> extract;
    PointIndices::Ptr indices (new PointIndices);
    for(size_t i=0; i<cloud->size(); i++){
        if((*cloud)[i].r == r && (*cloud)[i].g == g && (*cloud)[i].b == b)
            indices->indices.push_back(i);
    }
    extract.setInputCloud(cloud);
    extract.setNegative(true);
    extract.setIndices(indices);
    extract.filter(*cloud);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::saveCloud(PointCloud<PointTN>::Ptr nuvem, std::string nome){
    std::string nome_nuvem = pasta + nome + ".ply";
    savePLYFileBinary<PointTN>(nome_nuvem, *nuvem);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::saveCloud(PointCloud<PointT>::Ptr nuvem, std::string nome){
    std::string nome_nuvem = pasta + nome + ".ply";
    savePLYFileBinary<PointT>(nome_nuvem, *nuvem);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::saveImage(cv::Mat img, string nome){
    std::string final = pasta + nome + ".png";
    vector<int> params;
    params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    params.push_back(9);
    cv::imwrite(final, img, params);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
Matrix3f ProcessCloud::euler2matrix(float r, float p, float y){
    // Ja recebe os angulos aqui em radianos
    return (AngleAxisf(y, Vector3f::UnitY()) * AngleAxisf(p, Vector3f::UnitX())).matrix();
}
/////////////////////////////////////////////////////////////////////////////////////////////////
std::string ProcessCloud::escreve_linha_nvm(float foco, std::string nome, Eigen::MatrixXf C, Eigen::Quaternion<float> q){
    std::string linha = pasta+nome;
    // Adicionar foco
    linha = linha + " " + std::to_string(foco);
    // Adicionar quaternion
    linha = linha + " " + std::to_string(q.w()) + " " + std::to_string(q.x()) + " " + std::to_string(q.y()) + " " + std::to_string(q.z());
    // Adicionar centro da camera
    linha = linha + " " + std::to_string(C(0, 0)) + " " + std::to_string(C(1, 0)) + " " + std::to_string(C(2, 0));
    // Adicionar distorcao radial (crendo 0) e 0 final
    linha = linha + " 0 0\n"; // IMPORTANTE pular linha aqui, o MeshRecon precisa disso no MART
    // Muda as virgulas por pontos no arquivo
    std::replace(linha.begin(), linha.end(), ',', '.');
    return linha;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
std::string ProcessCloud::escreve_linha_sfm(string nome, Matrix3f r, Vector3f t){
    string linha = pasta+nome;
    // Adicionar matriz de rotacao linha 1
    linha = linha + " " + std::to_string(r(0, 0)) + " " + std::to_string(r(0, 1)) + " " + std::to_string(r(0, 2));
    // Adicionar matriz de rotacao linha 2
    linha = linha + " " + std::to_string(r(1, 0)) + " " + std::to_string(r(1, 1)) + " " + std::to_string(r(1, 2));
    // Adicionar matriz de rotacao linha 3
    linha = linha + " " + std::to_string(r(2, 0)) + " " + std::to_string(r(2, 1)) + " " + std::to_string(r(2, 2));
    // Adicionar vetor de translacao
    linha = linha + " " + std::to_string(t(0)) + " " + std::to_string(t(1)) + " " + std::to_string(t(2));
    // Adicionar foco x e y como na matriz da camera em resolucao HD
    linha = linha + " " + std::to_string(K2(0, 0)) + " " + std::to_string(K2(1, 1));
    // Adicionar centro optico em x e y como na matriz da camera em resolucao HD
    linha = linha + " " + std::to_string(K2(0, 2)) + " " + std::to_string(K2(1, 2));
    // Adicionando quebra de linha - padrao do MART
    linha = linha + "\n";
    // Muda as virgulas por pontos no arquivo, caso existam
    std::replace(linha.begin(), linha.end(), ',', '.');
    return linha;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::compileFinalNVM(vector<string> linhas){
    // Se ja existe o arquivo, deletar para sobreescrever
    struct stat buffer;
    string nome_nvm_final = pasta + "cameras.nvm";
    if(!stat(nome_nvm_final.c_str(), &buffer)){
        if(remove(nome_nvm_final.c_str()) == 0)
            ROS_INFO("Deletamos NVM anterior.");
        else
            ROS_ERROR("NVM final anterior nao foi deletado.");
    }
    // Anota num arquivo a partir do nome vindo
    ofstream nvm(nome_nvm_final);
    if(nvm.is_open()){

        nvm << "NVM_V3\n\n";
        nvm << std::to_string(linhas.size())+"\n"; // Quantas imagens
        for(int i=0; i < linhas.size(); i++)
            nvm << linhas[i]; // Imagem com detalhes de camera

    } // fim do if is open
    nvm.close(); // Fechar para nao ter erro
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::compileFinalSFM(vector<string> linhas){
    // Se ja existe o arquivo, deletar para sobreescrever
    struct stat buffer;
    string nome_sfm_final = pasta + "cameras.sfm";
    if(!stat(nome_sfm_final.c_str(), &buffer)){
        if(remove(nome_sfm_final.c_str()) == 0)
            ROS_INFO("Deletamos SFM anterior.");
        else
            ROS_ERROR("SFM final anterior nao foi deletado.");
    }
    // Anota num arquivo a partir do nome vindo
    ofstream sfm(nome_sfm_final);
    if(sfm.is_open()){

        sfm << std::to_string(linhas.size())+"\n\n"; // Quantas imagens
        for(int i=0; i < linhas.size(); i++)
            sfm << linhas[i]; // Imagem com detalhes de camera

    } // fim do if is open
    sfm.close(); // Fechar para nao ter erro
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::transformCloudAndCamServoAngles(PointCloud<PointT>::Ptr cloud, float pan, float tilt, Vector3f &C, Quaternion<float> &q){
    /// Cria matriz de rotacao de acordo com angulos de pan e tilt
    // Angulos chegam em DEGREES - passar para RAD aqui
    // Pan - Yaw em torno de Y, negativo; Tilt - pitch em torno de X, negativo
    // Considerar primeiro o pan, depois deslocar em tilt
    pan = DEG2RAD(pan); tilt = DEG2RAD(tilt);

    // Avanca a frente no eixo Z desde o eixo de rotacao
    Matrix3f Rp = euler2matrix(0, 0, -pan);
    Vector3f tp(0, 0, 0.0);
    tp = Rp*tp;
    // Finaliza a matriz homogenea e transformar a nuvem
    Matrix4f Tp = Matrix4f::Identity();
    Tp.block<3,3>(0, 0) = Rp;
    Tp.block<3,1>(0, 3) = tp;
//    transformPointCloud(*cloud, *cloud, Tp);

    Matrix3f Rt = euler2matrix(0, -tilt, 0);
    Matrix4f Tt = Matrix4f::Identity();
    Tt.block<3,3>(0, 0) = Rt;
//    transformPointCloud(*cloud, *cloud, Tt);

    // Transformada final - nuvem
    Matrix4f T = Matrix4f::Identity();
    T = Tp*Tt;
//    transformPointCloud(*cloud, *cloud, T.inverse());

    // Calculo do centro da camera - desenho do solid
    float bol = 0.056, blc = 0.0438, bac = 0.01;
    float x, y, z;
    // Primeiro braco
    z =  bol*cos(pan);
    x = -bol*sin(pan);
    T(0, 3) = x; T(2, 3) = z;
    // Segundo braco
    y = -blc*cos(tilt);
    z += blc*sin(tilt)*  cos(pan) ;
    x += blc*sin(tilt)*(-sin(pan));
    // Terceiro braco
    y += bac*sin(tilt);
    z += bac*cos(tilt)*  cos(pan) ;
    x += bac*cos(tilt)*(-sin(pan));
    // Inserindo offset do nivel horizontal
    y += blc;
    C << -x, y, z;

    transformPointCloud(*cloud, *cloud, T);

    // Quaternion da camera
    Matrix3f R = T.block<3,3>(0, 0);
    Quaternion<float> q_temp(R.inverse());
    q = q_temp;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
MatrixXf ProcessCloud::getRtcam(){
    return Rt1;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
Vector3f ProcessCloud::gettCam(){
    return Rt1.block<3,1>(0, 3);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::blueprint(PointCloud<PointTN>::Ptr cloud_in, float sa, float sr, Mat &bp){
    /// Separando a nuvem em clusters perpendiculares ao eixo y - y negativo para cima
    ///
    PointCloud<PointTN>::Ptr cloud (new PointCloud<PointTN>);
    *cloud = *cloud_in;
    // Filtrando a altura que vai entrar na roda
    float metros_altura_acima_pepo = 2; // quantos metros acima do PEPO para fazer a nuvem
    PassThrough<PointTN> pass;
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-metros_altura_acima_pepo, 100); // Negativo de tudo no eixo Y
    pass.setNegative(false);
    pass.setInputCloud(cloud);
    pass.filter(*cloud);
    // Filtrando pontos fora da area de interesse
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-sa/2, sa/2); // Quadrado em Z
    pass.setInputCloud(cloud);
    pass.filter(*cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-sa/2, sa/2); // Quadrado em Z
    pass.setInputCloud(cloud);
    pass.filter(*cloud);
    int w = sa/sr, h = sa/sr;
    vector< vector<PointTN> > supervoxels(w);
    for(int i=0; i<supervoxels.size(); i++) supervoxels[i].resize(h);
    // Preenchendo o os supervoxels com um ponto em altura inicial para a comparacao
    PointTN pini;
    pini.y = 100;
  #pragma omp parallel for
    for(int u=0; u<supervoxels.size(); u++){
        for(int v=0; v<supervoxels[u].size(); v++)
            supervoxels[u][v] = pini;
    }
    // Destinando cada ponto da nuvem original para o seu local no vetor de vetores segundo dimensoes, se for mais alto que o anterior
    for(size_t i=0; i<cloud->size(); i++){
        int u = abs(cloud->points[i].x - (-sa/2))/sa * w, v = abs(cloud->points[i].z - (-sa/2))/sa * h;
        if(u >= w) u = w - 1;
        if(v >= h) v = h - 1;
        if(cloud->points[i].y < supervoxels[u][v].y)
            supervoxels[u][v] = cloud->points[i];
    }
    cloud->clear();

    /// Colorindo a imagem final de acordo com a resolucao da separacao da nuvem
    ///
    // Iniciar a imagem com a quantidade de pixels de acordo com numero de supervoxels
    bp = Mat::zeros(cv::Size(w, h), CV_8UC3);
    // Processando em paralelo, procurar ponto mais alto de cada supervoxel
  #pragma omp parallel for
    for(int u=0; u<supervoxels.size(); u++){
        for(int v=0; v<supervoxels[u].size(); v++){
            PointTN p = supervoxels[u][v];
            Vec3b cor;
            // Atribuir cor do ponto mais alto aquele lugar da foto
            cor.val[0] = p.b; cor.val[1] = p.g; cor.val[2] = p.r;
            bp.at<Vec3b>(h-1-v, u) = cor;
        }
    }

    /// Salvando
    ///
    imwrite(pasta+"planta_baixa.png", bp);
}
