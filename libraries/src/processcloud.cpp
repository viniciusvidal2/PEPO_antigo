#include "../include/processcloud.h"

/////////////////////////////////////////////////////////////////////////////////////////////////
ProcessCloud::ProcessCloud()
{
    // Dimensoes da camera USB de entrada
    cam_w = 1024; cam_h = 768;
    // Inicia matriz intrinseca da camera USB
    K_cam << 1484.701399,    0.000000, float(cam_w)/2,//432.741036,
                0.000000, 1477.059238, float(cam_h)/2,//412.362072,
                0.000000,    0.000000,   1.000000;
    // Inicia nome da pasta -> criar pasta no Dados_B9 no DESKTOP!
    char* home;
    home = getenv("HOME");
    pasta = std::string(home)+"/Desktop/Dados_B9/";
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
    // Inicia estimador de normais
    search::KdTree<PointT>::Ptr tree (new search::KdTree<PointT>());
    NormalEstimationOMP<PointT, Normal> ne;
    ne.setInputCloud(in);
    ne.setSearchMethod(tree);
    ne.setKSearch(10);
    ne.setNumberOfThreads(20);
    // Nuvem de normais calculada
    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>());
    ne.compute(*cloud_normals);
    // Adiciona saida na nuvem concatenada PointTN
    concatenateFields(*in, *cloud_normals, *acc_normal);
    // Filtra por normais problematicas
    std::vector<int> indicesnan;
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
    // Inicia estimador de normais
    search::KdTree<PointTN>::Ptr tree (new search::KdTree<PointTN>());
    NormalEstimationOMP<PointTN, PointTN> ne;
    ne.setInputCloud(acc_normal);
    ne.setSearchMethod(tree);
    ne.setKSearch(10);
    ne.setNumberOfThreads(20);
    ne.compute(*acc_normal);
    // Filtra por normais problematicas
    std::vector<int> indicesnan;
    removeNaNNormalsFromPointCloud(*acc_normal, *acc_normal, indicesnan);

    // Forcar virar as normais na marra para a origem
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
void ProcessCloud::colorCloudThroughDistance(PointCloud<PointTN>::Ptr nuvem){
    // Varre nuvem atras da maior e menor distancia
    float mindist = 1000, maxdist = 0, dev = 400;
    float alpha = 250.0 / normaldist(0, 0, dev);
    std::vector<float> dists(nuvem->size());
    for(size_t i=0; i < nuvem->size(); i++){
        dists[i] = sqrt( pow(nuvem->points[i].x, 2) + pow(nuvem->points[i].y, 2) + pow(nuvem->points[i].z, 2) );
        if(dists[i] > maxdist)
            maxdist = dists[i];
        if(dists[i] < mindist)
            mindist = dists[i];
    }
    // Calcula distancias de cada ponto, regra sobre a distancia e atribui cor
    #pragma omp parallel for
    for(size_t i=0; i < nuvem->size(); i++){
        float scale = 750 * (dists[i] - mindist)/(maxdist - mindist);
        // Pegar a cor como funcao normal
        float r = alpha*normaldist(scale, 0, dev), g = alpha*normaldist(scale, 390, dev), b = alpha*normaldist(scale, 750, dev);

        nuvem->points[i].r = r;
        nuvem->points[i].g = g;
        nuvem->points[i].b = b;
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
void ProcessCloud::createVirtualLaserImage(PointCloud<PointTN>::Ptr nuvem, string nome){
    // Projetar os pontos na foto virtual e colorir imagem
    Mat fl(Size(cam_w, cam_h), CV_8UC3, Scalar(0, 0, 0)); // Mesmas dimensoes que a camera tiver
    #pragma omp parallel for
    for(size_t i = 0; i < nuvem->size(); i++){
        /// Pegar ponto em coordenadas normais
        Eigen::MatrixXf X_(3, 1);
        X_ << nuvem->points[i].x,
              nuvem->points[i].y,
              nuvem->points[i].z;
        Eigen::MatrixXf X = K_cam*X_;
        X = X/X(2, 0);
        /// Adicionando ponto na imagem se for o caso de projetado corretamente (para otimizacao de foco so funciona a menos de 65 metros)
        if(floor(X(0,0)) >= 0 && floor(X(0,0)) < fl.cols && floor(X(1,0)) >= 0 && floor(X(1,0)) < fl.rows){
            cv::Vec3b cor;
            cor.val[0] = nuvem->points[i].b; cor.val[1] = nuvem->points[i].g; cor.val[2] = nuvem->points[i].r;
            fl.at<Vec3b>(Point(int(X(0,0)), int(X(1,0)))) = cor;
        }
    }
    // Corrigir os ruidos cinzas antes de salvar
    fl = correctColorCluster(fl);
    // Salva de uma vez a foto do laser
    saveImage(fl, nome);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
Mat ProcessCloud::projectCloudToLaserCenter(PointCloud<PointTN>::Ptr cloud, float fx, float fy, float tx, float ty, Size s){
    Mat image = Mat::zeros(s, CV_8UC3);
    // Matriz intrinseca e extrinseca
    Eigen::Matrix3f K;
    K << fx,  0, s.width /2.0,
          0, fy, s.height/2.0,
          0,  0,      1      ;
    Eigen::MatrixXf Rt(3, 4);
    Rt << 1, 0, 0, tx/100.0,
          0, 1, 0, ty/100.0,
          0, 0, 1,    0    ;
    Eigen::MatrixXf P(3, 4);
    P = K*Rt;
    #pragma omp parallel for
    for(size_t i = 0; i < cloud->size(); i++){
        // Pegar ponto em coordenadas homogeneas
        Eigen::MatrixXf X_(4, 1);
        X_ << cloud->points[i].x,
              cloud->points[i].y,
              cloud->points[i].z,
                      1         ;
        Eigen::MatrixXf X(3, 1);
        X = P*X_;
        if(X(2, 0) > 0){
            X = X/X(2, 0);
            // Adicionando ponto na imagem se for o caso de projetado corretamente
            if(floor(X(0,0)) > 0 && floor(X(0,0)) < image.cols && floor(X(1,0)) > 0 && floor(X(1,0)) < image.rows){
                cv::Vec3b cor;
                cor.val[0] = cloud->points[i].b; cor.val[1] = cloud->points[i].g; cor.val[2] = cloud->points[i].r;
                image.at<Vec3b>(Point(int(X(0,0)), int(X(1,0)))) = cor;
            }
        }
    }

    return image;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::colorCloudWithCalibratedImage(PointCloud<PointTN>::Ptr cloud_in, PointCloud<PointTN>::Ptr cloud_out, Mat image, float fx, float fy, float tx, float ty){
    // Matriz intrinseca e extrinseca
    Eigen::Matrix3f K;
    K << fx,  0, image.cols/2.0,
          0, fy, image.rows/2.0,
          0,  0,      1      ;
    Eigen::MatrixXf Rt(3, 4);
    Rt << 1, 0, 0, tx/100.0,
          0, 1, 0, ty/100.0,
          0, 0, 1,    0    ;
    Eigen::MatrixXf P(3, 4);
    P = K*Rt;
    for(size_t i = 0; i < cloud_in->size(); i++){
        // Pegar ponto em coordenadas homogeneas
        Eigen::MatrixXf X_(4, 1);
        X_ << cloud_in->points[i].x,
              cloud_in->points[i].y,
              cloud_in->points[i].z,
                        1          ;
        Eigen::MatrixXf X(3, 1);
        X = P*X_;
        if(X(2, 0) > 0){
            X = X/X(2, 0);
            // Adicionando ponto na imagem se for o caso de projetado corretamente
            if(floor(X(0,0)) > 0 && floor(X(0,0)) < image.cols && floor(X(1,0)) > 0 && floor(X(1,0)) < image.rows){
                cv::Vec3b cor = image.at<Vec3b>(Point(X(0,0), X(1,0)));
                PointTN point = cloud_in->points[i];
                point.b = cor.val[0]; point.g = cor.val[1]; point.r = cor.val[2];
                cloud_out->push_back(point);
            }
        }
    }

}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::filterCloudDepthCovariance(PointCloud<PointTN>::Ptr cloud, int kn, float thresh){
    // Objetos para filtrar
    ExtractIndices<PointTN> extract;
    PointIndices::Ptr outliers (new PointIndices);
    PointCloud<PointTN>::Ptr marcar_outliers (new PointCloud<PointTN>);
    marcar_outliers->resize(cloud->size());
    // Objetos para procurar na nuvem o centroide e a covariancia
    KdTreeFLANN<PointTN>::Ptr tree (new KdTreeFLANN<PointTN>);
    tree->setInputCloud(cloud);
    // Varrer todos os pontos atras da covariancia
    #pragma omp parallel for
    for(size_t i=0; i<cloud->size(); i++){
        // Cria variaveis aqui dentro pelo processo ser paralelizado
        Eigen::Vector4f centroide;
        Eigen::Matrix4f rotacao_radial;
        Eigen::Matrix3f covariancia;
        rotacao_radial = Eigen::Matrix4f::Identity();
        // Calcula angulo para rotacionar a nuvem e cria matriz de rotacao (yaw em torno de Y, pitch em torno de X)
        float yaw_y   = atan2(cloud->points[i].x, cloud->points[i].z);
        float pitch_x = atan2(cloud->points[i].x, cloud->points[i].z);
        Eigen::Matrix3f rot = this->euler2matrix(0, pitch_x, yaw_y);
        rotacao_radial.block<3,3>(0, 0) << rot.inverse();
        // Calcula vizinhos mais proximos aqui por raio ou K neighbors
        vector<int> indices_vizinhos;
        vector<float> distancias_vizinhos;
        tree->nearestKSearch(int(i), kn, indices_vizinhos, distancias_vizinhos);
        // Separa nuvem com esses vizinhos
        PointCloud<PointTN>::Ptr temp (new PointCloud<PointTN>);
        temp->resize(indices_vizinhos.size());
        for(size_t j=0; j<indices_vizinhos.size(); j++){
            temp->points[j] = cloud->points[ indices_vizinhos[j] ];
        }
        // Rotaciona a nuvem separada segundo o raio que sai do centro do laser (origem)
        transformPointCloudWithNormals(*temp, *temp, rotacao_radial);
        // Calcula centroide e covariancia da nuvem
        compute3DCentroid(*temp, centroide);
        computeCovarianceMatrix(*temp, centroide, covariancia);
        // Se for muito maior em z que em x e y, considera ruim e marca na nuvem
        if(covariancia(2, 2) > thresh*covariancia(0, 0) && covariancia(2, 2) > thresh*covariancia(1, 1))
            marcar_outliers->points[i].x = 1;
    }
    // Passa rapidamente para nuvem de indices
    for(size_t i=0; i<marcar_outliers->size(); i++){
        if(marcar_outliers->points[i].x == 1)
            outliers->indices.push_back(i);
    }
    // Extrair pontos da nuvem
    ROS_INFO("Serao extraidos %zu pontos da nuvem, %.2f por cento.", outliers->indices.size(), 100*float(outliers->indices.size())/float(cloud->size()));
    extract.setInputCloud(cloud);
    extract.setNegative(true);
    extract.setIndices(outliers);
    extract.filter(*cloud);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::saveCloud(PointCloud<PointTN>::Ptr nuvem){
    std::string nome_nuvem = pasta+"nuvem_final.ply";
    savePLYFileASCII<PointTN>(nome_nuvem, *nuvem);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::saveImage(cv::Mat img, string nome){
    std::string final = pasta+nome+".png";
    vector<int> params;
    params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    params.push_back(9);
    cv::imwrite(final, img, params);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
float ProcessCloud::normaldist(float x, float media, float dev){
    return exp( -0.5*((x - media)/dev)*((x - media)/dev) ) / sqrt( 2*M_PI*dev*dev );
}
/////////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Matrix3f ProcessCloud::euler2matrix(float r, float p, float y){
    Eigen::AngleAxisf roll( r, Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf pitch(p, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf yaw(  y, Eigen::Vector3f::UnitY());
    Eigen::Quaternion<float> q = roll*yaw*pitch;

    return q.matrix();
}
/////////////////////////////////////////////////////////////////////////////////////////////////
Mat ProcessCloud::correctColorCluster(Mat in){
    // Limite da vizinhanca (pixels) a ser varrido em busca de uma moda de cor
    int lim = 5;
    // Varrer todos os pixels e achar os que sao cinza (excluindo as bordas segundo limite de vizinhanca)
    #pragma omp parallel for
    for(int u=0+lim; u<in.cols-lim; u++){
        for(int v=0+lim; v<in.rows-lim; v++){
            // Se for preto, varrer os vizinhos
            Vec3b cor = in.at<Vec3b>(Point(u, v));
            if(cor.val[0] == 0 && cor.val[1] == 0 && cor.val[2] == 0){ // A imagem criada tem esse valor para pixels nao projetados
                // Encontrar a moda dos vizinhos: quem tiver mais de tal cor ganha e essa cor e atribuida ao pixel central
                Vec3b cor_moda = findPredominantColor(u, v, in, lim);
                // Atribuir cor encontrada para o pixel em questao
                in.at<Vec3b>(Point(u, v)) = cor_moda;
            }
        }
    }

    // Retornar imagem corrigida
    return in;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
Vec3b ProcessCloud::findPredominantColor(int u, int v, Mat in, int desvio){
    // Cores encontradas
    vector<Vec3b> cores;
    // Contadores para cada cor encontrada
    vector<int> contadores;
    // Flag para cor encontrada e indice encontrado
    bool encontrada = false;
    size_t indice_encontrado = 1000;
    // Varrendo vizinhanca de n x n ao redor do pixel
    for(int i=u-desvio; i<u+desvio; i++){
        for(int j=v-desvio; j<v+desvio; j++){
            // Checando por limites
            if(i > 0 && i < in.cols && j > 0 && j < in.rows){

                Vec3b cor_temp = in.at<Vec3b>(Point(i, j));

                // Se os vetores estao vazios, iniciar eles
                if(cores.empty() && contadores.empty()){
                    cores.push_back(cor_temp);
                    contadores.push_back(1);
                    continue;
                }
                // Varrendo o vetor de cores pra ver se aquela ja foi vista
                for(size_t k=0; k<cores.size(); k++){
                    // Se for encontrada, marcar a flag e o indice
                    if(cor_temp.val[0] == cores[k].val[0] && cor_temp.val[1] == cores[k].val[1] && cor_temp.val[2] == cores[k].val[2]){
                        encontrada = true;
                        indice_encontrado = k;
                    }
                }
                // Se a cor ja foi adicionada, somar ao contador naquele indice
                if(encontrada){
                    contadores[indice_encontrado] = contadores[indice_encontrado] + 1;
                } else { // Se nao foi encontrada, adicionar mais um caso ao vetor
                    cores.push_back(cor_temp);
                    contadores.push_back(1);
                }
                // Resetando as variaveis de busca
                encontrada = false;
                indice_encontrado = 1000;
            }
        }
    }
    // Definindo o indice com o maximo de cores encontradas
    indice_encontrado = max_element(contadores.begin(), contadores.end()) - contadores.begin();

    return cores[indice_encontrado];
}
/////////////////////////////////////////////////////////////////////////////////////////////////
