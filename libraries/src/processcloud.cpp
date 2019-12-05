#include "../include/processcloud.h"

/////////////////////////////////////////////////////////////////////////////////////////////////
ProcessCloud::ProcessCloud()
{
    // Dimensoes da camera USB de entrada
    cam_w = 1024; cam_h = 768;
    // Inicia matriz intrinseca da camera USB

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
    ne.setKSearch(20);
    ne.setNumberOfThreads(8);
    // Nuvem de normais calculada
    PointCloud<Normal>::Ptr cloud_normals (new PointCloud<Normal>());
    ne.compute(*cloud_normals);
    // Adiciona saida na nuvem concatenada PointTN
    concatenateFields(*in, *cloud_normals, *acc_normal);
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
    float mindist = 1000, maxdist = 0, scale, dev = 300;
    float r, g, b;
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
    #pragma omp parallel for num_threads(10)
    for(size_t i=0; i < nuvem->size(); i++){
        scale = 750 * (dists[i] - mindist)/(maxdist - mindist);
        // Pegar a cor como funcao normal
        r = alpha*normaldist(scale, 0, dev); g = alpha*normaldist(scale, 390, dev); b = alpha*normaldist(scale, 750, dev);

        nuvem->points[i].r = r;
        nuvem->points[i].g = g;
        nuvem->points[i].b = b;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::createVirtualLaserImage(PointCloud<PointTN>::Ptr nuvem){
    // Projetar os pontos na foto virtual e colorir imagem
    cv::Mat fl(cv::Size(cam_w, cam_h), CV_8UC3, cv::Scalar(0, 0, 0)); // Mesmas dimensoes que a camera tiver
    #pragma omp parallel for num_threads(10)
    for(size_t i = 0; i < nuvem->size(); i++){
        /// Pegar ponto em coordenadas normais
        Eigen::MatrixXf X_(3, 1);
        X_ << nuvem->points[i].x,
              nuvem->points[i].y,
              nuvem->points[i].z;
        Eigen::MatrixXf X = K_cam*X_;
        X = X/X(2, 0);
        /// Adicionando ponto na imagem se for o caso de projetado corretamente
        if(floor(X(0,0)) >= 0 && floor(X(0,0)) < fl.cols && floor(X(1,0)) >= 0 && floor(X(1,0)) < fl.rows){
            cv::Vec3b cor;
            cor[0] = nuvem->points[i].b; cor[1] = nuvem->points[i].g; cor[2] = nuvem->points[i].r;
            fl.at<cv::Vec3b>(cv::Point(X(0,0), X(1,0))) = cor;
        }
    }
    // Coloca na foto que e variavel global
    fl.copyTo(foto_laser);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::saveCloud(PointCloud<PointTN>::Ptr nuvem){
    std::string nome_nuvem = pasta+"nuvem_final.ply";
    savePLYFileASCII(pasta+nome_nuvem, *nuvem);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void ProcessCloud::saveImage(cv::Mat img){
    std::string nome = pasta+"camera_virtual.png";
    std::vector<int> opcoes;
    opcoes.push_back(cv::IMWRITE_PNG_COMPRESSION);
    cv::imwrite(nome, foto_laser, opcoes);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
float ProcessCloud::normaldist(float x, float media, float dev){
    return exp( -0.5*((x - media)/dev)*((x - media)/dev) ) / sqrt( 2*M_PI*dev*dev );
}
