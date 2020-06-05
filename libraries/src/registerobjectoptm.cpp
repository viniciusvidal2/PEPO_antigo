#include "../include/registerobjectoptm.h"

/////////////////////////////////////////////////////////////////////////////////////////////////
RegisterObjectOptm::RegisterObjectOptm()
{

}
/////////////////////////////////////////////////////////////////////////////////////////////////
RegisterObjectOptm::~RegisterObjectOptm(){
    ros::shutdown();
    ros::waitForShutdown();
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void RegisterObjectOptm::readCloudAndPreProcess(string name, PointCloud<PointTN>::Ptr cloud){
    // Le a nuvem
    loadPLYFile<PointTN>(name, *cloud);
    // Filtro de voxels para aliviar a entrada
    VoxelGrid<PointTN> voxel;
    float lfsz = 0.03;
    voxel.setLeafSize(lfsz, lfsz, lfsz);
    // Filtro de profundidade para nao pegarmos muito fundo
    PassThrough<PointTN> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 5); // Z metros de profundidade
    // Filtro de ruidos aleatorios
    StatisticalOutlierRemoval<PointTN> sor;
    sor.setMeanK(10);
    sor.setStddevMulThresh(2);
    sor.setNegative(false);
    // Passando filtros
    voxel.setInputCloud(cloud);
    voxel.filter(*cloud);
    pass.setInputCloud(cloud);
    pass.filter(*cloud);
    sor.setInputCloud(cloud);
    sor.filter(*cloud);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void RegisterObjectOptm::projectCloudAndAnotatePixels(PointCloud<PointTN>::Ptr cloud, Mat im, PointCloud<PointTN>::Ptr cloud_pix, float f, Vector3f t){
    // Matriz intrinseca e extrinseca
    Matrix3f K;
    K << f, 0, 973,//image.cols/2.0,
            0, f, 536,//image.rows/2.0,
            0, 0,  1 ;
    MatrixXf Rt(3, 4); // Desenho do antonio - diferenca no frame da camera do laser para a camera
    Rt << 1, 0, 0, t(0),
            0, 1, 0, t(1),
            0, 0, 1, t(2);
    MatrixXf P(3, 4);
    P = K*Rt;
    // Preparando nuvem de pixels
    cloud_pix->resize(cloud->size());
    for(size_t i = 0; i < cloud->size(); i++){
        // Pegar ponto em coordenadas homogeneas
        MatrixXf X_(4, 1);
        X_ << cloud->points[i].x,
                cloud->points[i].y,
                cloud->points[i].z,
                1       ;
        MatrixXf X(3, 1);
        X = P*X_;
        if(X(2, 0) > 0){
            X = X/X(2, 0);
            // Colorindo ponto se for o caso de projetado corretamente
            if(floor(X(0,0)) > 0 && floor(X(0,0)) < im.cols && floor(X(1,0)) > 0 && floor(X(1,0)) < im.rows){
                cv::Vec3b cor = im.at<Vec3b>(Point(X(0,0), X(1,0)));
                PointTN point = cloud->points[i];
                point.b = cor.val[0]; point.g = cor.val[1]; point.r = cor.val[2];
                cloud->points[i] = point;
                // Anotando que aquele ponto 3D cai naquele pixel da imagem
                cloud_pix->points[i].x = X(0,0); cloud_pix->points[i].y = X(1,0);
            }
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
Matrix4f RegisterObjectOptm::icp(PointCloud<PointTN>::Ptr ctgt, PointCloud<PointTN>::Ptr csrc, int its){
    Matrix4f Ticp;

    // Criando o otimizador de ICP comum
    IterativeClosestPoint<PointTN, PointTN> icp;
    icp.setUseReciprocalCorrespondences(true);
    icp.setInputTarget(ctgt);
    icp.setInputSource(csrc);
    icp.setMaximumIterations(its); // Chute inicial bom 10-100
    icp.setTransformationEpsilon(1*1e-10);
    icp.setEuclideanFitnessEpsilon(1*1e-12);
    icp.setMaxCorrespondenceDistance(0.3);
    // Alinhando
    PointCloud<PointTN> dummy;
    icp.align(dummy, Matrix4f::Identity());
    // Obtendo a transformacao otimizada e aplicando
    if(icp.hasConverged())
        Ticp = icp.getFinalTransformation();
    else
        Ticp = Matrix4f::Identity();
    transformPointCloudWithNormals<PointTN>(*csrc, *csrc, Ticp);

    return Ticp;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void RegisterObjectOptm::matchFeaturesAndFind3DPoints(Mat imref, Mat imnow, PointCloud<PointTN>::Ptr cref, PointCloud<PointTN>::Ptr cnow, int npontos3d, vector<Point2d> &matches3d){
    /// Calculando descritores SIFT ///
    // Keypoints e descritores para astra e zed
    vector<KeyPoint> kpref, kpnow;
    Mat dref, dnow;
    /// Comparando e filtrando matchs ///
    cv::Ptr<DescriptorMatcher> matcher;
    matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    vector<vector< DMatch > > matches;
    vector< DMatch > good_matches;

    int tent = 0; // tentativas maximas de achar npontos3d correspondencias bacanas
    float min_hessian = 2000;

    /// Loop de busca por matches e pontos 3D correspondentes ///
    while(tent < 30 && matches3d.size() <= npontos3d){
        // Calculando features e realizando match
        good_matches.clear();
        Ptr<xfeatures2d::SURF> f2d = xfeatures2d::SURF::create(min_hessian);
        f2d->detectAndCompute(imref, Mat(), kpref, dref);
        f2d->detectAndCompute(imnow, Mat(), kpnow, dnow);
        matcher->knnMatch(dref, dnow, matches, 2);

        for (size_t i = 0; i < matches.size(); i++){
            if (matches.at(i).size() >= 2){
                if (matches.at(i).at(0).distance < 0.8*matches.at(i).at(1).distance)
                    good_matches.push_back(matches.at(i).at(0));
            }
        }

        tent += 1;
        min_hessian = 0.7*min_hessian;

        // Achando realmente os keypoints bons e anotando coordenadas
        vector<Point2f> imrefpts, imnowpts;
        imrefpts.resize( good_matches.size()); imnowpts.resize( good_matches.size());
#pragma omp parallel for
        for (size_t i = 0; i < good_matches.size(); i++){
            imrefpts[i]  = kpref[good_matches[i].queryIdx].pt;
            imnowpts[i]  = kpnow[good_matches[i].trainIdx].pt;
        }

        // Se ha matches suficientes, buscar nas nuvens 3D auxiliar se aquele ponto e correspondente
        // ao pixel daquela match
        if(good_matches.size() > npontos3d){
            // Para cada match
            for(size_t j=0; j<good_matches.size(); j++){
                float dist_thresh = 10, dist_good = 5, best_dist = 1000, dx2, dy2;
                size_t curr_pt_ref = -1, curr_pt_now = -1; // Indice do melhor ponto no momento
                // Procurando na nuvem REF
                for(size_t i=0; i<cref->size(); i++){
                    dx2 = pow(cref->points[i].x - imrefpts[j].x, 2), dy2 = pow(cref->points[i].y - imrefpts[j].y, 2);
                    if(sqrt(dx2+dy2) < best_dist && sqrt(dx2+dy2) < dist_thresh){
                        best_dist = sqrt(dx2+dy2) < best_dist;
                        curr_pt_ref = i;
                        if(best_dist < dist_good) // Aqui estamos suficientemente perto, esta encontrado
                            break;
                    }
                }
                // Procurando na nuvem NOW
                best_dist = 1000;
                for(size_t i=0; i<cnow->size(); i++){
                    dx2 = pow(cnow->points[i].x - imnowpts[j].x, 2), dy2 = pow(cnow->points[i].y - imnowpts[j].y, 2);
                    if(sqrt(dx2+dy2) < best_dist && sqrt(dx2+dy2) < dist_thresh){
                        best_dist = sqrt(dx2+dy2) < best_dist;
                        curr_pt_now = i;
                        if(best_dist < dist_good) // Aqui estamos suficientemente perto, esta encontrado
                            break;
                    }
                }
                // Se foi encontrado um indice para cada nuvem, anotar isso no vetor de indices de match nas nuvens
                if(curr_pt_ref != -1 && curr_pt_now != -1){
                    Point2d m3d;
                    m3d.x = curr_pt_ref; m3d.y = curr_pt_now;
                    matches3d.push_back(m3d); // Primeiro referencia ref, depois atual now
                }
                // Se achou ja npontos3d de matches boas, podemos parar aqui
                if(matches3d.size() >= npontos3d)
                    break;
            } // Fim do for de matches
        }
    } // Fim do while

    // Se nao deu certo, avisar com enfase
    if(matches3d.size() < npontos3d)
        ROS_ERROR("NAO ACHAMOS %d CORRESPONDENCIAS, MAS SIM %d", npontos3d, matches3d.size());
}
/////////////////////////////////////////////////////////////////////////////////////////////////
Matrix4f RegisterObjectOptm::optmizeTransformLeastSquares(PointCloud<PointTN>::Ptr cref, PointCloud<PointTN>::Ptr cnow, vector<Point2d> matches3d){
    /// Conta dos minimos quadrados
    /// Theta = inv(Xt*X)*Xt*y
    /// Theta sao os coeficientes finais
    /// X sao em linhas as coordenadas dos pontos na nuvem now
    /// y sao em linhas cada hora uma das coordenadas nos pontos na nuvem ref, a depender do sistema que estamos tratando (eixos, X, Y ou Z)

    // Iniciando vetores
    MatrixXf X(matches3d.size(), 4), Xt, y(matches3d.size(), 1), Theta; // minimos quadrados
    Matrix4f Tf = Matrix4f::Identity(); // Matriz que sera otimizada
    int indice_ref, indice_now; // Indices dos pontos 3D correspondentes nas nuvens
    // Para cada sistema (eixo 3D)
    for(int s=0; s<3; s++){
        // Preenchendo vetores least squares //
        for(int i=0; i<matches3d.size(); i++){
            indice_ref = matches3d[i].x;
            indice_now = matches3d[i].y;
            X.block<1,4>(i, 0) << cnow->points[indice_now].x, cnow->points[indice_now].y, cnow->points[indice_now].z, 1;
            if(s == 0)
                y(i, 0) = cref->points[indice_ref].x;
            else if(s == 1)
                y(i, 0) = cref->points[indice_ref].y;
            else if(s == 2)
                y(i, 0) = cref->points[indice_ref].z;
        }
        // Obtendo a transposta para fazer isso so uma vez
        Xt = X.transpose();
        // Operacao de minimos quadrados para obter resultado para o sistema atual
        Theta = (Xt*X).inverse()*Xt*y;
        // Inserindo resultado na matriz de transformacao final
        Tf.block<1,4>(s, 0) << Theta.transpose();
    }

    cout << "\nMatriz homogenea: \n" << Tf << endl;

    // Retorna a transformacao otimizada
    return Tf;
}
