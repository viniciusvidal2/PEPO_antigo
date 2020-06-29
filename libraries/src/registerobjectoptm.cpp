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
    PointCloud<PointT>::Ptr cin (new PointCloud<PointT>);
    loadPLYFile<PointT>(name, *cin);
    // Filtro de voxels para aliviar a entrada
    VoxelGrid<PointT> voxel;
    float lfsz = 0.04;
    voxel.setLeafSize(lfsz, lfsz, lfsz);
    // Filtro de profundidade para nao pegarmos muito fundo
    PassThrough<PointT> pass;
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 7); // Z metros de profundidade
    // Filtro de ruidos aleatorios
    StatisticalOutlierRemoval<PointT> sor;
    sor.setMeanK(30);
    sor.setStddevMulThresh(2.5);
    sor.setNegative(false);
    // Passando filtros
    sor.setInputCloud(cin);
    sor.filter(*cin);
    voxel.setInputCloud(cin);
    voxel.filter(*cin);
    pass.setInputCloud(cin);
    pass.filter(*cin);
    sor.setInputCloud(cin);
    sor.filter(*cin);
    // Passando polinomio pra suavizar a parada
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>());
    MovingLeastSquares<PointT, PointTN> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(cin);
    mls.setPolynomialOrder(1);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.05);
    mls.process(*cloud);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void RegisterObjectOptm::projectCloudAndAnotatePixels(PointCloud<PointTN>::Ptr cloud, Mat im, PointCloud<PointTN>::Ptr cloud_pix, float f, Vector3f t, MatrixXi &impix){
    // Matriz intrinseca e extrinseca
    Matrix3f K;
    K << f, 0, im.cols/2.0,
         0, f, im.rows/2.0,
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
                // Anotando que aquele ponto 3D cai naquele pixel da imagem, e usando a normal pra guardar a coordenada 3D em si - debug
                cloud_pix->points[i].x = X(0,0); cloud_pix->points[i].y = X(1,0);
                cloud_pix->points[i].r = cor.val[2]; cloud_pix->points[i].g = cor.val[1]; cloud_pix->points[i].b = cor.val[0];
                cloud_pix->points[i].normal_x = cloud->points[i].x;
                cloud_pix->points[i].normal_y = cloud->points[i].y;
                cloud_pix->points[i].normal_z = cloud->points[i].z;
                impix(X(1,0), X(0,0)) = i;
            } else {
                cloud_pix->points[i].x = 1000000; cloud_pix->points[i].y = 1000000;
            }
        } else {
            cloud_pix->points[i].x = 1000000; cloud_pix->points[i].y = 1000000;
        }
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
Matrix4f RegisterObjectOptm::icp(PointCloud<PointTN>::Ptr ctgt, PointCloud<PointTN>::Ptr csrc, int its){
    Matrix4f Ticp;

    // Reduzindo ainda mais as nuvens pra nao dar trabalho assim ao icp
    PointCloud<PointTN>::Ptr tgttemp(new PointCloud<PointTN>);
    PointCloud<PointTN>::Ptr srctemp(new PointCloud<PointTN>);
//    VoxelGrid<PointTN> voxel;
    float lfsz = 0.05;
//    voxel.setLeafSize(lfsz, lfsz, lfsz);
//    voxel.setInputCloud(ctgt);
//    voxel.filter(*tgttemp);
//    voxel.setInputCloud(csrc);
//    voxel.filter(*srctemp);
    *tgttemp = *ctgt;
    *srctemp = *csrc;

    // Criando o otimizador de ICP comum
    GeneralizedIterativeClosestPoint<PointTN, PointTN> icp;
//    IterativeClosestPoint<PointTN, PointTN> icp;
    icp.setUseReciprocalCorrespondences(false);
    icp.setInputTarget(tgttemp);
    icp.setInputSource(srctemp);
//    icp.setRANSACIterations(30);
    icp.setMaximumIterations(its); // Chute inicial bom 10-100
    icp.setTransformationEpsilon(1*1e-9);
    icp.setEuclideanFitnessEpsilon(1*1e-12);
    icp.setMaxCorrespondenceDistance(lfsz*3);
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
void RegisterObjectOptm::matchFeaturesAndFind3DPoints(Mat imref, Mat imnow, PointCloud<PointTN>::Ptr cref, PointCloud<PointTN>::Ptr cnow, int npontos3d, vector<Point2d> &matches3d,
                                                      MatrixXi refpix, MatrixXi nowpix){
    /// Calculando descritores SIFT ///
    // Keypoints e descritores para astra e zed
    vector<KeyPoint> kpref, kpnow;
    vector<Point2f> imrefpts, imnowpts, goodimrefpts, goodimnowpts;
    Mat dref, dnow;
    /// Comparando e filtrando matchs ///
    cv::Ptr<DescriptorMatcher> matcher;
    matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    vector<vector< DMatch > > matches;
    vector< DMatch > good_matches;

    int tent = 0; // tentativas maximas de achar npontos3d correspondencias bacanas
    float min_hessian = 1500; //sigma_thresh = 1.6/10, edge_thresh = 10*10, contrast_thresh = 0.04/10;

    /// Loop de busca por matches e pontos 3D correspondentes ///
    while(tent < 15 && matches3d.size() <= npontos3d){
        // Resetando vetores do algoritmo para buscarem novamente
        matches.clear(); good_matches.clear(); matches3d.clear();
        goodimnowpts.clear(); goodimrefpts.clear();
        imnowpts.clear(); imrefpts.clear();
        // Calculando features e realizando match
        Ptr<xfeatures2d::SURF> f2d = xfeatures2d::SURF::create(min_hessian);
        f2d->detectAndCompute(imref, Mat(), kpref, dref);
        f2d->detectAndCompute(imnow, Mat(), kpnow, dnow);
        matcher->knnMatch(dref, dnow, matches, 2);

        for (size_t i = 0; i < matches.size(); i++){
            if (matches.at(i).size() >= 2){
                if (matches.at(i).at(0).distance < 0.7*matches.at(i).at(1).distance){ // Se e bastante unica frente a segunda colocada
                    float dref, dnow; // Distancias dos pontos aos centros das imagens
                    Point2f pref, pnow;
                    pref = kpref[matches.at(i).at(0).queryIdx].pt; pnow = kpnow[matches.at(i).at(0).trainIdx].pt;
                    dref = sqrt( pow(pref.x - imref.cols/2, 2) + pow(pref.y - imref.rows, 2) );
                    dnow = sqrt( pow(pnow.x - imnow.cols/2, 2) + pow(pnow.y - imnow.rows, 2) );
                    if(dref < 0.8*imref.rows/2 && dnow < 0.8*imnow.rows/2) // Se esta localizada mais ao centro da imagem
                        good_matches.push_back(matches.at(i).at(0));
                }
            }
        }
        // Filtrando por distancia media entre os matches
        vector<float> distances (good_matches.size());
        for (int i=0; i < good_matches.size(); i++)
            distances[i] = good_matches[i].distance;
        float average = accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
        for(vector<DMatch>::iterator it = good_matches.begin(); it!=good_matches.end();){
            if(it->distance > average)
                good_matches.erase(it);
            else
                ++it;
        }
        // Filtrando por angulo das linhas entre os matches
        this->filterMatchesLineCoeff(good_matches, kpref, kpnow, imref.cols, 1);

        tent += 1;
        min_hessian *= 0.7;

        // Achando realmente os keypoints bons e anotando coordenadas
        imrefpts.resize(good_matches.size()); imnowpts.resize(good_matches.size());
        for (size_t i = 0; i < good_matches.size(); i++){
            imrefpts[i]  = kpref[good_matches[i].queryIdx].pt;
            imnowpts[i]  = kpnow[good_matches[i].trainIdx].pt;
        }

        // Se ha matches suficientes, buscar nas nuvens 3D auxiliar se aquele ponto e correspondente
        // ao pixel daquela match
        if(good_matches.size() > npontos3d){
//            vector< DMatch > good_temp(good_matches.begin(), good_matches.begin()+npontos3d-1);
//            vector< Point2f > tempnow(imnowpts.begin(), imnowpts.begin()+npontos3d-1), tempref(imrefpts.begin(), imrefpts.begin()+npontos3d-1);
//            good_matches = good_temp;
//            imnowpts = tempnow; imrefpts = tempref;
//            // Encontrar a homografia entre as imagens - de now para ref
//            Mat h = findHomography(imnowpts, imrefpts);
//            // Matriz de homografia em Eigen
//            Matrix3f H;
//            cv2eigen(h, H);
//            cout << H << endl;
//            // Projetando os pixels de now para ref
////            for(int i=0; i<nowpix.rows(); i++){
////                for(int j=0; j<nowpix.cols(); j++){
//            for(int i=0; i<imnowpts.size(); i++){
//                    // Se aqui existe um ponto da nuvem
////                    if(nowpix(i, j) != -1){
//                        Vector3f X_{imnowpts[i].x, imnowpts[i].y, 1};
//                        Vector3f X = H*X_;
//                        X = X/X(2);
//                        int u = X(1), v = X(0);
//                        cout << imrefpts[i].y << " " << imrefpts[i].x << " " << u << " " << v << endl;
//                        // Se caiu dentro da outra imagem e ali tambem existe um ponto, anotar
////                        if(u > 0 && u < nowpix.rows() && v > 0 && v < nowpix.cols()){
////                            if(refpix(u, v) != -1){
////                                Point2d m3d;
////                                m3d.x = refpix(u, v); m3d.y = nowpix(i, j);
////                                goodimrefpts.push_back(Point2f(v, u)); goodimnowpts.push_back(Point2f(j, i));
////                                matches3d.push_back(m3d); // Primeiro referencia ref, depois atual now
////                            }
////                        }
////                    }
////                }
//            }














            // Para cada match
            for(size_t j=0; j<good_matches.size(); j++){
            int curr_pt_ref = -1, curr_pt_now = -1; // Indice do melhor ponto no momento
            // Na imagem ref ver se naquela redondeza ha um pixel
            curr_pt_ref = this->searchNeighbors(refpix, imrefpts[j].y, imrefpts[j].x, 5);
            // Na imagem now ver se naquela redondeza ha um pixel
            curr_pt_now = this->searchNeighbors(nowpix, imnowpts[j].y, imnowpts[j].x, 5);

            // Se foi encontrado um indice para cada nuvem, anotar isso no vetor de indices de match nas nuvens
            if(curr_pt_ref != -1 && curr_pt_now != -1){
                Point2d m3d;
                m3d.x = curr_pt_ref; m3d.y = curr_pt_now;
                goodimrefpts.push_back(imrefpts[j]); goodimnowpts.push_back(imnowpts[j]);
                matches3d.push_back(m3d); // Primeiro referencia ref, depois atual now
            }
            // Se achou ja npontos3d de matches boas, podemos parar aqui
            if(matches3d.size() > npontos3d)
                    break;
            } // Fim do for de matches
        }
    } // Fim do while

    // Se nao deu certo, avisar com enfase
    if(matches3d.size() < npontos3d)
        ROS_ERROR("NAO ACHAMOS %d CORRESPONDENCIAS, MAS SIM %zu", npontos3d, matches3d.size());

    // Plotando os dados para debugar
    this->plotDebug(imref, imnow, cref, cnow, goodimrefpts, goodimnowpts, matches3d);
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

    // Retorna a transformacao otimizada
    return Tf;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void RegisterObjectOptm::plotDebug(Mat imref, Mat imnow, PointCloud<PointTN>::Ptr cref, PointCloud<PointTN>::Ptr cnow, vector<Point2f> pref, vector<Point2f> pnow, vector<Point2d> match){
    // Imagens temporarias para salvar
    Mat r, n;
    imref.copyTo(r); imnow.copyTo(n);
    // Nuvens temporarias para salvar
    PointCloud<PointTN>::Ptr cr (new PointCloud<PointTN>);
    PointCloud<PointTN>::Ptr cn (new PointCloud<PointTN>);
    PointCloud<PointTN>::Ptr crt (new PointCloud<PointTN>);
    PointCloud<PointTN>::Ptr cnt (new PointCloud<PointTN>);
    PointTN p;
    // Para cada ponto de match
    for(int i=0; i<match.size(); i++){
        // Criar uma cor aleatoria
        Scalar color = Scalar(rand() % 255, rand() % 255, rand() % 255);
        // Desenha circulo na imagem
        circle(r, pref[i], 9, color, 4);
        circle(n, pnow[i], 9, color, 4);
        // Colore ponto das nuvens com a cor tambem
        p.r = color[2]; p.g = color[1]; p.b = color[0];
        // Adiciona o ponto na nuvem temporaria
        p.x = cref->points[match[i].x].normal_x; p.y = cref->points[match[i].x].normal_y; p.z = cref->points[match[i].x].normal_z;
        cr->push_back(p);
        p.x = cnow->points[match[i].y].normal_x; p.y = cnow->points[match[i].y].normal_y; p.z = cnow->points[match[i].y].normal_z;
        cn->push_back(p);
    }
    // Toda a nuvem
    crt->resize(cref->size());
#pragma omp parallel for
    for(int i=0; i<cref->size(); i++){
        crt->points[i].x = cref->points[i].normal_x; crt->points[i].y = cref->points[i].normal_y; crt->points[i].z = cref->points[i].normal_z;
    }
    cnt->resize(cnow->size());
#pragma omp parallel for
    for(int i=0; i<cnow->size(); i++){
        cnt->points[i].x = cnow->points[i].normal_x; cnt->points[i].y = cnow->points[i].normal_y; cnt->points[i].z = cnow->points[i].normal_z;
    }
    // Salvando arquivos
    char* home;
    home = getenv("HOME");
    string pasta = string(home)+"/Desktop/";
    imwrite(pasta+"debugref.png", r);
    imwrite(pasta+"debugnow.png", n);
    savePLYFileBinary<PointTN>(pasta+"debugref.ply", *cr);
    savePLYFileBinary<PointTN>(pasta+"debugnow.ply", *cn);
    savePLYFileBinary<PointTN>(pasta+"debugreft.ply", *crt);
    savePLYFileBinary<PointTN>(pasta+"debugnowt.ply", *cnt);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
Matrix4f RegisterObjectOptm::optmizeTransformSVD(PointCloud<PointTN>::Ptr cref, PointCloud<PointTN>::Ptr cnow, vector<Point2d> matches3d){
    // Inicia estimador
    registration::TransformationEstimationSVD<PointTN, PointTN> svd;
    PointIndices pref, pnow;
    Matrix4f Tsvd;
    // Anota os indices vindos dos matches para os pontos correspondentes em cada nuvem
    for(int i=0; i<matches3d.size(); i++){
        pref.indices.push_back(matches3d[i].x);
        pnow.indices.push_back(matches3d[i].y);
    }
    // Estima a transformacao
    svd.estimateRigidTransformation(*cnow, pnow.indices, *cref, pref.indices, Tsvd);

    return Tsvd;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void RegisterObjectOptm::filterMatchesLineCoeff(vector<DMatch> &matches, vector<KeyPoint> kpref, vector<KeyPoint> kpnow, float width, float n){
    // Fazer e calcular vetor de coeficientes para cada ponto correspondente do processo de match
    vector<float> coefs(matches.size());
#pragma omp parallel for
    for(int i=0; i<matches.size(); i++){
        float xr, yr, xn, yn;
        xr = kpref[matches[i].queryIdx].pt.x;
        yr = kpref[matches[i].queryIdx].pt.y;
        xn = kpnow[matches[i].trainIdx].pt.x + width;
        yn = kpnow[matches[i].trainIdx].pt.y;
        // Calcular os coeficientes angulares
        coefs[i] = (yn - yr)/(xn - xr);
    }
    // Media e desvio padrao dos coeficientes
    float average = accumulate(coefs.begin(), coefs.end(), 0.0)/coefs.size();
    float accum = 0.0;
    for_each(coefs.begin(), coefs.end(), [&](const float d){
        accum += (d - average) * (d - average);
    });
    float stdev = sqrt(accum/(coefs.size()-1));
    // Filtrar o vetor de matches na posicao que os coeficientes estejam fora
    vector<DMatch> temp;
    for(int i=0; i<coefs.size(); i++){
        if(abs(coefs[i]-average) < n*stdev)
            temp.push_back(matches[i]);
    }
    matches = temp;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
Matrix4f RegisterObjectOptm::optmizeTransformP2P(PointCloud<PointTN>::Ptr cref, PointCloud<PointTN>::Ptr cnow, vector<Point2d> matches3d){
    // Inicia estimador
    registration::TransformationEstimationPointToPlaneLLS<PointTN, PointTN, float> p2p;
    PointIndices pref, pnow;
    Matrix4f Tp2p;
    // Anota os indices vindos dos matches para os pontos correspondentes em cada nuvem
    for(int i=0; i<matches3d.size(); i++){
        pref.indices.push_back(matches3d[i].x);
        pnow.indices.push_back(matches3d[i].y);
    }
    // Estima a transformacao
    p2p.estimateRigidTransformation(*cnow, pnow.indices, *cref, pref.indices, Tp2p);

    return Tp2p;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
int RegisterObjectOptm::searchNeighbors(MatrixXi im, int r, int c, int l){
    // Duplo for sobre a vizinhanca pra ver se encontra, e o primeiro de todos leva
    for(int i=r-l; i<r+l; i++){
        if(i >= 0 && i < im.rows()){
            for(int j=c-l; j<c+l; j++){
                if(j > 0 && j < im.cols()){
                    // Se nesse lugar esta algo diferente de -1
                    if(im(i, j) != -1)
                        return im(i, j);
                }
            }
        }
    }

    return -1;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void RegisterObjectOptm::searchNeighborsKdTree(PointCloud<PointTN>::Ptr cnow, PointCloud<PointTN>::Ptr cobj, float rate){
    // Iniciar kdtree de busca
    KdTreeFLANN<PointTN> kdtree;
    kdtree.setInputCloud(cobj);
    vector<int> pointIdxRadiusSearch;
    vector<float> pointRadiusSquaredDistance;
    float radius = 0.05;
    // Nuvem de pontos de indices bons
    PointIndices::Ptr indices (new PointIndices);
    float average_neighbors = 0, sumn = 0;
    // Achando quantidade media de vizinhos naquele raio
    for(size_t i=0; i<cnow->size(); i++)
        sumn += kdtree.radiusSearch(cnow->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    average_neighbors = sumn/float(cnow->size());
    // Para cada ponto, se ja houver vizinhos, nao seguir
    for(size_t i=0; i<cnow->size(); i++){
        if(kdtree.radiusSearch(cnow->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) <= rate*average_neighbors)
            indices->indices.push_back(i);
    }
    // Filtrar na nuvem now so os indices que estao sem vizinhos na obj
    ExtractIndices<PointTN> extract;
    extract.setInputCloud(cnow);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*cnow);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
