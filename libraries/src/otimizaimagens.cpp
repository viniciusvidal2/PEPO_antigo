#include "../include/otimizaimagens.h"

/////////////////////////////////////////////////////////////////////////////////////////////////
OtimizaImagens::OtimizaImagens(string p, string icam, string iclu, string id, string dist, string oc):pasta(p),
    arquivo_cam(icam), arquivo_clusters(iclu), arquivo_depth(id), arquivo_distancias(dist), arquivo_nuvem(oc)
{
    // Lendo imagens de uma vez
    im_cam      = imread((pasta+arquivo_cam).c_str()     );
    im_clusters = imread((pasta+arquivo_clusters).c_str());
    im_depth    = imread((pasta+arquivo_depth).c_str()   );
    im_dist     = imread((pasta+arquivo_distancias).c_str(), IMREAD_ANYDEPTH);
    im_nuvem    = imread((pasta+arquivo_nuvem).c_str()     , IMREAD_ANYDEPTH);
    // Inicia as matrizes para as imagens com arestas
    ed_cam.create(      im_cam.size()     , im_cam.type()      );
    ed_clusters.create( im_clusters.size(), im_clusters.type() );
    ed_depth.create(    im_depth.size()   , im_depth.type()    );
    // Remove a distorcao da imagem da camera com os parametros da calibracao
    Mat params = (Mat_<double>(1,5) << 0.113092, -0.577590, 0.005000, -0.008206, 0.000000);
    Mat K      = (Mat_<double>(3,3) << 1484.701399,    0.000000, im_cam.cols/2,//432.741036,
                                          0.000000, 1477.059238, im_cam.rows/2,//412.362072,
                                          0.000000,    0.000000,   1.000000);
    Mat temp, temp_l;
    undistort(im_cam,      temp  , K, params);
    undistort(im_clusters, temp_l, K, params);
    temp.copyTo(im_cam);
    temp_l.copyTo(im_clusters);
    // Salva os focos iniciais do laser para otimizacao da projecao
    fx_l = K.at<double>(Point(0, 0)); fy_l = K.at<double>(Point(1, 1));
//    vector<Mat> planos;
//    cv::split(im_nuvem, planos);
//    imshow("nuvem", planos[0]);
//    waitKey(0);
//    cvDestroyAllWindows();
}
/////////////////////////////////////////////////////////////////////////////////////////////////
OtimizaImagens::~OtimizaImagens(){

}
/////////////////////////////////////////////////////////////////////////////////////////////////
void OtimizaImagens::calculateEdgesOnImages(){
    // Corrigindo ruidos na imagem de clusters para calcular melhor edges
    im_clusters = correctColorCluster(im_clusters);
    // Converter imagens para escala de cinza
    Mat cam_gray, clusters_gray, depth_gray;
    cvtColor(im_cam     , cam_gray     , CV_BGR2GRAY);
    cvtColor(im_depth   , depth_gray   , CV_BGR2GRAY);
    cvtColor(im_clusters, clusters_gray, CV_BGR2GRAY);
    // Encontrar blobs na imagem de clusters para calcular melhor as arestas
//    clusters_gray = calculateBlobs(clusters_gray);
    // Definir valores para calculos de arestas
    int low_threshold = 10; // Maximo aqui de 100 pelo site do OpenCV
    int ratio = 3, kernel_size = 3;
    // Filtro gaussiano nas imagens para relaxar as mesmas
    blur(cam_gray     , cam_gray     , Size(5, 5));
    blur(clusters_gray, clusters_gray, Size(3, 3));
    blur(depth_gray   , depth_gray   , Size(5, 5));
    // Calcular as arestas sobre as imagens e guardar nas mascaras
    Mat mask_cam, mask_clusters, mask_depth;
    Canny(cam_gray     , mask_cam     , low_threshold, ratio*low_threshold, kernel_size);
    Canny(clusters_gray, mask_clusters, low_threshold, ratio*low_threshold, kernel_size);
    Canny(depth_gray   , mask_depth   , low_threshold, ratio*low_threshold, kernel_size);
    // Calcular contornos na imagem de clusters
    Mat clusters_cont;
    clusters_cont = calculateContours(mask_clusters);
    // Definir as imagens finais de arestas
    ed_clusters = clusters_cont;
    // Iniciar imagens com 0 onde nao ha arestas
    ed_cam      = Scalar::all(0);
//    ed_clusters = Scalar::all(0);
//    ed_depth    = Scalar::all(0);
    // Preencher com a mascara a imagem de saida com arestas
    im_cam.copyTo(     ed_cam     , mask_cam     );
//    im_clusters.copyTo(ed_clusters, mask_clusters);
//    im_depth.copyTo(   ed_depth   , mask_depth   );
    // Mostrar resultado
//    imshow("RGB", ed_cam);
//    imshow("Clusters", ed_clusters);
//    imshow("Depth", ed_depth);
    waitKey(0);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void OtimizaImagens::saveEdgeImages(){
    imwrite( (pasta+"edges_rgb.jpg").c_str()     , ed_cam      );
    imwrite( (pasta+"edges_clusters.jpg").c_str(), ed_clusters );
    imwrite( (pasta+"edges_depth.jpg").c_str()   , ed_depth    );
}
/////////////////////////////////////////////////////////////////////////////////////////////////
Mat OtimizaImagens::correctColorCluster(Mat in){
    // Limite da vizinhanca (pixels) a ser varrido em busca de uma moda de cor
    int lim = 5;
    // Varrer todos os pixels e achar os que sao cinza (excluindo as bordas segundo limite de vizinhanca)
    #pragma omp parallel for num_threads(100)
    for(int u=0+lim; u<in.cols-lim; u++){
        for(int v=0+lim; v<in.rows-lim; v++){
            // Se for cinza, varrer os vizinhos
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
Vec3b OtimizaImagens::findPredominantColor(int u, int v, Mat in, int desvio){
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
void OtimizaImagens::calcAndMatchFeatures(){
    // Keypoints e descritores para astra e zed
    std::vector<cv::KeyPoint> keypoints_cam, keypoints_clusters;
    cv::Mat descriptors_cam, descriptors_clusters;
    /// Comparando e filtrando matchs ///
    cv::Ptr<cv::DescriptorMatcher> matcher;
    matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    std::vector<std::vector< cv::DMatch > > matches;
    std::vector< cv::DMatch > good_matches;

    int tent = 0; // tentativas de achar X correspondencias bacanas
    float min_hessian = 2000;

    std::vector<cv::KeyPoint> goodKeypointsCam, goodKeypointsCluster;

    while(goodKeypointsCam.size() < 7 && tent < 20){

        good_matches.clear(); goodKeypointsCam.clear(); goodKeypointsCluster.clear();

        Ptr<xfeatures2d::SURF> f2d = xfeatures2d::SURF::create(min_hessian);

        f2d->detectAndCompute(im_cam     , Mat(), keypoints_cam     , descriptors_cam     );

        f2d->detectAndCompute(im_depth, Mat(), keypoints_clusters, descriptors_clusters);

        matcher->knnMatch(descriptors_cam, descriptors_clusters, matches, 2);

        for (size_t i = 0; i < matches.size(); i++)
        {
            if (matches.at(i).size() >= 2)
            {
                if (matches.at(i).at(0).distance < 0.75*matches.at(i).at(1).distance)
                {
                    good_matches.push_back(matches.at(i).at(0));
                }
            }
        }

        tent += 1;
        min_hessian = 0.7*min_hessian;

        std::vector<cv::Point2f> imgLeftPts;
        std::vector<cv::Point2f> imgRightPts;

        for (size_t i = 0; i < good_matches.size(); i++){
                goodKeypointsCam.push_back(keypoints_cam[good_matches[i].queryIdx]);
                goodKeypointsCluster.push_back(keypoints_clusters[good_matches[i].trainIdx]);
                imgLeftPts.push_back(keypoints_cam[good_matches[i].queryIdx].pt);
                imgRightPts.push_back(keypoints_clusters[good_matches[i].trainIdx].pt);
        }

        if(imgLeftPts.size() > 0){
            cv::Mat inliers;
            cv::Mat Ka = (cv::Mat_<double>(3, 3) << 1484.701399, 0.0, 432.741036, 0.0, 1477.059238, 412.362072, 0.0, 0.0, 1.0);
            cv::Mat E = findEssentialMat(imgLeftPts, imgRightPts, Ka, CV_RANSAC, 0.99999, 1.0, inliers);

            std::vector<cv::KeyPoint> goodKeypointsCamTemp;
            std::vector<cv::KeyPoint> goodKeypointsClusterTemp;
            bool dx = false, dy = false;
            for (size_t i = 0; i < inliers.rows; i++){
                if (inliers.at<uchar>(i, 0) == 1 && dx && dy){
                    goodKeypointsCamTemp.push_back(goodKeypointsCam.at(i));
                    goodKeypointsClusterTemp.push_back(goodKeypointsCluster.at(i));
                }
            }
            goodKeypointsCam     = goodKeypointsCamTemp;
            goodKeypointsCluster = goodKeypointsClusterTemp;
        }

    } // fim do while

    if(goodKeypointsCam.size()){
        Mat c, cl;
        im_cam.copyTo(c); im_depth.copyTo(cl);
        for(size_t i=0; i<goodKeypointsCam.size(); i++){
            cv::Scalar color = cv::Scalar(rand() % 255, rand() % 255, rand() % 255);
            circle(c, goodKeypointsCam[i].pt, 5, color, 2);
            circle(cl, goodKeypointsCluster[i].pt, 5, color, 2);
        }
        imshow("Features camera", c);
        imshow("Features clusters", cl);
        waitKey(0);
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
Mat OtimizaImagens::calculateBlobs(Mat in){
    // Parametros
    SimpleBlobDetector::Params params;
    params.filterByArea = true;
    params.minArea = 200; // [pixels]
    params.filterByCircularity = false;
    params.filterByColor = false;
    params.filterByConvexity = false;
    params.filterByInertia = false;
    // Detector
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
    vector<KeyPoint> keypoints;
    // Detectando
    detector->detect(in, keypoints);
    // Plotando aqui
    Mat im_with_keypoints;
    drawKeypoints(in, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    // Show blobs
    imshow("keypoints", im_with_keypoints);
    waitKey(0);

    return in;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
Mat OtimizaImagens::calculateContours(Mat in){
    // Encontrando os contornos
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours( in, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    // Filtrando por tamanho dos contornos
    for(vector<vector<Point>>::iterator it = contours.begin(); it!= contours.end(); ){
        if(it->size() < 100)
            it=contours.erase(it);
        else
            ++it;
    }

    // Desenhando
//    RNG rng(12345);
    Mat drawing = Mat::zeros( in.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ ){
        Scalar color = Scalar(255, 0, 0);
        drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
    }

    return drawing;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void OtimizaImagens::adjustImagesKeyboard(){
    // Imagem com a soma das imagens de arestas
    Mat soma;
    // Tecla pressionada
    int t;
    // Offsets em X e Y e passo em pixels de deslocamento
    int offx = 0, offy = 0, passo = 5;
    // Foco anterior e foco atual em X e Y
    float fx_p = fx_l, fy_p = fy_l, fx_c = fx_l, fy_c = fy_l;
    // Comeca a soma de imagens
    Mat aux;
    ed_clusters.copyTo(aux);
    addWeighted(ed_cam, 1.0, aux, 1.0, 0.0, soma);
    namedWindow("Ajuste teclado");
    imshow("Ajuste teclado", soma);
    // Loop sobre as teclas
    while(t != 32){ // espaco acaba tudo
        t = waitKey(0);
        switch(t){
        case 99: // letra c, para esquerda
            offx -= passo;
            break;
        case 102: // letra f, para cima
            offy -= passo;
            break;
        case 98: // letra b, para direita
            offx += passo;
            break;
        case 118: // letra v, para baixo
            offy += passo;
            break;
        case 97:  // letra a, reduz passo
            if(passo > 2)
                passo -= 1;
            break;
        case 115: // letra s, aumenta passo
            passo += 1;
            break;
        case 105: // letra i, diminui foco em X
            fx_c -= passo;
            break;
        case 111: // letra o, aumenta o foco em X
            fx_c += passo;
            break;
        case 107: // letra k, diminui o foco em Y
            fy_c -= passo;
            break;
        case 108: // letra l, aumenta o foco em Y
            fy_c += passo;
            break;
        default:
            break;
        }
        // Altera imagens com novos focos e atualiza valores dos focos
//        ed_clusters = adjustImageByFocus(ed_clusters, fx_c/fx_p, fy_c/fy_p);
        ed_clusters = adjustImageByProjection(ed_clusters, fx_c, fy_c, offx, offy);
        fx_p = fx_c; fy_p = fy_c;
        offx = 0; offy = 0; // Se for pra ajustar por projecao
        ed_clusters.copyTo(aux); // A principio para nao depender do offset
        // Cria a matriz com os offsets e desloca a danada dos clusters
        if(offx != 0 || offy != 0){
          Mat H = (cv::Mat_<double>(3,3) << 1, 0, offx, 0, 1, offy, 0, 0, 1);
          warpPerspective(ed_clusters, aux, H, ed_clusters.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, Scalar(0, 0, 0));
        }
        addWeighted(ed_cam, 1.0, aux, 1.0, 0.0, soma);
        imshow("Ajuste teclado", soma);
        cout << "\nOffset X: " << offx << "    Offset Y: " << offy << "    Passo: " << passo << "    Foco X: " << fx_c << endl;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
Mat OtimizaImagens::adjustImageByFocus(Mat in, float fx_r, float fy_r){
    // Procura os pixels que nao sao 0, tem distancia, na matriz de distancias
    Mat temp_edge = Mat::zeros(in.size()     , in.type()     );
    Mat temp_dist = Mat::zeros(im_dist.size(), im_dist.type());
    #pragma omp parallel for num_threads(100)
    for(int i=0; i<in.cols; i++){
        for(int j=0; j<in.rows; j++){
            // Se ha distancia, recalcular a posicao (u, v) daquele pixel na imagem de arestas do laser
            unsigned short dist_pixel = im_dist.at<unsigned short>(Point(i, j));
            float dist_float = (static_cast<float>(dist_pixel)-3.0) / 1000.0;
//            if(dist_pixel > 0){
                // Pegar a posicao antiga e achar uma nova com a taxa entre os focos
                float u = float(i) * fx_r;// * dist_float;
                float v = float(j) * fy_r;// * dist_float;
                // Se cair dentro das dimensoes das imagens, prosseguir
                if(u > 0 && u < in.cols && v > 0 && v < in.rows){
                    // Alterar na nova imagem de arestas
                    temp_edge.at<Vec3b>(Point(int(u),int(v)))          = in.at<Vec3b>(Point(i,j));
                    // Alterar na nova imagem de distancias, para seguir a perseguicao na proxima iteracao
                    temp_dist.at<unsigned short>(Point(int(u),int(v))) = dist_pixel;
                }
//            }
        }
    }

    // Renovar a imagem de distancias de dentro da classe
    temp_dist.copyTo(im_dist);

    // Retornar a imagem temporaria de arestas nova
    return temp_edge;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
Mat OtimizaImagens::adjustImageByProjection(Mat in, float fx, float fy, float tx, float ty){
    // Procura os pixels que nao sao 0, tem distancia, na matriz de distancias
    Mat temp_edge  = Mat::zeros(in.size()      , in.type()     );
    Mat temp_nuvem = Mat::zeros(im_nuvem.size(), im_nuvem.type());
    // Matriz intrinseca do laser
    Eigen::Matrix3f Kl;
    Kl << fx,  0, in.cols/2,
           0, fy, in.rows/2,
           0,  0,  1;
    Eigen::MatrixXf Rt(3, 4);
    Rt << 1, 0, 0, tx/100.0,
          0, 1, 0, ty/100.0,
          0, 0, 1,    0    ;
    Eigen::MatrixXf P = Kl*Rt;
    // Loop de teste de projecao
    #pragma omp parallel for num_threads(100)
    for(int i=0; i<in.cols; i++){
        for(int j=0; j<in.rows; j++){
            // Se ha distancia em Z, recalcular a posicao (u, v) daquele pixel na imagem de arestas do laser
            Vec3w ponto = im_nuvem.at<Vec3w>(Point(i, j));
            Eigen::MatrixXf X_(4, 1), X(3, 1);
            X_ << float(ponto.val[0])/1000.0-3.0, float(ponto.val[1])/1000.0-3.0, float(ponto.val[2])/1000.0-3.0, 1.0;
            X = P*X_;

            cout << X_.transpose() << endl;
            float u=0, v=0;
            if(X(2, 0) > 0)
                float u = X(0,0)/X(2,0), v = X(1,0)/X(2,0);
//            float X = float(ponto.val[0])/1000.0-3.0, Y = float(ponto.val[1])/1000.0-3.0, Z = float(ponto.val[2])/1000.0-3.0;
//            if(Z > 0){
//                cout << ponto.val[0] << "   " << ponto.val[1]  << "   " << ponto.val[2] << endl;
//                // Pegar a posicao antiga e achar uma nova com a taxa entre os focos - matriz de projecao desenvolvida
//                float u = fx*(X + tx*0.01)/Z + float(temp_edge.cols)/2; // trazendo o tx e ty para metros aqui
//                float v = fy*(Y + ty*0.01)/Z + float(temp_edge.rows)/2;
//                cout << u << "   " << v << endl;
                // Se cair dentro das dimensoes das imagens, prosseguir
                if(u > 0 && u < in.cols && v > 0 && v < in.rows){
                    // Alterar na nova imagem de arestas
                    temp_edge.at< Vec3b>(Point(int(u),int(v))) = in.at<Vec3b>(Point(i,j));
                    // Alterar na nova imagem de distancias, para seguir a perseguicao na proxima iteracao
                    temp_nuvem.at<Vec3w>(Point(int(u),int(v))) = ponto;
                }
//            }
        }
    }

    // Renovar a imagem da nuvem organizada de dentro da classe
    temp_nuvem.copyTo(im_nuvem);

    // Retornar a imagem temporaria de arestas nova
    return temp_edge;
}
/////////////////////////////////////////////////////////////////////////////////////////////////
