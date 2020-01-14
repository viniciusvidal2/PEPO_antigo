#include "../include/otimizaimagens.h"

/////////////////////////////////////////////////////////////////////////////////////////////////
OtimizaImagens::OtimizaImagens(string p, string icam, string iclu, string id):pasta(p),
    arquivo_cam(icam), arquivo_clusters(iclu), arquivo_depth(id)
{
    // Lendo imagens de uma vez
    im_cam      = imread((pasta+arquivo_cam).c_str()     );
    im_clusters = imread((pasta+arquivo_clusters).c_str());
    im_depth    = imread((pasta+arquivo_depth).c_str()   );
    // Inicia as matrizes para as imagens com arestas
    ed_cam.create(      im_cam.size()     , im_cam.type()      );
    ed_clusters.create( im_clusters.size(), im_clusters.type() );
    ed_depth.create(    im_depth.size()   , im_depth.type()    );
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
    cvtColor(im_clusters, clusters_gray, CV_BGR2GRAY);
    cvtColor(im_depth   , depth_gray   , CV_BGR2GRAY);
    // Definir valores para calculos de arestas
    int low_threshold = 20; // Maximo aqui de 100 pelo site do OpenCV
    int ratio = 3, kernel_size = 3;
    // Filtro gaussiano nas imagens para relaxar as mesmas
    blur(cam_gray     , cam_gray     , Size(3, 3));
    blur(clusters_gray, clusters_gray, Size(3, 3));
    blur(depth_gray   , depth_gray   , Size(5, 5));
    // Calcular as arestas sobre as imagens e guardar nas mascaras
    Mat mask_cam, mask_clusters, mask_depth;
    Canny(cam_gray     , mask_cam     , low_threshold, ratio*low_threshold, kernel_size);
    Canny(clusters_gray, mask_clusters, low_threshold, ratio*low_threshold, kernel_size);
    Canny(depth_gray   , mask_depth   , low_threshold, ratio*low_threshold, kernel_size);
    // Iniciar imagens com 0 onde nao ha arestas
    ed_cam      = Scalar::all(0);
    ed_clusters = Scalar::all(0);
    ed_depth    = Scalar::all(0);
    // Preencher com a mascara a imagem de saida com arestas
    im_cam.copyTo(     ed_cam     , mask_cam     );
    im_clusters.copyTo(ed_clusters, mask_clusters);
    im_depth.copyTo(   ed_depth   , mask_depth   );
    // Mostrar resultado
    imshow("RGB", ed_cam);
    imshow("Clusters", im_clusters);
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
            if(cor.val[0] == 100 && cor.val[1] == 100 && cor.val[2] == 100){ // A imagem criada tem esse valor para pixels nao projetados
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

//        if(imgLeftPts.size() > 0){
//            cv::Mat inliers;
//            cv::Mat Ka = (cv::Mat_<double>(3, 3) << 1484.701399, 0.0, 432.741036, 0.0, 1477.059238, 412.362072, 0.0, 0.0, 1.0);
//            cv::Mat E = findEssentialMat(imgLeftPts, imgRightPts, Ka, CV_RANSAC, 0.99999, 1.0, inliers);

//            std::vector<cv::KeyPoint> goodKeypointsCamTemp;
//            std::vector<cv::KeyPoint> goodKeypointsClusterTemp;
//            bool dx = false, dy = false;
//            for (size_t i = 0; i < inliers.rows; i++){
//                if (inliers.at<uchar>(i, 0) == 1 && dx && dy){
//                    goodKeypointsCamTemp.push_back(goodKeypointsCam.at(i));
//                    goodKeypointsClusterTemp.push_back(goodKeypointsCluster.at(i));
//                }
//            }
//            goodKeypointsCam     = goodKeypointsCamTemp;
//            goodKeypointsCluster = goodKeypointsClusterTemp;
//        }

//        cout << endl << goodKeypointsCam.size() << endl;

    } // fim do while

    if(goodKeypointsCam.size()){
//        char* home;
//        home = getenv("HOME");
//        std::string pasta = std::string(home)+"/Desktop/teste/";
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
//        std::string foto_zed = pasta+"fotozed.jpeg", foto_astra = pasta+"fotoastra.jpeg";
//        imwrite(foto_astra, a);
//        imwrite(foto_zed,   z);
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////
