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
    blur(clusters_gray, clusters_gray, Size(11, 11));
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
    imshow("Clusters", ed_clusters);
//    imshow("Depth", ed_depth);
    waitKey(0);
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void OtimizaImagens::saveEdgeImages(){
    imwrite((pasta+"edges_rgb.jpg").c_str()     , ed_cam     );
    imwrite((pasta+"edges_clusters.jpg").c_str(), ed_clusters);
    imwrite((pasta+"edges_depth.jpg").c_str()   , ed_depth   );
}
/////////////////////////////////////////////////////////////////////////////////////////////////
