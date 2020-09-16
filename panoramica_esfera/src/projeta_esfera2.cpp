#include <ros/ros.h>
#include <iostream>
#include <string>
#include <math.h>
#include <sys/stat.h>
#include <ostream>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "../../libraries/include/processcloud.h"

/// Definicoes e namespaces
///
using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace std;
using namespace Eigen;
typedef PointXYZRGBNormal PointTN;
typedef PointXYZRGB       PointC;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int getdir(string dir, vector<string> &nuvens){
  // Abrindo a pasta raiz para contar os arquivos de imagem e nuvem que serao lidos e enviados
  DIR *dp;
  struct dirent *dirp;
  string nome_temp;
  if((dp  = opendir(dir.c_str())) == NULL)
    ROS_ERROR("Nao foi possivel abrir o diretorio");

  while ((dirp = readdir(dp)) != NULL) {
    nome_temp = string(dirp->d_name);
    // Todas as nuvens na pasta
    if(nome_temp.substr(nome_temp.find_last_of(".")+1) == "ply")
      nuvens.push_back(nome_temp);
  }
  closedir(dp);

  return 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Matrix4f calculateCameraPose(Quaternion<float> q, Vector3f &C, int i){
    Matrix3f r = q.matrix();
    Vector3f t = C;

    Matrix4f T = Matrix4f::Identity();
    T.block<3,3>(0, 0) = r.transpose(); T.block<3,1>(0, 3) = t;

    return T;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void createFrustrumPlane(PointCloud<PointTN>::Ptr plano, Vector3f p2, Vector3f p4, Vector3f p5, Mat im){
    // Resolucao a ser somada igual a da imagem, dividir os vetores pela resolucao da imagem
    Vector3f hor_step, ver_step;
    hor_step = (p4 - p5) / float(im.cols);
    ver_step = (p2 - p5) / float(im.rows);
    plano->resize(im.rows*im.cols);
    // Criar a posicao de cada ponto em 3D e ainda atribuir a cor ali
    // Posicao 3D somando os vetores com a origem em p5
    omp_set_dynamic(0);
#pragma omp parallel for num_threads(20)
    for(int i=0; i < im.rows; i++){
        for(int j=0; j < im.cols; j++){
            Vector3f ponto;
            PointTN ponto_nuvem;
            ponto = p5 + hor_step*j + ver_step*i;
            ponto_nuvem.x = ponto(0); ponto_nuvem.y = ponto(1); ponto_nuvem.z = ponto(2);
            ponto_nuvem.r = im.at<Vec3b>(Point(j, im.rows-1 - i))[2];
            ponto_nuvem.g = im.at<Vec3b>(Point(j, im.rows-1 - i))[1];
            ponto_nuvem.b = im.at<Vec3b>(Point(j, im.rows-1 - i))[0];
            // Adiciona ponto na nuvem do plano de saida
            plano->points[i*im.cols + j] = ponto_nuvem;
        }
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PointTN intersectSphere(Vector3f C, PointTN ponto_plano, float R){
    Vector3f p(ponto_plano.x, ponto_plano.y, ponto_plano.z);
    // Equacao da reta saindo de C para ponto_plano
    Vector3f pDiff = p - C;
    float x0 = C(0)    , y0 = C(1)    , z0 = C(2);
    float xd = pDiff(0), yd = pDiff(1), zd = pDiff(2);
    // Equacao da reta na origem e x2 + y2 + z2 = R2
    // Intersecao com a equacao da reta gera duas solucoes para t. Os coeficientes da equacao de
    // segundo grau para resolver Bhaskara seriam a, b e c
    float a, b, c, D, t1, t2;
    a = xd*xd + yd*yd + zd*zd;
    b = 2*( x0*xd + y0*yd + z0*zd );
    c = x0*x0 + y0*y0 + z0*z0 - R*R;
    // Duas solucoes para dois pontos de intersecao, o mais perto e o que segue
    D = sqrt(b*b - 4*a*c);
//    cout << "a: " << xd << "  b: " << yd << "  c: " << zd << "  d:  " << D << endl;
    t1 = (-b + D) / (2*a);
    t2 = (-b - D) / (2*a);
    Vector3f int1, int2;
    PointTN final;
    int1 = C + t1*pDiff;
    int2 = C + t2*pDiff;
//    cout << "t1: " << t1 << "  t2: " << t2 << endl;
//    if( (p - int1).norm() < (p - int2).norm() ){
    if( t1 > 0 ){
        final.x = int1(0); final.y = int1(1); final.z = int1(2);
    } else {
        final.x = int2(0); final.y = int2(1); final.z = int2(2);
    }

    return final;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float calculateFrustrumFromPose(Quaternion<float> q, Matrix4f T, float r, Vector3f C){
    // Comprimento do eixo focal no frustrum
    float F;
    // Criar vetor unitario n na orientacao vinda do quaternion
    Vector3f k(0, 0, 1);
    Vector3f n = q.matrix()*k;
    // Calculo de P1 a partir da matriz de transformacao homogenea e o ponto central homogeneo
    Vector4f Ch(C[0], C[1], C[2], 1);
    Vector4f p1h = T*Ch;
    Vector3f p1 = p1h.block<3,1>(0, 0);
//    cout << "T:  " << T << endl;
//    cout << "p1:  " << p1h.transpose() << endl;
    // Definir coeficientes do plano a, b e c pelo vetor unitario n
    float a = n(0), b = n(1), c = n(2);
    // Achar a intersecao da reta que vai do centro da esfera C, com a orientacao de n, ate a superficie da esfera
    // pois o plano sera tangente ali
    Vector3f p_tangente = C + r*n; // Multiplicar o vetor unitario de orientacao pelo raio
//    cout << "p_tangente:  " << p_tangente.transpose() << endl;
    // Calcular coeficiente d do plano tangente
    float d = -(a*p_tangente(0) + b*p_tangente(1) + c*p_tangente(2));

//    cout << "a: " << a << "   b: " << b << "   c: " << c << "  d: " << d << endl;
    // Valor de F a partir da distancia entre o centro da camera p1 e o plano tangente
    float num = abs( a*p1(0) + b*p1(1) + c*p1(2) + d ), den = sqrt( a*a + b*b + c*c );
    F = num/den;

    return F;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "projeta_esfera2");
    ros::NodeHandle nh;
    ros::NodeHandle n_("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    // Objeto da classe de projecao de nuvens
    ProcessCloud pc;

    string nome_param;
    n_.param("pasta", nome_param, string("Dados_PEPO1"));

    // Define pasta a ser vasculhada e arquivo NVM
    char* home;
    home = getenv("HOME");
    std::string pasta = std::string(home)+"/Desktop/"+nome_param+"/";
    cout << pasta << endl;
    std::string arquivo_nvm = pasta + "cameras.nvm";

    /// Le arquivo NVM e carrega tudo nos vetores de variaveis respectivas
    ///
    ifstream nvm(arquivo_nvm);
    int contador_linhas = 1, espacamento = 4;
    vector<Quaternion<float>> rots;
    vector<Vector3f> Cs;
    vector<std::string> nomes_imagens, linhas;
    std::string linha;
    ROS_INFO("Abrindo e lendo arquivo NVM ...");
    if(nvm.is_open()){
        while(getline(nvm, linha)){
            if(contador_linhas > 3 && linha.size() > 4 && (contador_linhas-3) % espacamento == espacamento-1)
                linhas.push_back(linha);

            contador_linhas++;
        }
    } else {
        ROS_ERROR("Arquivo de cameras nao encontrado. Desligando ...");
        ros::shutdown();
    }
    // Alocar nos respectivos vetores
    rots.resize(linhas.size()); Cs.resize(linhas.size()), nomes_imagens.resize(linhas.size());
    float foco;
    // Para cada imagem, obter valores
    for(int i=0; i < linhas.size(); i++){
        istringstream iss(linhas[i]);
        vector<string> splits(istream_iterator<string>{iss}, istream_iterator<string>());
        // Nome
        string nome_fim = splits[0].substr(splits[0].find_last_of('/')+1, splits[0].size()-1);
        nomes_imagens[i] = pasta+nome_fim;
        // Foco
        foco = stof(splits[1]);
        // Quaternion
        Quaternion<float> q;
        q.w() = stof(splits[2]); q.x() = stof(splits[3]); q.y() = stof(splits[4]); q.z() = stof(splits[5]);
        rots[i] = q;
        // Centro
        Vector3f C(stof(splits[6]), stof(splits[7]), stof(splits[8]));
        Cs[i] = C;
    }

    /// Ler todas as nuvens, somar e salvar
    ///
    struct stat buffer;
    string nome_acumulada = pasta + "acumulada_hd.ply";
    if(stat(nome_acumulada.c_str(), &buffer)){
        ROS_INFO("Lendo e salvando as nuvens ...");
        vector<string> nomes_nuvens;
        getdir(pasta, nomes_nuvens);
        PointCloud<PointC>::Ptr acc   (new PointCloud<PointC>);
        PointCloud<PointTN>::Ptr accn (new PointCloud<PointTN>);
        vector<PointCloud<PointC>> acc_vec(nomes_nuvens.size());
        omp_set_dynamic(0);
        #pragma omp parallel for num_threads(15)
        for(int i=0; i < nomes_nuvens.size(); i++){
            PointCloud<PointC>::Ptr  temp  (new PointCloud<PointC>);
            ROS_INFO("Processando nuvem %d ...", i+1);
            loadPLYFile<PointC>(pasta+nomes_nuvens[i], *temp);
            StatisticalOutlierRemoval<PointC> sor;
            sor.setInputCloud(temp);
            sor.setMeanK(5);
            sor.setStddevMulThresh(2);
            sor.filter(*temp);

            acc_vec[i] = *temp;
            temp->clear();
        }
        for(auto pc : acc_vec)
            *acc += pc;
        acc_vec.clear();
        ROS_INFO("Calculando a normal na nuvem final ...");
        pc.calculateNormals(acc, accn);
        savePLYFileBinary<PointTN>(nome_acumulada, *accn);
        acc->clear(); accn->clear();
    }

    /// Desenha a ESFERA - adicionando pontos
    ///
    PointCloud<PointTN>::Ptr esfera (new PointCloud<PointTN>);
    float R = 1; // Raio da esfera [m]
    // Angulos para lat e lon, 360 de range para cada, resolucao a definir no step_deg
    float step_deg = 0.2; // [DEGREES]
    int raios_360 = int(360.0/step_deg), raios_180 = raios_360/2; // Quantos raios sairao do centro para formar 360 e 180 graus de um circulo 2D

    string nome_nuvem_esfera = pasta+"esfera_raw.ply";

    ROS_INFO("Criando esfera com resolucao de %.4f graus ...", step_deg);
    esfera->resize(raios_360*raios_180);
    omp_set_dynamic(0);
#pragma omp parallel for num_threads(50)
    for (int a = 0; a < raios_180; a++) {
        for (int o = 0; o < raios_360; o++) {
            PointTN ponto;
            float lat, lon;
            lat = DEG2RAD(a*step_deg), lon = DEG2RAD(o*step_deg);
            // Calculo segundo latitude e longitude do ponto para coordenadas - latitude corre em Y !!!
            ponto.y = R*cos(lat);
            ponto.x = R*sin(lat)*cos(-lon);
            ponto.z = R*sin(lat)*sin(-lon);
            //                ponto.g = ponto.y < 0   ? abs(ponto.y)/R*250 : 0;
            //                ponto.r = ponto.y > 0   ? abs(ponto.y)/R*250 : 0;
            //                ponto.b = ponto.x > R/2 ? abs(ponto.x)/R*250 : 0;
            ponto.normal_x = o; ponto.normal_y = raios_180-1 - a; ponto.normal_z = 0;
            // Adiciona ponto na esfera
            esfera->points[a*raios_360 + o] = ponto;
        }
    }

    /// Para cada imagem
    ///

    ROS_INFO("Rodando o processo das imagens sobre a esfera ...");
    omp_set_dynamic(0);
    #pragma omp parallel for num_threads(4)
    for(int i=0; i < nomes_imagens.size(); i++){

        ROS_INFO("Projetando imagem %d ...", i+1);
        // Ler a imagem a ser usada
        Mat image = imread(nomes_imagens[i]);
        if(image.cols < 3){
            ROS_ERROR("Imagem nao foi encontrada, checar NVM ...");
            ros::shutdown();
        }
        // Parametros da camera
        Mat params = (Mat_<double>(1,5) << 0.0723, -0.1413, -0.0025 -0.0001, 0.0000);
        Mat K      = (Mat_<double>(3,3) << foco,  0.0, image.cols/2,
                                            0.0, foco, image.rows/2,
                                            0.0,  0.0,     1.0     );
        // Tirar distorcao da imagem - fica melhor sim o resultado ou proximo
        Mat temp;
        undistort(image, temp, K, params);
        temp.copyTo(image);
        // Calcular a vista da camera pelo Rt inverso - rotacionar para o nosso mundo, com Z para cima
        ROS_INFO("Calculando pose da camera e frustrum da imagem %d...", i+1);
        Matrix4f T = calculateCameraPose(rots[i], Cs[i], i);
        // Definir o foco em dimensoes fisicas do frustrum
        float F = R;//calculateFrustrumFromPose(rots[i], T, R, Cs[i]);
        Vector3f C = Cs[i];
        double minX, minY, maxX, maxY;
        maxX = F*(float(image.cols) / (2.0*foco));
        minX = -maxX;
        maxY = F*(float(image.rows) / (2.0*foco));
        minY = -maxY;
        // Calcular os 4 pontos do frustrum
        /*
                                origin of the camera = p1
                                p2--------p3
                                |          |
                                |  pCenter |<--- Looking from p1 to pCenter
                                |          |
                                p5--------p4
        */
        Vector4f p, p1, p2, p3, p4, p5, pCenter;
        float p_dist = sqrt( pow(F, 2) + sqrt(pow(minX, 2) + pow(minY, 2)) ); // Comprimento da diagonal que vai do centro da camera a cada ponto pX
        p << 0, 0, 0, 1;
        p1 = T*p;
        p << minX, minY, F, 1;
        p2 = T*p;
        p << maxX, minY, F, 1;
        p3 = T*p;
        p << maxX, maxY, F, 1;
        p4 = T*p;
        p << minX, maxY, F, 1;
        p5 = T*p;
        p << 0, 0, F, 1;
        pCenter = T*p;
        // Criar plano colorido a partir dos vetores de canto
        PointCloud<PointTN>::Ptr plano_cam (new PointCloud<PointTN>);
        createFrustrumPlane(plano_cam, p2.block<3,1>(0, 0), p4.block<3,1>(0, 0), p5.block<3,1>(0, 0), image);
        PointTN tempP;
        tempP.x = p5(0); tempP.y = p5(1); tempP.z = p5(2);
        tempP.r = 100; tempP.g = 250; tempP.b = 40;
        plano_cam->push_back(tempP);
        tempP.x = p4(0); tempP.y = p4(1); tempP.z = p4(2);
        tempP.r = 0; tempP.g = 250; tempP.b = 0;
        plano_cam->push_back(tempP);
        tempP.x = p3(0); tempP.y = p3(1); tempP.z = p3(2);
        tempP.r = 0; tempP.g = 250; tempP.b = 0;
        plano_cam->push_back(tempP);
        tempP.x = p2(0); tempP.y = p2(1); tempP.z = p2(2);
        tempP.r = 0; tempP.g = 250; tempP.b = 0;
        plano_cam->push_back(tempP);
        tempP.x = p1(0); tempP.y = p1(1); tempP.z = p1(2);
        tempP.r = 0; tempP.g = 250; tempP.b = 0;
        plano_cam->push_back(tempP);
        tempP.x = pCenter(0); tempP.y = pCenter(1); tempP.z = pCenter(2);
        tempP.r = 0; tempP.g = 250; tempP.b = 0;
        plano_cam->push_back(tempP);
        savePLYFileBinary<PointTN>(pasta+"plano_tangente"+std::to_string(i+1)+".ply", *plano_cam);
        // Para cada ponto no plano
        ROS_INFO("Ajustando plano do frustrum %d sobre a esfera ...", i+1);
        KdTreeFLANN<PointTN> tree; // Kdtree utilizada na esfera
        tree.setInputCloud(esfera);
        omp_set_dynamic(0);
#pragma omp parallel for num_threads(100)
        for(size_t k=0; k < plano_cam->size(); k++){
            // Achar intersecao entre a reta que sai do centro da camera ate o ponto do plano e a esfera
            PointTN inters = intersectSphere(p1.block<3,1>(0, 0), plano_cam->points[k], R);
            if(!isnan(inters.x) && !isnan(inters.y) && !isnan(inters.z)){
                // Encontrar ponto na esfera mais proximo da intersecao por Kdtree
                vector<int> indices;
                vector<float> distancias_elevadas;
                tree.nearestKSearch(inters, 1, indices, distancias_elevadas);
                // Se encontrado, pintar ponto na esfera com a cor do ponto no plano
                if(indices.size() > 0){
                    esfera->points[indices[0]].r = plano_cam->points[k].r;
                    esfera->points[indices[0]].g = plano_cam->points[k].g;
                    esfera->points[indices[0]].b = plano_cam->points[k].b;
                }
            }
        }

    } // Fim do for imagens

    // Salvar a nuvem da esfera agora colorida
    ROS_INFO("Salvando esfera final colorida ...");
    savePLYFileBinary<PointTN>(pasta+"esfera_cor.ply", *esfera);

    // Salva imagem circular ao final
    ROS_INFO("Salvando imagem esferica ...");
    Mat imagem_esferica = Mat::zeros( Size(raios_360, raios_180), CV_8UC3 );
    omp_set_dynamic(0);
#pragma omp parallel for num_threads(40)
    for(size_t i = 0; i < esfera->size(); i++){
        // Coordenada da imagem salva na normal
        int u = esfera->points[i].normal_x, v = esfera->points[i].normal_y;
        // Pega as cores da nuvem ali e coloca na imagem final
        Vec3b cor_im;
        cor_im[0] = esfera->points[i].b; cor_im[1] = esfera->points[i].g; cor_im[2] = esfera->points[i].r;
        imagem_esferica.at<Vec3b>(Point(u, v)) = cor_im;
    }
    // Salvando imagem esferica final
    imwrite(pasta+"imagem_esferica.png", imagem_esferica);

    ROS_INFO("Processo finalizado.");
    ros::spinOnce();
    return 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
