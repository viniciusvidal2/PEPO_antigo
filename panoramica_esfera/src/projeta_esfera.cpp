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

/// Definicoes e namespaces
///
using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace std;
using namespace Eigen;
typedef PointXYZRGBNormal PointT ;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Matrix4f calculateCameraPose(Quaternion<float> q, Vector3f C){
    Matrix3f r = q.matrix();
    Vector3f t = r.transpose()*C;

    Matrix4f T = Matrix4f::Identity();
    T.block<3,3>(0, 0) = r; T.block<3,1>(0, 3) = t;

    return T.inverse();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Vector4f calculatePlane(Vector4f p1, Vector4f p2, Vector4f p3){
    float a1 = p2(0) - p1(0); // x2 - x1;
    float b1 = p2(1) - p1(1); // y2 - y1;
    float c1 = p2(2) - p1(2); // z2 - z1; // Subtracao dos dois vetores
    float a2 = p3(0) - p1(0); // x3 - x1; // p3-p1 e p2-p1
    float b2 = p3(1) - p1(1); // y3 - y1;
    float c2 = p3(2) - p1(2); // z3 - z1;
    float a = b1 * c2 - b2 * c1;
    float b = a2 * c1 - a1 * c2;
    float c = a1 * b2 - b1 * a2;
    float d = (- a * p1(0) - b * p1(1) - c * p1(2));

    Vector4f coefs(a, b, c, d);
    return coefs;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Vector3f intersectLinePlane(Vector3f pCenter, Vector3f p, Vector4f plane){
    // Diferenca do ponto para o centro da camera, vetor ira do centro da camera para o ponto
    Vector3f pDiff = p - pCenter;
    // Descobrindo t, multiplicador da equacao vetorial da reta
    float a = plane(0), b = plane(1), c = plane(2), d = plane(3);
    float t = - (a*pCenter(0) + b*pCenter(1) + c*pCenter(2) + d) / (a*pDiff(0) + b*pDiff(1) + c*pDiff(2));
    // Calculando ponto a partir do ponto inicial da reta - centro da camera
    Vector3f inters = pCenter + t*pDiff;

    return inters;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool isInsideCam(Vector3f p_esf, Vector3f p, Vector3f p2, Vector3f p3, Vector3f p4, Vector3f p5, Vector3f C, float R){
    // Para ver se estamos dentro do retangulo e preciso checar todos os angulos possiveis entre o ponto
    // e os cantos do retangulo. Se um deles for maior que 90 graus, o ponto esta fora
    //
    // Os dois lados da esfera serao projetados sobre o plano, portanto o lado oposto deve ser cortado.
    // Se a distancia do ponto da esfera ao centro da camera for maior que o raio R da esfera, estamos
    // do lado errado.

    bool angulos_coerentes = false; // Contador de quantos cantos a distancia foi menor
    // Calculando os angulos com os pontos das cameras, utilizando as arestas na vertical 2-5 e 3-4
    float cos_p25 = ( (p - p2).dot(p5 - p2)) / ( (p - p2).norm()*(p5 - p2).norm() );
    float cos_p52 = ( (p - p5).dot(p2 - p5)) / ( (p - p5).norm()*(p2 - p5).norm() );
    float cos_p34 = ( (p - p3).dot(p4 - p3)) / ( (p - p3).norm()*(p4 - p3).norm() );
    float cos_p43 = ( (p - p4).dot(p3 - p4)) / ( (p - p4).norm()*(p3 - p4).norm() );
    // Calculando os angulos com os pontos das cameras, utilizando as arestas na horizontal 2-3 e 5-4
    float cos_p23 = ( (p - p2).dot(p3 - p2)) / ( (p - p2).norm()*(p3 - p2).norm() );
    float cos_p32 = ( (p - p3).dot(p2 - p3)) / ( (p - p3).norm()*(p2 - p3).norm() );
    float cos_p54 = ( (p - p5).dot(p4 - p5)) / ( (p - p5).norm()*(p4 - p5).norm() );
    float cos_p45 = ( (p - p4).dot(p5 - p4)) / ( (p - p4).norm()*(p5 - p4).norm() );
    if( cos_p25 > 0 && cos_p52 > 0 && cos_p34 > 0 && cos_p43 > 0 && cos_p23 > 0 && cos_p32 > 0 && cos_p54 > 0 && cos_p45 > 0 )
        angulos_coerentes = true;

    bool lado_certo = false; // Verificar se esta do lado certo
    if(angulos_coerentes){ // Se todos estao dentro, checamos o lado da esfera
        if( (p_esf - C).norm() < R )
            lado_certo = true;
    }

    if(angulos_coerentes && lado_certo)
        return true;
    else
        return false;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Vec3b mapColor(Mat im, Vector3f p, Vector3f p5, Vector3f p4, Vector3f p2){
    // Cosseno e seno do angulo entre p, p2(origem) e p3
    float cos_p54 = ( (p - p5).dot(p4 - p5)) / ( (p - p5).norm()*(p4 - p5).norm() );
    float sin_p54 = 1 - pow(cos_p54, 2);
    // Comprimento total no mundo real entre os vertices na horizontal e vertical
    float hor = (p4 - p5).norm(), ver = (p2 - p5).norm();
    // Valores em pixels das coordenadas do ponto respectivo
    int u = int(  float(im.cols)/hor * (p - p5).norm()*cos_p54  );
    int v = int(  float(im.rows)/ver * (p - p5).norm()*sin_p54  );

    // Cor naquele ponto para retornar
    Vec3b cor = im.at<Vec3b>(Point(u, im.rows - v));
    return cor;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "projeta_esfera");
    ros::NodeHandle nh;    
    ros::NodeHandle n_("~");

    string nome_param;
    n_.param("pasta", nome_param, string("Dados_PEPO"));

    // Define pasta a ser vasculhada e arquivo NVM
    char* home;
    home = getenv("HOME");
    std::string pasta = std::string(home)+"/Desktop/"+nome_param+"/";
    cout << pasta << endl;
    std::string arquivo_nvm = pasta + "cameras.nvm";

    /// Le arquivo NVM e carrega tudo nos vetores de variaveis respectivas
    ///
    ifstream nvm(arquivo_nvm);
    int contador_linhas = 1;
    vector<Quaternion<float>> rots;
    vector<Vector3f> Cs;
    vector<std::string> nomes_imagens, linhas;
    std::string linha;
    ROS_INFO("Abrindo e lendo arquivo NVM ...");
    if(nvm.is_open()){
        while(getline(nvm, linha)){
            if(contador_linhas > 3 && linha.size() > 4)
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

    /// Desenha a ESFERA - adicionando pontos
    ///
    PointCloud<PointT>::Ptr esfera (new PointCloud<PointT>);
    float R = 1; // Raio da esfera [m]
    // Angulos para lat e lon, 360 de range para cada, resolucao a definir no step_deg
    float step_deg = 0.2; // [DEGREES]
    int raios_360 = int(360.0/step_deg), raios_180 = raios_360/2; // Quantos raios sairao do centro para formar 360 e 180 graus de um circulo 2D
    bool criar_esfera = true;
    if(criar_esfera){
        ROS_INFO("Criando esfera com resolucao de %.4f graus ...", step_deg);
        esfera->resize(raios_360*raios_180);
        omp_set_dynamic(0);
#pragma omp parallel for num_threads(50)
        for (int a = 0; a < raios_180; a++) {
            for (int o = 0; o < raios_360; o++) {
                PointT ponto;
                float lat, lon;
                lat = DEG2RAD(a*step_deg), lon = DEG2RAD(o*step_deg);
                // Calculo segundo latitude e longitude do ponto para coordenadas - latitude corre em Y !!!
                ponto.y = R*cos(lat);
                ponto.x = R*sin(lat)*cos(-lon);
                ponto.z = R*sin(lat)*sin(-lon);
                ponto.g = ponto.y < 0   ? abs(ponto.y)/R*250 : 0;
                ponto.r = ponto.y > 0   ? abs(ponto.y)/R*250 : 0;
                ponto.b = ponto.x > R/2 ? abs(ponto.x)/R*250 : 0;
                ponto.normal_x = o; ponto.normal_y = raios_180-1 - a; ponto.normal_z = 0;
                // Adiciona ponto na esfera
                esfera->points[a*raios_360 + o] = ponto;
            }
        }
        // Salvando esfera crua
        ROS_INFO("Salvando esfera sem cores com %zu pontos ...", esfera->size());
        savePLYFileASCII<PointT>(pasta+"esfera_raw.ply", *esfera);
    } else {
        ROS_INFO("Carregando esfera sem cores ...");
        loadPLYFile(pasta+"esfera_raw.ply", *esfera);
    }

    /// Para cada imagem
    ///
    for(int i=0; i < nomes_imagens.size(); i++){

        ROS_INFO("Projetando imagem %d ...", i+1);
        // Ler a imagem a ser usada
        Mat image = imread(nomes_imagens[i]);
        if(image.cols < 3){
            ROS_ERROR("Imagem nao foi encontrada, checar NVM ...");
            ros::shutdown();
        }
        // Parametros da camera
        Mat params = (Mat_<double>(1,5) << 0.113092, -0.577590, 0.005000, -0.008206, 0.000000);
        Mat K      = (Mat_<double>(3,3) << foco,  0.0, image.cols/2,
                                            0.0, foco, image.rows/2,
                                            0.0,  0.0,     1.0     );
        // Tirar distorcao da imagem
//        Mat temp;
//        undistort(image, temp, K, params);
//        temp.copyTo(image);
        // Definir o foco em dimensoes fisicas do frustrum
        float F = 2;
        // Calcular a vista da camera pelo Rt inverso - rotacionar para o nosso mundo, com Z para cima
        ROS_INFO("Calculando pose da camera e frustrum ...");
        Matrix4f T = calculateCameraPose(rots[i], Cs[i]);
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
        // Equacao do plano da foto no mundo real
        Vector4f p, p1, p2, p3, p4, p5, pCenter;
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
        Vector4f plano = calculatePlane(p2, p3, p5);

        // Varrer os pontos, se as coordenadas polares estao dentro da area
        ROS_INFO("Varrendo pontos da esfera para reprojetar nos pixels certos ...");
        omp_set_dynamic(0);
        #pragma omp parallel for num_threads(50)
        for(size_t i=0; i < esfera->size(); i++){
            // Calcular intersecao com o plano, se tiver - a principio deve ter no plano infinito, checar se esta no limite do frustrum da camera depois
            Vector3f ponto_esfera(esfera->points[i].x, esfera->points[i].y, esfera->points[i].z);
            Vector3f inters = intersectLinePlane(pCenter.block<3,1>(0, 0), ponto_esfera, plano);
            // Se estiver dentro da camera
            if( isInsideCam(ponto_esfera, inters, p2.block<3,1>(0, 0), p3.block<3,1>(0, 0),
                            p4.block<3,1>(0, 0), p5.block<3,1>(0, 0), pCenter.block<3,1>(0, 0), R) ){
                Vec3b cor = mapColor(image, inters, p5.block<3,1>(0, 0), p4.block<3,1>(0, 0), p2.block<3,1>(0, 0));
                esfera->points[i].r = cor[2]; esfera->points[i].g = cor[1]; esfera->points[i].b = cor[0];
            }
        }

    } // Fim do for imagens

    // Salvar a nuvem da esfera agora colorida
    ROS_INFO("Salvando esfera final colorida ...");
    savePLYFileASCII<PointT>(pasta+"esfera_cor"+std::to_string(R)+".ply", *esfera);

    // Salva imagem circular ao final
    ROS_INFO("Salvando imagem esferica ...");
    Mat imagem_esferica = Mat::zeros( Size(raios_360, raios_180), CV_8UC3 );
    omp_set_dynamic(0);
#pragma omp parallel for num_threads(20)
    for(size_t i = 0; i < esfera->size(); i++){
        // Coordenada da imagem salva na normal
        int u = esfera->points[i].normal_x, v = esfera->points[i].normal_y;
        // Pega as cores da nuvem ali e coloca na imagem final
        Vec3b cor_im;
        cor_im[0] = esfera->points[i].b; cor_im[1] = esfera->points[i].g; cor_im[2] = esfera->points[i].r;
        imagem_esferica.at<Vec3b>(Point(u, v)) = cor_im;
    }
    // Salvando imagem esferica final
    imwrite(pasta+"imagem_esferica"+std::to_string(R)+".png", imagem_esferica);

    ROS_INFO("Processo finalizado.");
    ros::spinOnce();
    return 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
