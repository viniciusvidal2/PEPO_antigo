#include <ros/ros.h>
#include <iostream>
#include <string>
#include <math.h>
#include <sys/stat.h>
#include <ostream>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "../../libraries/include/processcloud.h"
#include "../../libraries/include/registerobjectoptm.h"

using namespace pcl;
using namespace pcl::io;
using namespace Eigen;
using namespace std;
using namespace cv;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int getdir(string dir, vector<string> &imgs, vector<string> &nuvens){
    // Abrindo a pasta raiz para contar os arquivos de imagem e nuvem que serao lidos e enviados
    DIR *dp;
    struct dirent *dirp;
    string nome_temp;
    if((dp  = opendir(dir.c_str())) == NULL)
        ROS_ERROR("Nao foi possivel abrir o diretorio");

    while ((dirp = readdir(dp)) != NULL) {
        nome_temp = string(dirp->d_name);
        // Todas as imagens na pasta
        if(nome_temp.substr(nome_temp.find_last_of(".")+1) == "png")
            imgs.push_back(nome_temp);
        // Todas as nuvens na pasta
        if(nome_temp.substr(nome_temp.find_last_of(".")+1) == "ply")
            nuvens.push_back(nome_temp);
    }
    closedir(dp);

    return 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "acc_obj");
    ros::NodeHandle nh;
    ros::NodeHandle n_("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    // Objetos de classe usados no processo
    ProcessCloud pc;
    RegisterObjectOptm roo;

    // Ler pasta com os dados
    string nome_param;
    n_.param("pasta", nome_param, string("Dados_PEPO"));
    char* home;
    home = getenv("HOME");
    std::string pasta = std::string(home)+"/Desktop/"+nome_param+"/";

    // Se ja houver objeto salvo, deletar ele nesse ponto e comecar outro
    struct stat buffer;
    string nome_objeto_final = pasta + "objeto_final.ply";
    if(!stat(nome_objeto_final.c_str(), &buffer)){
        if(remove(nome_objeto_final.c_str()) == 0)
            ROS_INFO("Deletamos nuvem final anterior.");
        else
            ROS_ERROR("Nuvem final anterior nao foi deletada.");
    }

    // Vasculhar a pasta em busca de todas as nuvens e imagens
    vector<string> nomes_nuvens, nomes_imagens;
    getdir(pasta, nomes_imagens, nomes_nuvens);
    ROS_INFO("Temos um total de %zu nuvens a processar ...", nomes_nuvens.size());

    /// Inicia as variaveis - aqui comeca a pancadaria
    ///
    // Nuvem de agora, nuvem referencia, nuvem do objeto acumulado,
    // nuvem de pixels auxiliar referencia, nuvem de pixels auxiliar atual
    PointCloud<PointTN>::Ptr cnow (new PointCloud<PointTN>), cref (new PointCloud<PointTN>), cobj (new PointCloud<PointTN>);
    PointCloud<PointTN>::Ptr cpixref (new PointCloud<PointTN>), cpixnow (new PointCloud<PointTN>);
    // Imagem de agora e da referencia
    Mat imnow, imref;
    // Pose da nuvem agora e na referencia
    Matrix4f Tnow, Tref;
    // Vetor de offset entre centro do laser e da camera - desenho solid, e foco
    Vector3f t_off_lc(0.01, 0.0448, 0.023);
    float f = 1130;

    /// Inicia primeiro a nuvem referencia e outros dados com o primeiro dado lido
    ///
    if(cref->empty()){
        ROS_INFO("Iniciando os dados de referencia ...");
        roo.readCloudAndPreProcess(pasta+nomes_nuvens[0], cref);
        imref = imread(pasta+nomes_imagens[0]);
        cout << nomes_nuvens[0] << " " << nomes_imagens[0] << endl;

        // Projetar a nuvem na imagem de forma calibrada, otimizar a cor e salvar em nuvem de
        // pontos auxiliar os pixels correspondentes a cada ponto
        roo.projectCloudAndAnotatePixels(cref, imref, cpixref, f, t_off_lc);

        *cobj = *cref;

        Tref = Matrix4f::Identity();
        Tnow = Tref;
    }

    /// A partir da segunda nuvem e imagem, processo de match e anotacao
    ///
    for(int i=1; i<nomes_nuvens.size(); i++){
        ROS_INFO("Acumulando nuvem de objeto %d ...", i+1);
        // Le dados, filtra e guarda na memoria
        ROS_INFO("Pre-processando nuvem %d ...", i+1);
        roo.readCloudAndPreProcess(pasta+nomes_nuvens[i], cnow);
        imnow = imread(pasta+nomes_imagens[i]);

        // Projetar nuvem e salvar nuvem auxiliar de pixels        
        ROS_INFO("Projetando para otimizar cores e anotar os pixels ...");
        roo.projectCloudAndAnotatePixels(cnow, imnow, cpixnow, f, t_off_lc);

//        // Transforma a nuvem para mais perto da nuvem de referencia com a transformacao de referencia
//        transformPointCloudWithNormals<PointTN>(*cnow, *cnow, Tref);

        // Calcular features e matches e retornar os N melhores e seus pontos em 3D
        // correspondentes a partir da nuvem auxiliar de pixels
        ROS_INFO("Match de features 2D e obtendo correspondencias em 3D ...");
        vector<Point2d> matches3Dindices;
        roo.matchFeaturesAndFind3DPoints(imref, imnow, cpixref, cpixnow, 6, matches3Dindices);
        ROS_INFO("Foram obtidas %zu correspondencias 3D.", matches3Dindices.size());

        // Rodar a otimizacao da transformada por minimos quadrados das coordenadas desacopladas
        ROS_INFO("Otimizando a transformacao relativa das nuvens por minimos quadrados ...");
        Tnow = roo.optmizeTransformLeastSquares(cref, cnow, matches3Dindices);

        // Transforma a nuvem atual com a transformacao encontrada
        transformPointCloudWithNormals<PointTN>(*cnow, *cnow, Tnow);

        // Refina a transformacao por ICP com poucas iteracoes
//        ROS_INFO("Refinando registro por ICP ...");
//        Matrix4f Ticp = roo.icp(cref, cnow, 100);
//        Tnow = Tnow*Ticp;

        // Soma a nuvem transformada e poe no lugar certo
        ROS_INFO("Registrando nuvem atual no objeto final ...");
        *cobj += *cnow;

        // Atualiza as referencias e parte para a proxima aquisicao
        *cref = *cnow;
        *cpixref = *cpixnow;
        Tref = Tnow;
        imref = imnow;
    }

    // Salvando a nuvem final do objeto
    savePLYFileBinary(nome_objeto_final, *cobj);

    ros::spinOnce();
    return 0;
}
