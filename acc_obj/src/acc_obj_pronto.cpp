#include <ros/ros.h>
#include <iostream>
#include <string>
#include <math.h>
#include <sys/stat.h>
#include <ostream>
#include <iterator>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>

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
    sort(imgs.begin()  , imgs.end()  );
    sort(nuvens.begin(), nuvens.end());

    closedir(dp);

    return 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "acc_obj_pronto");
    ros::NodeHandle nh;
    ros::NodeHandle n_("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    // Inicia publicador da nuvem parcial e do objeto
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/acc_obj/obj", 10);
    sensor_msgs::PointCloud2 msg;

    // Ler pasta com os dados
    string nome_param;
    n_.param("pasta", nome_param, string("Dados_PEPO"));
    char* home;
    home = getenv("HOME");
    string pasta = string(home)+"/Desktop/"+nome_param+"/";

    // Objetos de classe usados no processo
    ProcessCloud pc(pasta);
    RegisterObjectOptm roo;

    // Se ja houver objeto salvo, deletar ele nesse ponto e comecar outro
    struct stat buffer;
    string nome_objeto_final = pasta + "objeto_final.ply";
    if(!stat(nome_objeto_final.c_str(), &buffer)){
        if(remove(nome_objeto_final.c_str()) == 0)
            ROS_INFO("Deletamos nuvem final anterior.");
        else
            ROS_ERROR("Nuvem final anterior nao foi deletada.");
    }

    // Ler arquivo NVM e pegar somente as nuvens que interessam
    vector<string> nomes_nuvens, nomes_imagens;
    vector<Matrix4f> poses;
    float f;
    roo.readNVM(pasta, "cameras", nomes_nuvens, nomes_imagens, poses, f);
    ROS_INFO("Temos um total de %zu nuvens a processar ...", nomes_nuvens.size());

    // Nuvens
    PointCloud<PointTN>::Ptr cnow (new PointCloud<PointTN>), cobj (new PointCloud<PointTN>);
    cnow->header.frame_id = "map";
    cobj->header.frame_id = "map";
    // Imagem de agora
    Mat imnow;
    // Pose da nuvem
    Matrix4f Tnow;

    /// Vai processando as nuvens como estao vindo
    ///
    for(size_t i=0; i<nomes_nuvens.size(); i++){
        ROS_INFO("Acumulando nuvem de objeto %zu ...", i+1);
        // Le dados, filtra e guarda na memoria
        ROS_INFO("Pre-processando nuvem %zu ...", i+1);
        roo.readCloudAndPreProcess(pasta+nomes_nuvens[i], cnow);
        pc.calculateNormals(cnow);
        pc.filterCloudDepthCovariance(cnow, 20, 1);
        imnow = imread(pasta+nomes_imagens[i]);

        // Projetar nuvem e salvar nuvem auxiliar de pixels
        ROS_INFO("Projetando para otimizar cores ...");
        if(imnow.cols > 1000) // Aqui se estivermos com a resolucao HD, senao pode usar o do NVM
            f = 1130;
        pc.colorCloudWithCalibratedImage(cnow, imnow, f, f);

        // Levar aonde paramos no processo de reconstrucao do objeto
        transformPointCloudWithNormals<PointTN>(*cnow, *cnow, poses[i]);

        // Soma a nuvem transformada e poe no lugar certo somente pontos "novos"
        ROS_INFO("Registrando nuvem %d no objeto final ...", i+1);
        PointCloud<PointTN>::Ptr cnowtemp (new PointCloud<PointTN>);
        *cnowtemp = *cnow;
        if(cobj->size() > 0){ // Adicionando a partir da segunda
            roo.searchNeighborsKdTree(cnowtemp, cobj, 0.8);
            *cobj += *cnowtemp;
        } else { // Se primeira vez, inicia a nuvem
            *cobj = *cnow;
        }

        // Publicando o resultado atual para visualizacao
        toROSMsg(*cobj, msg);
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        pub.publish(msg);
    }

    // Filtrando novamente objeto final
    ROS_INFO("Filtrando e salvando nuvem de objeto ...");
    StatisticalOutlierRemoval<PointTN> sor;
    sor.setMeanK(10);
    sor.setStddevMulThresh(2);
    sor.setInputCloud(cobj);
    sor.filter(*cobj);

    // Publicando o resultado atual para visualizacao
    toROSMsg(*cobj, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    pub.publish(msg);

    // Salvando a nuvem final do objeto
    savePLYFileBinary<PointTN>(nome_objeto_final, *cobj);
    ROS_INFO("Processo finalizado.");

    // Manter a publicacao enquanto o no nao morre para visualizacao
    ros::Rate r(1);
    while(ros::ok()){
        msg.header.stamp = ros::Time::now();
        pub.publish(msg);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
