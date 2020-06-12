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

#include "../../libraries/include/processcloud.h"

/// Definicoes e namespaces
///
using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace std;
using namespace Eigen;
typedef PointXYZRGBNormal PointTN;
typedef PointXYZRGB       PointT;

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
int main(int argc, char **argv)
{
    ros::init(argc, argv, "acc_cloud");
    ros::NodeHandle nh;
    ros::NodeHandle n_("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    string nome_param;
    n_.param("pasta", nome_param, string("Dados_PEPO1"));

    // Define pasta a ser vasculhada e arquivo NVM
    char* home;
    home = getenv("HOME");
    std::string pasta = std::string(home)+"/Desktop/"+nome_param+"/";

    // Objeto da classe de projecao de nuvens
    ProcessCloud pc(pasta);

    /// Ler todas as nuvens, somar e salvar
    ///
    struct stat buffer;
    string nome_acumulada = pasta + "acumulada_hd.ply";
    if(stat(nome_acumulada.c_str(), &buffer)){
        ROS_INFO("Lendo e salvando as nuvens ...");
        vector<string> nomes_nuvens;
        getdir(pasta, nomes_nuvens);
        PointCloud<PointT>::Ptr acc   (new PointCloud<PointT>);
        PointCloud<PointTN>::Ptr accn (new PointCloud<PointTN>);
        vector<PointCloud<PointT>> acc_vec(nomes_nuvens.size());
        omp_set_dynamic(0);
#pragma omp parallel for num_threads(15)
        for(int i=0; i < nomes_nuvens.size(); i++){
            PointCloud<PointT>::Ptr temp (new PointCloud<PointT>);
            ROS_INFO("Processando nuvem %d ...", i+1);
            loadPLYFile<PointT>(pasta+nomes_nuvens[i], *temp);
            StatisticalOutlierRemoval<PointT> sor;
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
        ROS_INFO("Salvando nuvem final ...");
        savePLYFileBinary<PointTN>(nome_acumulada, *accn);
        acc->clear(); accn->clear();

        ROS_INFO("Fim do processo.");
    }

    ros::spinOnce();
    return 0;
}
