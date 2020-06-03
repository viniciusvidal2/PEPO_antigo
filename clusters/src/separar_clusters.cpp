/// Includes
///
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <math.h>
#include <sys/stat.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
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

#include "../../libraries/include/clusters.h"
#include "../../libraries/include/processcloud.h"

/// Definicoes e namespaces
///
using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace std;
typedef PointXYZRGB       PointT ;
typedef PointXYZRGBNormal PointTN;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int getdir(string dir, vector<string> &imgs, vector<string> &nuvens)
{
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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Main
///
int main(int argc, char **argv)
{
    // Inicia no
    ros::init(argc, argv, "separar_clusters");
    ros::NodeHandle nh;
    ros::NodeHandle n_("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    // Define ambiente para ver o tratamento
    int ambiente = 1; // 1 para interno, 2 para externo
    n_.param("ambiente", ambiente, 1);

    // Define pasta do projeto todo
    ROS_INFO("Carregando as nuvens de pontos e imagens da pasta ...");
    char* home;
    home = getenv("HOME");
    string nome_pasta;
    n_.param("pasta", nome_pasta, string("Dados_PEPO1"));
    nome_pasta = string(home)+"/Desktop/"+nome_pasta+"/";
    struct stat buffer; // Para checar se ja existem arquivos

    // Checando se ha nuvem final, se ja houver e melhor deletar e faze-la novamente para nao
    // se misturar com as outras
    string nome_final = nome_pasta+"nuvem_clusters_filtrada.ply";
    if(!stat(nome_final.c_str(), &buffer)){
        if(remove(nome_final.c_str()) == 0)
            ROS_INFO("Deletamos nuvem final anterior.");
        else
            ROS_ERROR("Nuvem final anterior nao foi deletada.");
    }

    // Ler pasta e obter as nuvens e imagens, guardar os nomes nos vetores corretos
    vector<string> nomes_nuvens, nomes_imagens;
    getdir(nome_pasta, nomes_imagens, nomes_nuvens);
    ROS_INFO("Temos um total de %d nuvens a processar ...", nomes_nuvens.size());

    // Classes que possuem funcoes de processamento
    Clusters cl;
    ProcessCloud pc;

    //////////////////////////
    /// Para cada nuvem parcial, realizar o procedimento
    /// (a principio nao vejo possibilidade de deixar paralelo aqui, somente durante o procedimento de cada nuvem)
    // Todos os clusters e planos obtidos nessa
    vector<PointCloud<PointTN>> planos_totais, clusters_totais; // Aqui somente cor original por enquanto - menos memoria
    PointCloud<PointTN>::Ptr final (new PointCloud<PointTN>);

    for(int i=0; i < nomes_nuvens.size(); i++){
        // Inicia variaveis
        PointCloud<PointT>::Ptr inicial             (new PointCloud<PointT>);
        PointCloud<PointT>::Ptr filtrada            (new PointCloud<PointT>);
        PointCloud<PointT>::Ptr filtrada_sem_planos (new PointCloud<PointT>);
        vector<PointCloud<PointT >> vetor_planos     , vetor_clusters     ;
        vector<PointCloud<PointTN>> vetor_planos_filt, vetor_clusters_filt;

        ROS_WARN("Comecando a processar nuvem %d de %d ...", i+1, nomes_nuvens.size());

        // Le nuvem atual
        loadPLYFile<PointT>(nome_pasta+nomes_nuvens[i], *inicial);


        /// AQUI SERIA PARA RECALIBRAR A IMAGEM CASO NAO ESTIVESSE ALINHADA - AGORA JA PARECE MELHOR, COMENTADO
//        // Acerta a coloracao da nuvem com a respectiva imagem
//        Mat imagem = imread(nome_pasta+nomes_imagens[i]);
//        Mat params = (Mat_<double>(1,5) << 0.0723, -0.1413, -0.0025 -0.0001, 0.0000);
//        Mat K      = (Mat_<double>(3,3) << 1130,  0.0, imagem.cols/2,
//                      0.0, 1130, imagem.rows/2,
//                      0.0,  0.0,     1.0     );
//        // Tirar distorcao da imagem - fica melhor sim o resultado ou proximo
//        Mat temp;
//        undistort(imagem, temp, K, params);
//        temp.copyTo(imagem);
//        pc.colorCloudWithCalibratedImage(inicial, imagem, 1130, 1130);
//        savePLYFileBinary<PointT>(nome_pasta+"calibrada_"+to_string(i)+".ply", *inicial);

        // Filtra por voxels pra aliviar a nuvem e o processo seguinte
        VoxelGrid<PointT> voxel;
        float lfsz = 0.03; // m
        voxel.setInputCloud(inicial);
        voxel.setLeafSize(lfsz, lfsz, lfsz);
        voxel.filter(*filtrada);

        // Filtra por outliers
        std::string nuvem_covariancia_nome = nome_pasta + "nuvem_filtrada_covariancia"+to_string(i)+".ply";
        if(stat(nuvem_covariancia_nome.c_str(), &buffer) && ambiente == 1){ // Se nao existe o arquivo, calcular somente para interno
            ROS_INFO("Filtrando por outliers ...");
            StatisticalOutlierRemoval<PointT> sor;
            sor.setInputCloud(filtrada);
            sor.setMeanK(10);
            sor.setStddevMulThresh(2);
            sor.setNegative(false);
            sor.filter(*filtrada);

            ROS_INFO("Filtrando ruidos radiais ...");
            pc.filterCloudDepthCovariance(filtrada, 30, 1.1);

            ROS_INFO("Salvando a nuvem filtrada por covariancia ...");
            savePLYFileBinary<PointT>(nuvem_covariancia_nome, *filtrada);
        } else if(ambiente == 1) { // Se o arquivo ja existe, carregar somente
            ROS_INFO("Carregando a nuvem filtrada por covariancia ...");
            loadPLYFile<PointT>(nuvem_covariancia_nome, *filtrada);
        }
        if(ambiente == 2){ // Se ambiente externo, so tirar mesmo alguns ruidos
            ROS_INFO("Ambiente externo, filtrando somente por outliers ...");
            StatisticalOutlierRemoval<PointT> sor;
            sor.setInputCloud(filtrada);
            sor.setMeanK(10);
            sor.setStddevMulThresh(2);
            sor.setNegative(false);
            sor.filter(*filtrada);
        }
        inicial->clear();

        // Extrai um vetor de planos e retorna nuvem sem eles
        ROS_INFO("Obtendo planos na nuvem ...");
        cl.obtainPlanes(filtrada, vetor_planos, filtrada_sem_planos);
        cl.separateClustersByDistance(vetor_planos);
        cl.killSmallClusters(vetor_planos, 1);
        ROS_INFO("Foram obtidos %zu planos apos filtragem.", vetor_planos.size());
        vetor_planos_filt.resize(vetor_planos.size());
        filtrada->clear();

        // Aplicando polinomios sobre os planos
        ROS_INFO("Filtrando por polinomio os planos ...");
        pc.applyPolynomialFilter(vetor_planos, vetor_planos_filt, 1, 0.1);
        vetor_planos.clear();

        // Extrai clusters da nuvem de pontos que restou
        ROS_INFO("Obtendo clusters para o restante da nuvem ...");
        cl.extractClustersRegionGrowingRGB(filtrada_sem_planos, vetor_clusters);
        cl.separateClustersByDistance(vetor_clusters);
        cl.killSmallClusters(vetor_clusters, 1);
        ROS_INFO("Foram obtidos %zu clusters apos filtragem.", vetor_clusters.size());
        vetor_clusters_filt.resize(vetor_clusters.size());
        filtrada_sem_planos->clear();

        // Aplicando polinomio sobre clusters
        ROS_INFO("Filtrando por polinomio os clusters ...");
        pc.applyPolynomialFilter(vetor_clusters, vetor_clusters_filt, 5, 0.15);
        vetor_clusters.clear();

        /// A PRINCIPIO NAO QUERO MAIS COLORIR, SO SEPARAR OS CLUSTERS E SEGUIR COM ELES EM FRENTE
        /// COM A COR ORIGINAL

        //        // Definindo paleta de cores de cada plano e cluster
        //        cl.setColorPallete(vetor_planos.size() + vetor_clusters.size());
        //
        //        // Colore nuvem de pontos cada qual com sua cor selecionada da paleta
        //        ROS_INFO("Colorindo planos ...");
        //        omp_set_dynamic(0);
        //#pragma omp parallel for num_threads(vetor_planos.size())
        //        for(size_t i=0; i < vetor_planos.size(); i++){
        //            PointCloud<PointTN>::Ptr temp (new PointCloud<PointTN>);
        //            *temp = vetor_planos[i];
        //            cl.colorCloud(temp, i);
        //            vetor_planos[i] = *temp;
        //            temp->clear();
        //            *temp = vetor_planos_filt[i];
        //            cl.colorCloud(temp, i);
        //            vetor_planos_filt[i] = *temp;
        //        }
        //        ROS_INFO("Colorindo clusters ...");
        //        omp_set_dynamic(0);
        //#pragma omp parallel for num_threads(vetor_planos.size())
        //        for(size_t i=0; i < vetor_clusters.size(); i++){
        //            PointCloud<PointTN>::Ptr temp (new PointCloud<PointTN>);
        //            *temp = vetor_clusters[i];
        //            cl.colorCloud(temp, vetor_planos.size()+i);
        //            vetor_clusters[i] = *temp;
        //            temp->clear();
        //            *temp = vetor_clusters_filt[i];
        //            cl.colorCloud(temp, vetor_planos.size()+i);
        //            vetor_clusters_filt[i] = *temp;
        //        }

        // Adiciona nos vetores de clusters e planos de toda a acumulacao
        ROS_INFO("Registrando os %d planos e %d clusters obtidos ...", vetor_planos_filt.size(), vetor_clusters_filt.size());
        for(size_t i=0; i < vetor_planos_filt.size(); i++)
            planos_totais.push_back(vetor_planos_filt[i]);
        for(size_t i=0; i < vetor_clusters_filt.size(); i++)
            clusters_totais.push_back(vetor_clusters_filt[i]);
        vetor_planos_filt.clear(); vetor_clusters_filt.clear();
    }

    // Acumula nuvem final
    ROS_INFO("Acumulando clusters apos processo ...");
    for(size_t i=0; i < planos_totais.size(); i++)
        *final += planos_totais[i];
    for(size_t i=0; i < clusters_totais.size(); i++)
        *final += clusters_totais[i];

    // Salva cada nuvem de clusters na pasta certa
    ROS_INFO("Salvando os clusters na subpasta Clusters ...");
    string pasta_cluters = nome_pasta + "Clusters";
    system(("rm -r "+pasta_cluters).c_str());
    if(stat(pasta_cluters.c_str(), &buffer))
        mkdir(pasta_cluters.c_str(), 0777);
    for(size_t i=0; i < planos_totais.size()  ; i++)
        savePLYFileBinary<PointTN>(pasta_cluters+"/p_"+std::to_string(i+1)+".ply", planos_totais[i]);
    for(size_t i=0; i < clusters_totais.size(); i++)
        savePLYFileBinary<PointTN>(pasta_cluters+"/o_"+std::to_string(i+1)+".ply", clusters_totais[i]);

    // Passa um ultimo filtro e salva nuvem final
    ROS_INFO("Filtrando e salvando nuvem final ...");
    StatisticalOutlierRemoval<PointTN> sor;
    sor.setInputCloud(final);
    sor.setMeanK(30);
    sor.setStddevMulThresh(2);
    sor.setNegative(false);
    sor.filter(*final);
    savePLYFileBinary<PointTN>(nome_final, *final);

    planos_totais.clear(); clusters_totais.clear();
    final->clear();

    ROS_INFO("Processo finalizado.");

    return 0;
}
