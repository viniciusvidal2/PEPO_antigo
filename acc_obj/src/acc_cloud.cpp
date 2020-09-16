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
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/sample_consensus/ransac.h>

#include <nav_msgs/Odometry.h>

#include "../../libraries/include/processcloud.h"
#include "../../libraries/include/registerobjectoptm.h"

/// Definicoes e namespaces
///
using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace std;
using namespace Eigen;
typedef PointXYZRGBNormal PointTN;
typedef PointXYZRGB       PointT;

// Inicia variaveis do motor PAN
double raw_min_pan = 35, raw_max_pan = 4077;
double deg_min_pan =  3, deg_max_pan =  358;
double deg_raw_pan, raw_deg_pan; // Definidas no main
double dentro = 3; // Raio de seguranca que estamos dentro ja [RAW] (pelas contas aproximadas, 1 RAW = 0.08 degrees)
// Inicia variaveis do motor TILT - horizontal da offset para ser o zero
double raw_min_tilt = 2595, raw_hor_tilt = 2280, raw_max_tilt = 1595;
double deg_min_tilt =   28, deg_hor_tilt =    0, deg_max_tilt =  -60.9;
double deg_raw_tilt, raw_deg_tilt;
vector<int> posicoes_pan, posicoes_tilt; // Posicoes a serem vasculhadas pelos motores

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
int deg2raw(double deg, std::string motor){
    if(motor == "pan")
        return int((deg - deg_min_pan )*raw_deg_pan  + raw_min_pan);
    else
        return int((deg - deg_min_tilt)*raw_deg_tilt + raw_min_tilt);
}
double raw2deg(int raw, std::string motor){
    if(motor == "pan")
        return (double(raw) - raw_min_pan )*deg_raw_pan  + deg_min_pan;
    else
        return (double(raw) - raw_min_tilt)*deg_raw_tilt + deg_min_tilt - deg_hor_tilt;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
    ros::init(argc, argv, "acc_cloud");
    ros::NodeHandle nh;
    ros::NodeHandle n_("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    string nome_param;
    n_.param("pasta", nome_param, string("patio1"));

    // Define pasta a ser vasculhada e arquivo NVM
    char* home;
    home = getenv("HOME");
    std::string pasta = std::string(home)+"/Desktop/"+nome_param+"/";

    // Definindo as taxas raw - deg dos servos
    deg_raw_pan  = (deg_max_pan  - deg_min_pan ) / (raw_max_pan  - raw_min_pan ); raw_deg_pan  = 1/deg_raw_pan;
    deg_raw_tilt = (deg_max_tilt - deg_min_tilt) / (raw_max_tilt - raw_min_tilt); raw_deg_tilt = 1/deg_raw_tilt;

    // Preenchendo os vetores de posicao a ser escaneadas
    int step = 20; // [degrees]
    int vistas_tilt = abs(int(deg_max_tilt - deg_min_tilt))/step + 1;
    vector<double> tilts(vistas_tilt);
    for(int j=0; j < vistas_tilt; j++)
        tilts[j] = deg2raw(deg_min_tilt - double(j*step), "tilt");
    int vistas_pan = int(deg_max_pan - deg_min_pan)/step + 1; // Vistas na horizontal
    posicoes_pan.resize(vistas_pan * vistas_tilt);
    posicoes_tilt.resize(posicoes_pan.size());
    omp_set_dynamic(0);
    #pragma omp parallel for num_threads(tilts.size())
    for(int j=0; j < vistas_tilt; j++){
        for(int i=0; i < vistas_pan; i++){
            if(remainder(j, 2) == 0) // Define assim um vai e volta no scanner, ao inves de voltar ao inicio
                posicoes_pan[i + j*vistas_pan] = deg2raw(deg_min_pan + double(i*step), "pan");
            else
                posicoes_pan[i + j*vistas_pan] = deg2raw(deg_max_pan - double(i*step), "pan");
            // Posicoes tilt repetidas vezes para cada volta completa de pan
            posicoes_tilt[i + j*vistas_pan] = int(tilts[j]);
        }
    }

    // Objeto da classe de projecao de nuvens
    ProcessCloud pc(pasta);
    RegisterObjectOptm roo;

    // Publicador de odometria do NVM
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("odometria_nvm", 10);
    nav_msgs::Odometry msg;

    /// Ler todas as nuvens, somar e salvar
    ///
    struct stat buffer;
    string nome_acumulada = pasta + "acumulada_hd.ply";
    ROS_INFO("Lendo e salvando as nuvens ...");
    vector<string> nomes_nuvens, nomes_imagens, nomes_nuvens_dumb, nomes_imagens_dumb;
    vector<Matrix4f> poses, poses_certo;
    float f;
    roo.readNVM(pasta, "cameras", nomes_nuvens, nomes_imagens, poses, f);
    roo.readNVM(pasta, "cameras_acertado", nomes_nuvens_dumb, nomes_imagens_dumb, poses_certo, f);
    PointCloud<PointT >::Ptr acc  (new PointCloud<PointT >);
    PointCloud<PointTN>::Ptr accn (new PointCloud<PointTN>);
    vector<PointCloud<PointT>> acc_vec(nomes_nuvens.size());    
    StatisticalOutlierRemoval<PointT> sor;
    for(int i=0; i < nomes_nuvens.size(); i++){
        PointCloud<PointT>::Ptr temp (new PointCloud<PointT>);
        ROS_INFO("Processando nuvem %d ...", i+1);
        loadPLYFile<PointT>(pasta+nomes_nuvens[i], *temp);
        VoxelGrid<PointT> voxel;
        float lfsz = 0.02;
        voxel.setInputCloud(temp);
        voxel.setLeafSize(lfsz, lfsz, lfsz);
//        voxel.filter(*temp);
        sor.setInputCloud(temp);
        sor.setMeanK(20);
        sor.setStddevMulThresh(2);
//        sor.filter(*temp);
//        // Trazendo de volta ao frame da camera
//        transformPointCloud<PointT>(*temp, *temp, poses[i].inverse());
        // Trazendo a nuvem de volta ao lugar de onde foi capturada
        Matrix4f solaser = Matrix4f::Identity();
        Matrix3f r = poses[i].block<3,3>(0, 0);
        Vector3f off{0, 0, 0.056};
        off = r*off;
        solaser.block<3,1>(0, 3) = off;
        solaser.block<3,3>(0, 0) = r;
        transformPointCloud<PointT>(*temp, *temp, solaser.inverse());
        // Projetando na camera com uma calibracao melhor
        Mat im = imread(pasta+nomes_imagens[i]);
        pc.colorCloudWithCalibratedImage(temp, im, 1130, 1130);
        // Filtro de profundidade para nao pegarmos muito fundo
        PassThrough<PointT> pass;
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0, 15); // Z metros de profundidade
        pass.setInputCloud(temp);
//        pass.filter(*temp);
        // Transformar nuvem de acordo com a posicao dos servos, e pegar pose da camera em consequencia
        float p = raw2deg(posicoes_pan[i], "pan"), t = raw2deg(posicoes_tilt[i], "tilt");
        Vector3f C;
        Quaternion<float> qmsg;
        pc.transformCloudAndCamServoAngles(temp, p, t, C, qmsg);
        // Mensagem para verificar a odometria
//        Quaternion<float>qmsg(solaser.block<3,3>(0, 0).inverse());
        msg.pose.pose.orientation.w = qmsg.w();
        msg.pose.pose.orientation.x = qmsg.x();
        msg.pose.pose.orientation.y = qmsg.y();
        msg.pose.pose.orientation.z = qmsg.z();
        msg.pose.pose.position.x = C(0);
        msg.pose.pose.position.y = C(1);
        msg.pose.pose.position.z = C(2);
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        pub.publish(msg);
        // Salvando nova nuvem com cores no lugar certo
        savePLYFileBinary(pasta+nomes_nuvens[i], *temp);

        // Se primeira nuvem
        if(acc->size() < 10){
            *acc = *temp;
        } else {
//            ROS_INFO("Aproximando nuvem %d por ransac ...", i+1);
//            SampleConsensusModelRegistration<PointT>::Ptr sac_model(new SampleConsensusModelRegistration<PointT>(temp));
//            sac_model->setInputTarget(acc);
//            RandomSampleConsensus<PointT> rsac(sac_model);
//            rsac.setDistanceThreshold(0.1);
//            Eigen::VectorXf coeffs;
//            rsac.computeModel(1);
//            rsac.getModelCoefficients(coeffs);
//            assert(coeffs.size() == 16);
//            Matrix4f Tresult = Eigen::Map<Matrix4f>(coeffs.data(),4,4);
//            transformPointCloud<PointT>(*temp, *temp, Tresult);
//            ROS_INFO("Aproximando nuvem %d por ICP ...", i+1);
//            PointCloud<PointT>::Ptr acc2 (new PointCloud<PointT>);
//            PointCloud<PointT>::Ptr temp2(new PointCloud<PointT>);
//            voxel.setLeafSize(0.10, 0.10, 0.10);
//            voxel.setInputCloud(acc);
//            voxel.filter(*acc2);
//            voxel.setInputCloud(temp);
//            voxel.filter(*temp2);
//            *acc2 = *acc;
//            *temp2 = *temp;
//            // Criando o otimizador de ICP comum
//            GeneralizedIterativeClosestPoint<PointT, PointT> icp;
//            Matrix4f Ticp;
//            icp.setUseReciprocalCorrespondences(false);
//            icp.setInputTarget(acc2);
//            icp.setInputSource(temp2);
////            icp.setRANSACIterations(20);
//            icp.setMaximumIterations(10); // Chute inicial bom 10-100
//            icp.setTransformationEpsilon(1*1e-9);
//            icp.setEuclideanFitnessEpsilon(1*1e-10);
//            icp.setMaxCorrespondenceDistance(lfsz*3);
//            // Alinhando
//            PointCloud<PointT> dummy;
//            icp.align(dummy, Matrix4f::Identity());
//            // Obtendo a transformacao otimizada e aplicando
//            if(icp.hasConverged())
//                Ticp = icp.getFinalTransformation();
//            else
//                Ticp = Matrix4f::Identity();
//            transformPointCloud<PointT>(*temp, *temp, Ticp);
            // Iniciar kdtree de busca
//            ROS_INFO("Retirando pontos repetidos da nuvem %d ...", i+1);
//            KdTreeFLANN<PointT> kdtree;
//            kdtree.setInputCloud(acc);
//            vector<int> pointIdxRadiusSearch;
//            vector<float> pointRadiusSquaredDistance;
//            float radius = 0.05;
//            // Nuvem de pontos de indices bons
//            PointIndices::Ptr indices (new PointIndices);
//            float average_neighbors = 0, sumn = 0;
//            // Retirando indices NaN se existirem
//            vector<int> indicesNaN;
//            removeNaNFromPointCloud(*temp, *temp, indicesNaN);
//            // Achando quantidade media de vizinhos naquele raio
//            for(size_t i=0; i<temp->size(); i++)
//                sumn += kdtree.radiusSearch(temp->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
//            average_neighbors = sumn/float(temp->size());
//            // Para cada ponto, se ja houver vizinhos, nao seguir
//            for(size_t i=0; i<temp->size(); i++){
//                if(kdtree.radiusSearch(temp->points[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) <= average_neighbors)
//                    indices->indices.push_back(i);
//            }
//            // Filtrar na nuvem now so os indices que estao sem vizinhos na obj
//            ExtractIndices<PointT> extract;
//            extract.setInputCloud(temp);
//            extract.setIndices(indices);
//            extract.setNegative(false);
//            extract.filter(*temp);

            // Soma tudo na acumulada
            ROS_INFO("Acumulando nuvem %d no space ...", i+1);
//            *acc += *temp;
        }
    }
    sor.setInputCloud(acc);
    sor.filter(*acc);
    ROS_INFO("Calculando a normal na nuvem final ...");
    pc.calculateNormals(acc, accn);
    ROS_INFO("Salvando nuvem final ...");
    savePLYFileBinary<PointTN>(nome_acumulada, *accn);
    acc->clear(); accn->clear();

    ROS_INFO("Fim do processo.");

    ros::spinOnce();
    return 0;
}
