#include <ros/ros.h>
#include <dirent.h>
#include <errno.h>
#include <ostream>
#include <istream>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../msgs/nuvem.pb.h"
#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/zero_copy_stream.h"

#include <zmq.hpp>
#include <zmq_utils.h>

using namespace NuvemMsgProto;
using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace std;
using namespace zmq;

typedef PointXYZRGB PointT;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "recebe_nuvem");
    ros::NodeHandle nh;
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    // Iniciando o contexto e subscriber para as mensagens na porta TCP correta
    ROS_WARN("LIGANDO CONTEXTO E SUBSCRIBER ZMQ, PORTA 5558");
    void *context = zmq_ctx_new();
    void *receiver = zmq_socket(context, ZMQ_PULL);
    int con = zmq_connect(receiver, "tcp://192.168.0.101:5558");

    // Inicia o publicador de nuvens que usaremos
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/nuvem_atual", 10);

    // Criando mensagem para cada imagem
    zmq_msg_t nuvem_zmq;
    int rc = zmq_msg_init(&nuvem_zmq);
    // Para cada nuvem que chega equanto o ros estiver funcionando
    while(ros::ok()){
        rc = zmq_recvmsg(receiver, &nuvem_zmq, 0);
        vector<uchar> buffer(zmq_msg_size(&nuvem_zmq));
        memcpy(buffer.data(), zmq_msg_data(&nuvem_zmq), zmq_msg_size(&nuvem_zmq));
        string temp_cloud(buffer.begin(), buffer.end());
        // Convertendo a nuvem para protobuf
        Nuvem nuvem_proto;
        nuvem_proto.ParseFromString(temp_cloud);
        // Criando a nuvem em pcl
        PointCloud<PointT>::Ptr cloud (new PointCloud<PointT>);
        cloud->header.frame_id = "map";
        cloud->resize(nuvem_proto.size());
        // Escrever cada ponto no lugar certo
#pragma omp parallel for
        for(int p=0; p<nuvem_proto.size(); p++){
            // Escreve o ponto como veio na mensagem
            PointT ponto;
            ponto.x = nuvem_proto.pontos(p).x(); ponto.y = nuvem_proto.pontos(p).y(); ponto.z = nuvem_proto.pontos(p).z();
            ponto.r = nuvem_proto.pontos(p).r(); ponto.g = nuvem_proto.pontos(p).g(); ponto.b = nuvem_proto.pontos(p).b();
            // Coloca no local na nuvem
            cloud->points[p] = ponto;
        }
        // Enviando por mensagem ROS
        sensor_msgs::PointCloud2 pclmsg;
        pclmsg.header.frame_id = "map";
        pclmsg.header.stamp = ros::Time::now();
        toROSMsg(*cloud, pclmsg);
        pub.publish(pclmsg);

        ros::spinOnce();
    }

    zmq_ctx_term(context);

    return 0;
}
