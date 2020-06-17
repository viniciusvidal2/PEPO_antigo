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
  context_t ctx{1};
  socket_t receiver(ctx, ZMQ_SUB); // Tipo PULL para aguardar a mensagem e nao perder nada enviado
  receiver.connect("tcp://192.168.0.101:5558");

  // Inicia o publicador de nuvens que usaremos
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/nuvem_atual", 10);

  // Para cada nuvem que chega equanto o ros estiver funcionando
  message_t nuvem_zmq;
  while(ros::ok()){
    ROS_INFO("Aguardando nuvem ...");
    receiver.recv(&nuvem_zmq);
    string temp_cloud(static_cast<char *>(nuvem_zmq.data()), nuvem_zmq.size());
    // Convertendo a nuvem para protobuf
    Nuvem nuvem_proto;
    nuvem_proto.ParseFromString(temp_cloud);
    ROS_INFO("Convertendo a nuvem ...");
    // Criando a nuvem em pcl
    PointCloud<PointT>::Ptr cloud (new PointCloud<PointT>);
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
    pclmsg.header.frame_id = "obj";
    pclmsg.header.stamp = ros::Time::now();
    toROSMsg(*cloud, pclmsg);
    pub.publish(pclmsg);

    ros::spinOnce();
  }

  ctx.close();

  return 0;
}
