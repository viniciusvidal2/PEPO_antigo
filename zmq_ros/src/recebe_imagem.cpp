#include <ros/ros.h>
#include <dirent.h>
#include <errno.h>
#include <ostream>
#include <istream>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../msgs/imagem.pb.h"
#include "../msgs/nuvem.pb.h"
#include "../msgs/arquivos.pb.h"
#include "../msgs/nvm.pb.h"
#include "google/protobuf/io/coded_stream.h"
#include "google/protobuf/io/zero_copy_stream.h"

#include <zmq.hpp>
#include <zmq_utils.h>

using namespace ImagemMsgProto;
using namespace cv;
using namespace std;
using namespace zmq;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "recebe_imagem");
  ros::NodeHandle nh;

  // Iniciando o contexto e subscriber para as mensagens na porta TCP correta
  ROS_WARN("LIGANDO CONTEXTO E SUBSCRIBER ZMQ, PORTA 5557");
  context_t ctx{1};
  socket_t receiver(ctx, ZMQ_PULL); // Tipo PULL para aguardar a mensagem e nao perder nada enviado
  receiver.connect("tcp://192.168.0.101:5557");

  // Publisher para imagem
  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/imagem", 10);

  // Criando mensagem para cada imagem
  message_t imagem_zmq;
  // Para cada imagem
  while(ros::ok()){
    ROS_INFO("Recebendo imagem...");
    receiver.recv(&imagem_zmq);
    string temp_img(static_cast<char *>(imagem_zmq.data()), imagem_zmq.size());
    Imagem img_proto;
    img_proto.ParseFromString(temp_img);
    ROS_INFO("Convertendo a imagem ...");
    // Criando a matriz de opencv
    Mat im(img_proto.height(), img_proto.width(), CV_8UC3, Scalar(0, 0, 0));
    // Escrever cada pixel no lugar certo
    #pragma omp parallel for
    for(int p=0; p<img_proto.pixels_size(); p++){
      // Pega cor do pixel
      Vec3b cor;
      cor[0] = img_proto.pixels(p).b(); cor[1] = img_proto.pixels(p).g(); cor[2] = img_proto.pixels(p).r();
      // Pega posicao do pixel
      int u = img_proto.pixels(p).u(), v = img_proto.pixels(p).v();
      // Anota isso no lugar certo ali na imagem
      im.at<Vec3b>(Point(u, v)) = cor;
    }
    // Publicando
    cv_bridge::CvImagePtr im_ptr;
    im.copyTo(im_ptr->image);
    pub.publish(im_ptr->toImageMsg());

    ros::spinOnce();
  }

  ctx.close();

  return 0;
}
