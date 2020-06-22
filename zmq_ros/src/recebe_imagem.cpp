#include <ros/ros.h>
#include <dirent.h>
#include <errno.h>
#include <ostream>
#include <istream>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>

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
    void *context = zmq_ctx_new();
    void *receiver = zmq_socket(context, ZMQ_PULL);
    int con = zmq_connect(receiver, "tcp://192.168.0.101:5557");

    // Publisher para imagem
    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/imagem", 10);

    // Criando mensagem para cada imagem
    zmq_msg_t imagem_zmq;
    int rc = zmq_msg_init(&imagem_zmq);
    // Para cada imagem
    while(ros::ok()){
        rc = zmq_recvmsg(receiver, &imagem_zmq, 0);
        vector<uchar> buffer(zmq_msg_size(&imagem_zmq));

        memcpy(buffer.data(), zmq_msg_data(&imagem_zmq), zmq_msg_size(&imagem_zmq));
        Mat im = imdecode(Mat(buffer), CV_LOAD_IMAGE_COLOR);

        // Publicando
        std_msgs::Header header;
        sensor_msgs::Image im_msg;
        header.stamp = ros::Time::now(); // time
        cv_bridge::CvImage im_bridge;
        im_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, im);
        im_bridge.toImageMsg(im_msg);
        pub.publish(im_msg);

        ros::spinOnce();
    }

    zmq_disconnect(receiver, "tcp://192.168.0.101:5557");
    zmq_ctx_term(context);

    return 0;
}