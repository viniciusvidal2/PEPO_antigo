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

#include "../msgs/imagem.pb.h"
#include "../msgs/nuvem.pb.h"
#include "../msgs/arquivos.pb.h"
#include "../msgs/nvm.pb.h"

#include <zmq.hpp>
#include <zmq_utils.h>

using namespace ArquivosMsgProto;
using namespace ImagemMsgProto;
using namespace NuvemMsgProto;
using namespace NVMMsgProto;
using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace std;
using namespace zmq;

typedef PointXYZRGB PointT;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "recebe_dados_zmq");
  ros::NodeHandle nh;
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  /////////////////////////////////////////////
  /// Criando pasta no Desktop com o nome
  /// desejado para colocar os arquivos
  ///

  // Diretorio raiz
  char* home;
  home = getenv("HOME");
  string root = string(home)+"/Desktop/";

  struct stat buffer;
  std::string pasta = root+"Dados_B9_recebidos/";
  system(("rm -r "+pasta).c_str());
  if(stat(pasta.c_str(), &buffer))
    mkdir(pasta.c_str(), 0777);

  ROS_INFO("Pasta de recebimento criada e pronta para receber arquivos. Iniciando comunicacao ...");

  /////////////////////////////////////////////
  /// Ligar Subscriber ZMQ do tipo PULL e
  /// receber todas as mensagens antes de
  /// processar
  ///

  // Iniciando o contexto e subscriber para as mensagens na porta TCP correta
  ROS_WARN("LIGANDO CONTEXTO E SUBSCRIBER ZMQ, PORTA 5558");
  context_t ctx{1};
  socket_t receiver(ctx, ZMQ_PULL); // Tipo PULL para aguardar a mensagem e nao perder nada enviado
  receiver.connect("tcp://localhost:5558");

  /// Recebendo cabecalho com quantidade de imagens e nuvens que virao
  message_t cabecalho_zmq;
  ROS_INFO("Recebendo cabecalho sobre os dados que virao ...");
  receiver.recv(&cabecalho_zmq);
  ROS_INFO("Cabecalho recebido. Recebendo dados ...");
  string buffer_cabecalho(static_cast<char *>(cabecalho_zmq.data()), cabecalho_zmq.size());
  Arquivos cabecalho_proto;
  cabecalho_proto.ParseFromString(buffer_cabecalho);
  cabecalho_proto.DebugString(); // Mostrar quantas imagens e nuvens virao

  /// Recebendo imagens
  // Criando mensagem e vetor de buffers para cada imagem
  message_t imagem_zmq;
  vector<string> buffer_imagens(cabecalho_proto.imagens());
  // Para cada imagem
  for(int i=0; i < cabecalho_proto.imagens(); i++){
    ROS_INFO("Recebendo imagem %d de %d ...", i+1, cabecalho_proto.imagens());
    receiver.recv(&imagem_zmq);
    string temp(static_cast<char *>(imagem_zmq.data()), imagem_zmq.size());
    buffer_imagens[i] = temp;
  }

  /// Recebendo nuvens
  // Criando mensagem e vetor de buffers para cada nuvem
  message_t nuvem_zmq;
  vector<string> buffer_nuvens(cabecalho_proto.nuvens());
  // Para cada imagem
  for(int i=0; i < cabecalho_proto.nuvens(); i++){
    ROS_INFO("Recebendo nuvem %d de %d ...", i+1, cabecalho_proto.nuvens());
    receiver.recv(&nuvem_zmq);
    string temp(static_cast<char *>(nuvem_zmq.data()), nuvem_zmq.size());
    buffer_nuvens[i] = temp;
  }

  /// Recebendo arquivo NVM
  message_t nvm_zmq;
  ROS_INFO("Recebendo arquivo NVM ...");
  receiver.recv(&nvm_zmq);
  string buffer_nvm(static_cast<char *>(nvm_zmq.data()), nvm_zmq.size());

  // Finalizando o contexto ZMQ
  ROS_INFO("Tudo foi recebido. Processando mensagens ...");
  receiver.close();
//  ctx.close();

  /////////////////////////////////////////////
  /// Converter mensagens protobuf em dados
  ///

  // Para cada imagem
  ROS_INFO("Convertendo e salvando as %d imagens agora ...", cabecalho_proto.imagens());
  omp_set_dynamic(0);
  #pragma omp parallel for num_threads(int(cabecalho_proto.imagens()/2))
  for(int i=0; i < cabecalho_proto.imagens(); i++){
    // Convertendo a imagem para protobuf
    Imagem img_proto;
    img_proto.ParseFromString(buffer_imagens[i]);
    ROS_INFO("Convertendo a imagem %d de %d, com nome %s ...", i+1, cabecalho_proto.imagens(), img_proto.name().c_str());
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
    // Salvar a imagem como nome especificado
    imwrite(pasta+img_proto.name(), im);
  }

  // Para cada nuvem
  ROS_INFO("Convertendo e salvando as %d nuvens agora ...", cabecalho_proto.nuvens());
  omp_set_dynamic(0);
  #pragma omp parallel for num_threads(int(cabecalho_proto.nuvens()/2))
  for(int i=0; i < cabecalho_proto.nuvens(); i++){
    // Convertendo a nuvem para protobuf
    Nuvem nuvem_proto;
    nuvem_proto.ParseFromString(buffer_nuvens[i]);
    ROS_INFO("Convertendo a nuvem %d de %d, com nome %s ...", i+1, cabecalho_proto.nuvens(), nuvem_proto.name().c_str());
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
    // Salvar a nuvem na pasta com o nome especificado
    savePLYFileASCII<PointT>(pasta+nuvem_proto.name(), *cloud);
    cloud->clear();
  }

  // Para o arquivo NVM
  ROS_INFO("Escrevendo arquivo NVM ...");
  NVM nvm_proto;
  nvm_proto.ParseFromString(buffer_nvm);
  ofstream file(pasta+nvm_proto.name());
  if(file.is_open()){
    // Escrever cabecalho
    file << "NVM_V3\n\n";
    // Para cada linha na mensagem, escrever no arquivo de saida
    for(int i=0; i<nvm_proto.nlinhas(); i++)
      file << nvm_proto.linhas(i)+"\n";
  }
  file.close();

  ROS_INFO("Tudo recebido, processo concluido.");

  ros::spinOnce();
  return 0;
}
