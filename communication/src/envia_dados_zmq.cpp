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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int getdir(string dir, vector<string> &files, vector<string> &imgs, vector<string> &nuvens, string &nvm)
{
  // Abrindo a pasta raiz para contar os arquivos de imagem e nuvem que serao lidos e enviados
  DIR *dp;
  struct dirent *dirp;
  string nome_temp;
  if((dp  = opendir(dir.c_str())) == NULL)
    ROS_ERROR("Nao foi possivel abrir o diretorio");

  while ((dirp = readdir(dp)) != NULL) {
    nome_temp = string(dirp->d_name);
    // Todos os arquivos lidos
    files.push_back(nome_temp);
    // Todas as imagens na pasta
    if(nome_temp.substr(nome_temp.find_last_of(".")+1) == "png")
      imgs.push_back(nome_temp);
    // Todas as nuvens na pasta
    if(nome_temp.substr(nome_temp.find_last_of(".")+1) == "ply")
      nuvens.push_back(nome_temp);
    // Arquivo NVM
    if(nome_temp.substr(nome_temp.find_last_of(".")+1) == "nvm")
      nvm = nome_temp;
  }
  closedir(dp);

  return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "envia_dados_zmq");
  ros::NodeHandle nh;
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  /////////////////////////////////////////////
  /// Lendo o diretorio raiz para preparar
  /// todos os arquivos que serao enviados
  ///

  // Diretorio raiz
  char* home;
  home = getenv("HOME");
  string root = string(home)+"/Desktop/Dados_B9/";

  // Nome dos arquivos no diretorio
  vector<string> arquivos, nomes_imagens, nomes_nuvens;
  string nome_nvm;
  getdir(root, arquivos, nomes_imagens, nomes_nuvens, nome_nvm);
  ROS_INFO("Transmitindo %zu imagens e %zu nuvens ...", nomes_imagens.size(), nomes_nuvens.size());

  /////////////////////////////////////////////
  /// Secao de preparo de mensagens em protobuf
  ///

  // Mensagem de quantos arquivos irao, para o cliente ter nocao na recepcao
  Arquivos arq;
  arq.set_imagens(nomes_imagens.size());
  arq.set_nuvens(nomes_nuvens.size());
  string buffer_arq;
  // Serializar para string a ser enviada
  arq.SerializeToString(&buffer_arq);
  ROS_INFO("Mensagem de cabecalho preparada para cliente ...");

  // Para cada imagem
  vector<string> buffer_imagens(nomes_imagens.size()), buffer_nuvens(nomes_nuvens.size());
  omp_set_dynamic(0);
  #pragma omp parallel for num_threads(buffer_imagens.size())
  for(size_t i=0; i < buffer_imagens.size(); i++){
    // Ler imagem da pasta na area de trabalho
    string nome_imagem = nomes_imagens[i];
    Mat image = imread(root+nome_imagem);
    if(image.empty())
      ROS_ERROR("No image was found.");

    // Iniciar a mensagem protobuf de imagem com dados iniciais
    Imagem img_proto;
    img_proto.set_height(image.rows);
    img_proto.set_width(image.cols);
    img_proto.set_name(nome_imagem);

    // Varrer os pixels da imagem e adicionar na mensagem esses dados
    for(int i=0; i<img_proto.height(); i++){
      for(int j=0; j<img_proto.width(); j++){
        Vec3b cor = image.at<Vec3b>(Point(j, i));

        Imagem::Pixel *pix = img_proto.add_pixels();
        pix->set_b(cor(0));
        pix->set_g(cor(1));
        pix->set_r(cor(2));
        pix->set_u(j);
        pix->set_v(i);
      }
    }

    // Serializar em string para ser enviada e armazenar no vetor de buffer
    img_proto.SerializeToString(&buffer_imagens[i]);
    ROS_INFO("Imagem %zu de %zu lida e serializada ...", i+1, buffer_imagens.size());
  }

  // Para cada nuvem
  omp_set_dynamic(0);
  #pragma omp parallel for num_threads(buffer_nuvens.size())
  for(size_t i=0; i < buffer_nuvens.size(); i++){
    // Ler nuvem da area de trabalho
    string nome_nuvem = nomes_nuvens[i];
    PointCloud<PointT>::Ptr temp_cloud (new PointCloud<PointT>);
    loadPLYFile<PointT>(root+nome_nuvem, *temp_cloud);

    // Preparando mensagem para preencher com pontos da nuvem
    Nuvem cloud_proto;
    cloud_proto.set_name(nome_nuvem);
    cloud_proto.set_size(temp_cloud->size());

    // Preenchendo pontos na estrutura da mensagem
    PointT cloud_point;
    for(size_t i=0; i<temp_cloud->size(); i++){
      cloud_point = temp_cloud->points[i];

      Nuvem::Ponto *p = cloud_proto.add_pontos();
      p->set_x(cloud_point.x); p->set_y(cloud_point.y); p->set_z(cloud_point.z);
      p->set_r(cloud_point.r); p->set_g(cloud_point.g); p->set_b(cloud_point.b);
    }

    // Serializar em string para ser enviada
    cloud_proto.SerializeToString(&buffer_nuvens[i]);
    ROS_INFO("Nuvem %zu de %zu lida e serializada ...", i+1, buffer_nuvens.size());
  }

  // Ler arquivo nvm e formar mensagem correspondente
  ifstream file; // Arquivo em si
  NVM nvm_proto; // Mensagem compilada final
  nvm_proto.set_name("cameras.nvm");
  int conta_linhas = 0; // Quantas linhas ha no arquivo, menos as duas iniciais de cabecalho
  string linha_atual; // Qual linha estamos para guardar na mensagem
  file.open(root+nvm_proto.name());
  if(file.is_open()){
    // Para cada linha do arquivo, ler e adicionar a mensagem
    while(getline(file, linha_atual)){
      conta_linhas++;
      // Se ja passamos o cabecalho, podemos ler e guardar
      if(conta_linhas >= 3){
        string *l = nvm_proto.add_linhas();
        l->assign(linha_atual);
      }
    }
    // Adiciona por redundancia o numero de linhas que o arquivo possuia
    nvm_proto.set_nlinhas(nvm_proto.linhas_size());
  } else {
    ROS_ERROR("Nao foi achado arquivo NVM na pasta. Desligando ...");
    ros::shutdown();
  }

  // Serializar arquivo como string para enviar
  string buffer_nvm;
  nvm_proto.SerializeToString(&buffer_nvm);
  ROS_INFO("Arquivo NVM lido e serializado ...");

  /////////////////////////////////////////////
  /// Enviando as mensagens por ZMQ
  ///

  ROS_WARN("ENVIANDO DADOS POR ZMQ, PORTA 5558");

  // Criando contexto e socket do tipo PUSH, que a principio deveria esperar a outra ponta receber para enviar o proximo item
  ROS_INFO("Criando contexto e socket do publicador ...");
  context_t ctx{1};
  socket_t sender(ctx, ZMQ_PUSH); // Tipo PUSH permite aguardar o recebedor para enviar o proximo dado
  sender.bind("tcp://*:5558"); // Aqui se fez necessario o asterisco

  // Criando mensagem ZMQ de cabecalho e enviando
  message_t arq_zmq(buffer_arq.length());
  memcpy(arq_zmq.data(), buffer_arq.data(), buffer_arq.length());
  ROS_INFO("Enviando a mensagem de cabecalho ...");
  sender.send(arq_zmq);
  ROS_INFO("Mensagem de cabecalho enviada.");

  // Para cada mensagem de imagem
  for(size_t i=0; i<buffer_imagens.size(); i++){
    message_t img_zmq(buffer_imagens[i].length());
    memcpy(img_zmq.data(), buffer_imagens[i].data(), buffer_imagens[i].length());
    ROS_INFO("Enviando a mensagem %zu de %zu de imagem ...", i+1, buffer_imagens.size());
    sender.send(img_zmq);
    ROS_INFO("Mensagem %zu de imagem enviada.", i+1);
  }

  // Para cada mensagem de nuvem
  for(size_t i=0; i<buffer_nuvens.size(); i++){
    message_t nuvem_zmq(buffer_nuvens[i].length());
    memcpy(nuvem_zmq.data(), buffer_nuvens[i].data(), buffer_nuvens[i].length());
    ROS_INFO("Enviando a mensagem %zu de %zu de nuvem ...", i+1, buffer_nuvens.size());
    sender.send(nuvem_zmq);
    ROS_INFO("Mensagem %zu de nuvem enviada.", i+1);
  }

  // Para o NVM
  message_t nvm_zmq(buffer_nvm.length());
  memcpy(nvm_zmq.data(), buffer_nvm.data(), buffer_nvm.length());
  ROS_INFO("Enviando arquivo NVM ...");
  sender.send(nvm_zmq);
  ROS_INFO("Arquivo NVM enviado.");

  // Finalizando porta
  ROS_INFO("Tudo foi enviado. Terminando porta de comunicacao ZMQ.");
  sender.close();
//  ctx.close();

  ros::spinOnce();
  return 0;
}
