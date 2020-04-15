#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <dirent.h>
#include <errno.h>
#include <ostream>
#include <istream>

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
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int getdir(string dir, vector<string> &files, vector<string> &imgs, vector<string> &nuvens, string &nvm)
{
	// Abrindo a pasta raiz para contar os arquivos de imagem e nuvem que serao lidos e enviados
	DIR *dp;
	struct dirent *dirp;
	string nome_temp;
	if ((dp = opendir(dir.c_str())) == NULL)
		printf("Nao foi possivel abrir o diretorio");

	while ((dirp = readdir(dp)) != NULL) {
		nome_temp = string(dirp->d_name);
		// Todos os arquivos lidos
		files.push_back(nome_temp);
		// Todas as imagens na pasta
		if (nome_temp.substr(nome_temp.find_last_of(".") + 1) == "png")
			imgs.push_back(nome_temp);
		// Todas as nuvens na pasta
		if (nome_temp.substr(nome_temp.find_last_of(".") + 1) == "ply")
			nuvens.push_back(nome_temp);
		// Arquivo NVM
		if (nome_temp.substr(nome_temp.find_last_of(".") + 1) == "nvm")
			nvm = nome_temp;
	}
	closedir(dp);

	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
	
	pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

	/////////////////////////////////////////////
	/// Lendo o diretorio raiz para preparar
	/// todos os arquivos que serao enviados
	///

	// Diretorio raiz
	char* home;
	home = getenv("HOME");
	string root = "C:/Users/julia/Pictures/ArquivosB9/Dados_B9_seguranvm/";

	// Nome dos arquivos no diretorio
	vector<string> arquivos, nomes_imagens, nomes_nuvens;
	string nome_nvm;
	getdir(root, arquivos, nomes_imagens, nomes_nuvens, nome_nvm);
	printf("Transmitindo %zu imagens e %zu nuvens ...\n", nomes_imagens.size(), nomes_nuvens.size());

	/////////////////////////////////////////////
	/// Iniciando objetos de protobuf e contexto
	/// e socket ZMQ para enviar
	

	// Entidades protobuf
	Arquivos arq_proto;
	Imagem img_proto;
	Nuvem cloud_proto;
	NVM nvm_proto;

	// Criando contexto e socket do tipo PUSH, que a principio deveria esperar a outra ponta receber para enviar o proximo item
	printf("Criando contexto e socket do publicador ...\n");
	context_t ctx{ 1 };
	socket_t sender(ctx, ZMQ_PUSH); // Tipo PUSH permite aguardar o recebedor para enviar o proximo dado
	sender.bind("tcp://*:5558"); // Aqui se fez necessario o asterisco

	/////////////////////////////////////////////
	/// Trabalhando e enviando cabecalho

	arq_proto.set_imagens(nomes_imagens.size());
	arq_proto.set_nuvens(nomes_nuvens.size());
	string buffer_arq;
	// Serializar para string a ser enviada
	arq_proto.SerializeToString(&buffer_arq);
	printf("Mensagem de cabecalho preparada para cliente ...\n");

	// Criando mensagem ZMQ de cabecalho e enviando
	message_t arq_zmq(buffer_arq.length());
	memcpy(arq_zmq.data(), buffer_arq.data(), buffer_arq.length());
	printf("Enviando a mensagem de cabecalho ...\n");
	sender.send(arq_zmq);
	printf("Mensagem de cabecalho enviada.\n");

	// Liberando memoria
	buffer_arq.clear();

	/////////////////////////////////////////////
	/// Trabalhando todas as imagens e enviando


	printf("Comecando o envio de imagens ...\n");
	Mat image;
	string buffer_imagem;
	for (int k = 0; k < arq_proto.imagens(); k++) {
		// Ler imagem da pasta na area de trabalho
		image = imread(root + nomes_imagens[k]);
		if (image.empty())
			printf("No image was found.\n");

		// Iniciar a mensagem protobuf de imagem com dados iniciais
		img_proto.set_height(image.rows);
		img_proto.set_width(image.cols);
		img_proto.set_name(nomes_imagens[k]);

		// Varrer os pixels da imagem e adicionar na mensagem esses dados
		Vec3b cor;
		for (int i = 0; i < img_proto.height(); i++) {
			for (int j = 0; j < img_proto.width(); j++) {
				cor = image.at<Vec3b>(Point(j, i));

				Imagem::Pixel *pix = img_proto.add_pixels();
				pix->set_b(cor(0));
				pix->set_g(cor(1));
				pix->set_r(cor(2));
				pix->set_u(j);
				pix->set_v(i);
			}
		}
		image.release();

		// Serializar em string para ser enviada e armazenar no vetor de buffer
		img_proto.SerializeToString(&buffer_imagem);

		// Copiando para mensagem ZMQ e enviando
		message_t img_zmq(buffer_imagem.length());
		memcpy(img_zmq.data(), buffer_imagem.data(), buffer_imagem.length());
		printf("Enviando a mensagem %d de %d de imagem ...\n", k + 1, arq_proto.imagens());
		sender.send(img_zmq);
		printf("Mensagem %d de imagem enviada.\n", k + 1);

		// Apagando dados para aliviar memoria
		img_proto.Clear();
		buffer_imagem.clear();
	}

	/////////////////////////////////////////////
	/// Trabalhando e enviando todas as nuvens
	

	printf("Comecando o envio de nuvens ...\n");
	string buffer_nuvem;
	for (int i = 0; i < arq_proto.nuvens(); i++) {
		// Ler nuvem da area de trabalho
		PointCloud<PointT>::Ptr temp_cloud(new PointCloud<PointT>);
		loadPLYFile<PointT>(root + nomes_nuvens[i], *temp_cloud);

		// Preparando mensagem para preencher com pontos da nuvem
		cloud_proto.set_name(nomes_nuvens[i]);
		cloud_proto.set_size(temp_cloud->size());

		// Preenchendo pontos na estrutura da mensagem
		PointT cloud_point;
		for (size_t k = 0; k < temp_cloud->size(); k++) {
			cloud_point = temp_cloud->points[k];

			Nuvem::Ponto *p = cloud_proto.add_pontos();
			p->set_x(cloud_point.x); p->set_y(cloud_point.y); p->set_z(cloud_point.z);
			p->set_r(cloud_point.r); p->set_g(cloud_point.g); p->set_b(cloud_point.b);
		}
		temp_cloud->clear();

		// Serializar em string para ser enviada
		cloud_proto.SerializeToString(&buffer_nuvem);

		// Copiando para mensagem ZMQ e enviando
		message_t nuvem_zmq(buffer_nuvem.length());
		memcpy(nuvem_zmq.data(), buffer_nuvem.data(), buffer_nuvem.length());
		printf("Enviando a mensagem %d de %d de nuvem ...\n", i + 1, arq_proto.nuvens());
		sender.send(nuvem_zmq);
		printf("Mensagem %d de nuvem enviada.\n", i + 1);

		// Limpar memoria
		cloud_proto.Clear();
		buffer_nuvem.clear();
	}


	/////////////////////////////////////////////
	/// Trabalhando e enviando arquivo NVM
	

	printf("Comecando envio de arquivo NVM ...\n");
	// Ler arquivo nvm e formar mensagem correspondente
	ifstream file; // Arquivo em si
	nvm_proto.set_name("cameras.nvm");
	int conta_linhas = 0; // Quantas linhas ha no arquivo, menos as duas iniciais de cabecalho
	string linha_atual; // Qual linha estamos para guardar na mensagem
	file.open(root + nvm_proto.name());
	if (file.is_open()) {
		// Para cada linha do arquivo, ler e adicionar a mensagem
		while (getline(file, linha_atual)) {
			conta_linhas++;
			// Se ja passamos o cabecalho, podemos ler e guardar
			if (conta_linhas >= 3) {
				string *l = nvm_proto.add_linhas();
				l->assign(linha_atual);
			}
		}
		// Adiciona por redundancia o numero de linhas que o arquivo possuia
		nvm_proto.set_nlinhas(nvm_proto.linhas_size());
	}
	else {
		printf("Nao foi achado arquivo NVM na pasta. Desligando ...\n");
		
	}

	// Serializar arquivo como string para enviar
	string buffer_nvm;
	nvm_proto.SerializeToString(&buffer_nvm);

	// Enviando a mensagem em ZMQ
	message_t nvm_zmq(buffer_nvm.length());
	memcpy(nvm_zmq.data(), buffer_nvm.data(), buffer_nvm.length());
	printf("Enviando arquivo NVM ...\n");
	sender.send(nvm_zmq);
	printf("Arquivo NVM enviado.\n\n");

	// Liberando memoria
	nvm_proto.Clear();
	buffer_nvm.clear();

	/////////////////////////////////////////////
	/// Finalizando o socket e o programa
	
	printf("Tudo foi enviado. Terminando porta de comunicacao ZMQ e processo.\n\n");
	sender.close();


	return 0;
}