/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/programinha/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace programinha {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    setWindowIcon(QIcon(":/images/icon.png"));
    //	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    // Acertando o SSH
    pepo_ssh = ssh_new();
    int verbosity = SSH_LOG_WARNING;
    ssh_options_set(pepo_ssh, SSH_OPTIONS_HOST, "192.168.0.101");
    ssh_options_set(pepo_ssh, SSH_OPTIONS_LOG_VERBOSITY, &verbosity);
    ssh_options_set(pepo_ssh, SSH_OPTIONS_PORT, &pepo_ssh_port);
    ssh_options_set(pepo_ssh, SSH_OPTIONS_USER, "pepo");
    int rc = ssh_connect(pepo_ssh);
    if(rc == SSH_OK)
        ROS_INFO("Conectamos por SSH.");
    else
        ROS_ERROR("Nao foi possivel conectar por SSH.");
    string password = "12";
    rc = ssh_userauth_password(pepo_ssh, "pepo", password.c_str());
    if(rc == SSH_AUTH_SUCCESS)
        ROS_INFO("Conectamos por SSH com o password.");
    else
        ROS_ERROR("Nao foi possivel conectar por SSH com o password.");

    // Acertando os limites dos sliders
    ui.horizontalSlider_exposure->setRange(300, 2200);
    ui.horizontalSlider_brightness->setRange(50, 130);
    ui.horizontalSlider_exposure->setValue(1500);
    ui.horizontalSlider_brightness->setValue(80);

    // Iniciando o contador para qual aquisicao estamos
    contador_aquisicao = 0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
MainWindow::~MainWindow() {}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::closeEvent(QCloseEvent *event)
{
    ssh_channel channel;
    channel = ssh_channel_new(pepo_ssh);
    if(ssh_channel_open_session(channel) == SSH_OK){
        // Enviando comando para matar os nos
        if(ssh_channel_request_exec(channel, "rosnode kill --all") == SSH_OK)
            ROS_INFO("Comando de matar todos os nos de ROS foi enviado.");
        else
            ROS_INFO("Comando de matar todos os nos de ROS nao pode ser executado.");
    }
    ssh_channel_close(channel);
    ssh_channel_free(channel);
    ssh_disconnect(pepo_ssh);
    ssh_free(pepo_ssh);
    system("gnome-terminal -x sh -c 'rosnode kill --all'");
    QMainWindow::closeEvent(event);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}  // namespace programinha
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void programinha::MainWindow::on_pushButton_iniciarcaptura_clicked()
{
    // Nome da pasta para gravar os dados
    string pasta = ui.lineEdit_pasta->text().toStdString();
    // Montando comando de acordo com a escolha nos radiobuttons
    string comando;
    if(ui.radioButton_space->isChecked())
        comando = "roslaunch pepo_space pepo_space.launch pasta:="+pasta;
    else if(ui.radioButton_object->isChecked())
        comando = "roslaunch pepo_obj pepo_obj.launch pasta:="+pasta;

    // Iniciando canal e secao SSH
    ssh_channel channel;
    channel = ssh_channel_new(pepo_ssh);
    if(ssh_channel_open_session(channel) == SSH_OK){
        if(ssh_channel_request_exec(channel, comando.c_str()) == SSH_OK)
            ROS_INFO("Comando de inicio do processo foi enviado.");
        else
            ROS_INFO("Comando de inicio da captura nao pode ser executado.");
    }
    ssh_channel_close(channel);
    ssh_channel_free(channel);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void programinha::MainWindow::on_pushButton_finalizarcaptura_clicked(){
    // Iniciando canal e secao SSH
    ssh_channel channel;
    if(ui.radioButton_object->isChecked()){
        channel = ssh_channel_new(pepo_ssh);
        if(ssh_channel_open_session(channel) == SSH_OK){
            // Enviando comando para matar os nos
            if(ssh_channel_request_exec(channel, "rosservice call /proceder_obj 2") == SSH_OK)
                ROS_INFO("Fechando a pasta de captura do objeto.");
        }
        ssh_channel_close(channel);
        ssh_channel_free(channel);
    }
    channel = ssh_channel_new(pepo_ssh);
    if(ssh_channel_open_session(channel) == SSH_OK){
        // Enviando comando para matar os nos
        if(ssh_channel_request_exec(channel, "rosnode kill --all") == SSH_OK)
            ROS_INFO("Comando de matar todos os nos de ROS foi enviado.");
        else
            ROS_INFO("Comando de matar todos os nos de ROS nao pode ser executado.");
    }
    ssh_channel_close(channel);
    ssh_channel_free(channel);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void programinha::MainWindow::on_horizontalSlider_brightness_sliderReleased(){
    // Pega o valor
    int valor = ui.horizontalSlider_brightness->value();
    string comando = "v4l2-ctl --set-ctrl=brightness="+to_string(valor);
    // Inicia o canal e envia o comando
    ssh_channel channel;
    channel = ssh_channel_new(pepo_ssh);
    if(ssh_channel_open_session(channel) == SSH_OK){
        if(ssh_channel_request_exec(channel, comando.c_str()) == SSH_OK)
            ROS_INFO("Brilho alterado com sucesso");
    }
    ssh_channel_close(channel);
    ssh_channel_free(channel);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void programinha::MainWindow::on_horizontalSlider_exposure_sliderReleased(){
    // Pega o valor
    int valor = ui.horizontalSlider_exposure->value();
    string comando = "v4l2-ctl --set-ctrl=exposure_absolute="+to_string(valor);
    // Inicia o canal e envia o comando
    ssh_channel channel;
    channel = ssh_channel_new(pepo_ssh);
    if(ssh_channel_open_session(channel) == SSH_OK){
        if(ssh_channel_request_exec(channel, "v4l2-ctl --set-ctrl=exposure_auto=1") == SSH_OK)
            ROS_INFO("Tiramos exposicao automatica.");
    }
    ssh_channel_close(channel);
    ssh_channel_free(channel);
    channel = ssh_channel_new(pepo_ssh);
    if(ssh_channel_open_session(channel) == SSH_OK){
        if(ssh_channel_request_exec(channel, comando.c_str()) == SSH_OK)
            ROS_INFO("Exposicao alterada com sucesso");
    }
    ssh_channel_close(channel);
    ssh_channel_free(channel);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void programinha::MainWindow::on_pushButton_cameraimagemcalibrar_clicked(){
    system("gnome-terminal -x sh -c 'rqt_image_view /imagem'");
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void programinha::MainWindow::on_pushButton_capturar_clicked()
{
    // Inicia o canal e envia o comando
    ssh_channel channel;
    channel = ssh_channel_new(pepo_ssh);
    if(ssh_channel_open_session(channel) == SSH_OK){
        if(ssh_channel_request_exec(channel, "rosservice call /proceder_obj 1") == SSH_OK){
            ROS_INFO("Enviado pedido de captura");
            // Aguarda um tempo para o comando ser executado e acusa que esta bem
            sleep(10);
            contador_aquisicao++;
            ROS_INFO("Aquisicao %d feita com sucesso.", contador_aquisicao);
        }
    }
    ssh_channel_close(channel);
    ssh_channel_free(channel);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void programinha::MainWindow::on_pushButton_visualizar_clicked()
{
    system("gnome-terminal -x sh -c 'rosrun rviz rviz -d $HOME/pepo_ws/src/PEPO/programinha/resources/visual.rviz'");
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
