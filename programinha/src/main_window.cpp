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

    // No externo pode funcionar
    qnode.run();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
MainWindow::~MainWindow() {}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MainWindow::closeEvent(QCloseEvent *event)
{
    ssh_disconnect(pepo_ssh);
    ssh_free(pepo_ssh);
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
    char buffer[256];
    int nbytes;
    if(ssh_channel_open_session(channel) == SSH_OK){
        if(ssh_channel_request_exec(channel, comando.c_str()) == SSH_OK)
            ROS_INFO("Comando de inicio do processo foi enviado.");
        else
            ROS_ERROR("Comando de inicio da captura nao pode ser executado.");
        // Acertando a camera de acordo com os sliders
        ssh_channel_request_exec(channel, "v4l2-ctl --set-ctrl=exposure_auto=1");
//        ssh_channel_request_exec(channel, ("v4l2-ctl --set-ctrl=exposure_absolute="+to_string(ui.horizontalSlider_exposure->value())).c_str());
//        ssh_channel_request_exec(channel, ("v4l2-ctl --set-ctrl=brightness="+to_string(ui.horizontalSlider_brightness->value())).c_str());
        // Falando o que tem de output do roslaunch
        char buffer[256];
        int nbytes;
        nbytes = ssh_channel_read(channel, buffer, sizeof(buffer), 0);
        while (nbytes > 0)
        {
          if (fwrite(buffer, 1, nbytes, stdout) != nbytes)
          {
            ssh_channel_close(channel);
            ssh_channel_free(channel);
            cout<< SSH_ERROR;
          }
          nbytes = ssh_channel_read(channel, buffer, sizeof(buffer), 0);
        }

    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void programinha::MainWindow::on_pushButton_finalizarcaptura_clicked(){
    // Iniciando canal e secao SSH
    ssh_channel channel;
    channel = ssh_channel_new(pepo_ssh);
    if(ssh_channel_open_session(channel) == SSH_OK){
        // Enviando comando para matar os nos
        if(ssh_channel_request_exec(channel, "rosnode kill --all") == SSH_OK)
            ROS_INFO("Comando de matar todos os nos de ROS foi enviado.");
        else
            ROS_ERROR("Comando de matar todos os nos de ROS nao pode ser executado.");
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void programinha::MainWindow::on_horizontalSlider_brightness_sliderReleased(){
    // Pega o valor
    int valor = ui.horizontalSlider_brightness->value();
    string comando = "v4l2-ctl --set-ctrl=brightness=";//+to_string(valor);
    // Inicia o canal e envia o comando
    ssh_channel channel;
    channel = ssh_channel_new(pepo_ssh);
    if(ssh_channel_open_session(channel) == SSH_OK){
        if(ssh_channel_request_exec(channel, comando.c_str()) == SSH_OK)
            ROS_INFO("Brilho alterado com sucesso");
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void programinha::MainWindow::on_horizontalSlider_exposure_sliderReleased(){
    // Pega o valor
    int valor = ui.horizontalSlider_exposure->value();
    string comando = "v4l2-ctl --set-ctrl=exposure_absolute=";//+to_string(valor);
    // Inicia o canal e envia o comando
    ssh_channel channel;
    channel = ssh_channel_new(pepo_ssh);
    if(ssh_channel_open_session(channel) == SSH_OK){
        ssh_channel_request_exec(channel, "v4l2-ctl --set-ctrl=exposure_auto=1");
        if(ssh_channel_request_exec(channel, comando.c_str()) == SSH_OK)
            ROS_INFO("Exposicao alterada com sucesso");
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void programinha::MainWindow::on_pushButton_cameraimagemcalibrar_clicked(){
    system("gnome-terminal -x sh -c 'rqt_image_view /imagem'");
}
