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

    // Acertando os limites dos sliders
    ui.horizontalSlider_exposure->setRange(0, 2200);
    ui.horizontalSlider_brightness->setRange(0, 130);
    ui.horizontalSlider_contrast->setRange(0, 50);
    ui.horizontalSlider_saturation->setRange(50, 200);
    ui.horizontalSlider_exposure->setValue(1500);
    ui.horizontalSlider_brightness->setValue(80);
    ui.horizontalSlider_contrast->setValue(20);
    ui.horizontalSlider_saturation->setValue(100);

    ui.pushButton_visualizar->setEnabled(true);

    // Acertando dials de limites de acao
    ui.dial_panmax->setRange(3, 356);
    ui.dial_panmax->setValue(356);
    ui.dial_panmin->setRange(3, 356);
    ui.dial_panmin->setValue(3);
    ui.verticalSlider_tiltmax->setRange(-28, 60);
    ui.verticalSlider_tiltmax->setValue(60);
    ui.verticalSlider_tiltmin->setRange(-28, 60);
    ui.verticalSlider_tiltmin->setValue(-28);

    // Marcando o radiobutton do space
    ui.radioButton_space->setChecked(true);
    ui.radioButton_object->setChecked(false);

    // Acertando o SSH
    pepo_ssh = ssh_new();
    int verbosity = SSH_LOG_NOLOG;
    ssh_options_set(pepo_ssh, SSH_OPTIONS_HOST, "192.168.0.101");
    ssh_options_set(pepo_ssh, SSH_OPTIONS_LOG_VERBOSITY, &verbosity);
    ssh_options_set(pepo_ssh, SSH_OPTIONS_PORT, &pepo_ssh_port);
    ssh_options_set(pepo_ssh, SSH_OPTIONS_USER, "pepo");
    int rc = ssh_connect(pepo_ssh);
    if(rc == SSH_OK)
        ui.listWidget->addItem(QString::fromStdString("Conectamos por SSH."));
    else
        ui.listWidget->addItem(QString::fromStdString("Nao foi possivel conectar por SSH."));
    string password = "12";
    rc = ssh_userauth_password(pepo_ssh, "pepo", password.c_str());
    if(rc == SSH_AUTH_SUCCESS)
        ui.listWidget->addItem(QString::fromStdString("Conectamos por SSH com o password."));
    else
        ui.listWidget->addItem(QString::fromStdString("Nao foi possivel conectar por SSH com o password."));
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
            ui.listWidget->addItem(QString::fromStdString("Comando de matar todos os nos de ROS foi enviado."));
        else
            ui.listWidget->addItem(QString::fromStdString("Comando de matar todos os nos de ROS nao pode ser executado."));
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
    string comando = "roslaunch pepo_space pepo_space.launch pasta:="+pasta+
            " inicio_pan:="+std::to_string(ui.dial_panmin->value())+
            " fim_pan:="+std::to_string(ui.dial_panmax->value())+
            " step:="+ui.lineEdit_step->text().toStdString();

    // Iniciando canal e o processo remoto
    ssh_channel channel;
    channel = ssh_channel_new(pepo_ssh);
    if(ssh_channel_open_session(channel) == SSH_OK){
        if(ssh_channel_request_exec(channel, comando.c_str()) == SSH_OK)
            ui.listWidget->addItem(QString::fromStdString("Comando de inicio do processo foi enviado."));
        else
            ui.listWidget->addItem(QString::fromStdString("Comando de inicio da captura nao pode ser executado."));
    }
    ssh_channel_close(channel);
    ssh_channel_free(channel);

    // Aguardar o inicio remoto para ligar o master assim
    sleep(3);

    // Lancar o no de fog do nosso lado
    comando = "gnome-terminal -x sh -c 'roslaunch acc_obj acc_space.launch pasta:="+pasta+
            " voxel_size:="+ui.lineEdit_voxel->text().toStdString()+
            " depth:="+ui.lineEdit_depth->text().toStdString()+
            " filter_poli:="+ui.lineEdit_poli->text().toStdString()+"'";
    cout << comando << endl;
    system(comando.c_str());

    // Habilita a visualizacao
    ui.pushButton_visualizar->setEnabled(true);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void programinha::MainWindow::on_pushButton_finalizarcaptura_clicked(){
    // Iniciando canal SSH
    ssh_channel channel;
    channel = ssh_channel_new(pepo_ssh);
    if(ssh_channel_open_session(channel) == SSH_OK){
        // Enviando comando para matar os nos
        if(ssh_channel_request_exec(channel, "rosnode kill --all") == SSH_OK)
            ui.listWidget->addItem(QString::fromStdString("Comando de matar todos os nos de ROS foi enviado."));
        else
            ui.listWidget->addItem(QString::fromStdString("Comando de matar todos os nos de ROS nao pode ser executado."));
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
            ui.listWidget->addItem(QString::fromStdString("Brilho alterado com sucesso para "+to_string(valor)+"."));
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
            ui.listWidget->addItem(QString::fromStdString("Tiramos exposicao automatica."));
    }
    ssh_channel_close(channel);
    ssh_channel_free(channel);
    channel = ssh_channel_new(pepo_ssh);
    if(ssh_channel_open_session(channel) == SSH_OK){
        if(ssh_channel_request_exec(channel, comando.c_str()) == SSH_OK)
            ui.listWidget->addItem(QString::fromStdString("Exposicao alterada com sucesso para "+to_string(valor)+"."));
    }
    ssh_channel_close(channel);
    ssh_channel_free(channel);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void programinha::MainWindow::on_horizontalSlider_contrast_sliderReleased(){
    // Pega o valor
    int valor = ui.horizontalSlider_contrast->value();
    string comando = "v4l2-ctl --set-ctrl=contrast="+to_string(valor);
    // Inicia o canal e envia o comando
    ssh_channel channel;
    channel = ssh_channel_new(pepo_ssh);
    if(ssh_channel_open_session(channel) == SSH_OK){
        if(ssh_channel_request_exec(channel, comando.c_str()) == SSH_OK)
            ui.listWidget->addItem(QString::fromStdString("Contraste alterado com sucesso para "+to_string(valor)+"."));
    }
    ssh_channel_close(channel);
    ssh_channel_free(channel);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void programinha::MainWindow::on_horizontalSlider_saturation_sliderReleased(){
    // Pega o valor
    int valor = ui.horizontalSlider_saturation->value();
    string comando = "v4l2-ctl --set-ctrl=saturation="+to_string(valor);
    // Inicia o canal e envia o comando
    ssh_channel channel;
    channel = ssh_channel_new(pepo_ssh);
    if(ssh_channel_open_session(channel) == SSH_OK){
        if(ssh_channel_request_exec(channel, comando.c_str()) == SSH_OK)
            ui.listWidget->addItem(QString::fromStdString("Saturacao alterada com sucesso para "+to_string(valor)+"."));
    }
    ssh_channel_close(channel);
    ssh_channel_free(channel);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void programinha::MainWindow::on_pushButton_cameraimagemcalibrar_clicked(){
    system("gnome-terminal -x sh -c 'export ROS_IP=192.168.0.102 && export ROS_MASTER_URI=http://192.168.0.101:11311 && rqt_image_view /camera/image_raw'");
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void programinha::MainWindow::on_pushButton_visualizar_clicked()
{
    system("gnome-terminal -x sh -c 'export ROS_IP=192.168.0.102 && export ROS_MASTER_URI=http://192.168.0.101:11311 && rosrun rviz rviz -d $HOME/pepo_ws/src/PEPO/programinha/resources/visual.rviz'"    );
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void programinha::MainWindow::on_pushButton_transferircaptura_clicked(){
    // Inicia o canal e envia o comando
    ssh_channel channel;
    channel = ssh_channel_new(pepo_ssh);
    string comando = "rsync -avz -e 'ssh' /home/pepo/Desktop/"+ui.lineEdit_pasta->text().toStdString()+" vinicius@192.168.0.102:/home/vinicius/Desktop/";
    if(ssh_channel_open_session(channel) == SSH_OK){
        if(ssh_channel_request_exec(channel, comando.c_str()) == SSH_OK)
            ui.listWidget->addItem(QString::fromStdString("Transferindo a pasta ")+ui.lineEdit_pasta->text()+QString::fromStdString(" para o Desktop."));
    }
    ssh_channel_close(channel);
    ssh_channel_free(channel);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void programinha::MainWindow::on_radioButton_space_clicked(){

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void programinha::MainWindow::on_radioButton_object_clicked(){

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
