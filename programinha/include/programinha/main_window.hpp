/**
 * @file /include/programinha/main_window.hpp
 *
 * @brief Qt based gui for programinha.
 *
 * @date November 2010
 **/
#ifndef programinha_MAIN_WINDOW_H
#define programinha_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <libssh/libssh.h>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace programinha {

using namespace std;

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

        void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:


private Q_SLOTS:

        void on_pushButton_iniciarcaptura_clicked();
        void on_pushButton_finalizarcaptura_clicked();
        void on_pushButton_cameraimagemcalibrar_clicked();

        void on_horizontalSlider_brightness_sliderReleased();
        void on_horizontalSlider_exposure_sliderReleased();

        void on_pushButton_capturar_clicked();

        void on_pushButton_visualizar_clicked();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
        ssh_session pepo_ssh;
        int pepo_ssh_port;
};

}  // namespace programinha

#endif // programinha_MAIN_WINDOW_H
