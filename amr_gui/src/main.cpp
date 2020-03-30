//
// Created by jakub on 22. 3. 2020.
//

#include <QtGui>
#include <QApplication>
#include <amr_gui/MainWindow.h>
#include <ros/ros.h>
#include <csignal>

int main(int argc, char **argv) {

    ros::init(argc, argv, "mrvk_gui");

    QApplication app(argc, argv);
    MainWindow w(argc, argv);
    w.show();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    signal(SIGINT, SIG_DFL);    // listen interrupt signal from console

    return QApplication::exec();
}