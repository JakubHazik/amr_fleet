//
// Created by jakub on 22. 3. 2020.
//

#ifndef SRC_CLIENTMONITORWIDGET_H
#define SRC_CLIENTMONITORWIDGET_H

#include <QWidget>
#include <QtWidgets>

#include <ros/ros.h>
#include <amr_msgs/ClientInfo.h>
#include <mutex>

namespace Ui {
    class ClientMonitorWidget;
}

namespace amr_gui {
    class ClientMonitorWidget : public QWidget {
    Q_OBJECT

    public:
        explicit ClientMonitorWidget(QWidget *parent = nullptr);

        void updateClientInfo(const amr_msgs::ClientInfo& clientInfo);
    private:
        Ui::ClientMonitorWidget *ui;

    };
}

#endif //SRC_CLIENTMONITORWIDGET_H
