//
// Created by jakub on 22. 3. 2020.
//

#include <amr_monitor/qt_widget/ClientMonitorWidget.h>
#include <ui_ClientMonitorWidget.h>


namespace amr_gui {

    ClientMonitorWidget::ClientMonitorWidget(QWidget *parent)
            : QWidget(parent),
              ui(new Ui::ClientMonitorWidget) {

        ui->setupUi(this);


    }


    void ClientMonitorWidget::updateClientInfo(const amr_msgs::ClientInfo& clientInfo) {
//        ROS_INFO("update client");
        ui->poseX->setNum(clientInfo.poseWithCovariance.pose.pose.position.x);
        ui->poseY->setNum(clientInfo.poseWithCovariance.pose.pose.position.y);
    }

}