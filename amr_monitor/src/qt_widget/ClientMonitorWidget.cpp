//
// Created by jakub on 22. 3. 2020.
//

#include <amr_monitor/qt_widget/ClientMonitorWidget.h>
#include <ui_ClientMonitorWidget.h>

#include <tf/transform_datatypes.h>

namespace amr_gui {

    ClientMonitorWidget::ClientMonitorWidget(QWidget *parent)
            : QWidget(parent),
              ui(new Ui::ClientMonitorWidget) {

        ui->setupUi(this);

    }

    void ClientMonitorWidget::updateClientInfo(const amr_msgs::ClientInfo& clientInfo) {
        // set current pose fields
        ui->poseX->setNum(clientInfo.poseWithCovariance.pose.pose.position.x);
        ui->poseY->setNum(clientInfo.poseWithCovariance.pose.pose.position.y);
        ui->poseTheta->setNum(tf::getYaw(clientInfo.poseWithCovariance.pose.pose.orientation));

        // set current goal fields
        ui->goalX->setNum(clientInfo.robotCurrentGoal.pose.x);
        ui->goalY->setNum(clientInfo.robotCurrentGoal.pose.y);
        ui->goalTheta->setNum(clientInfo.robotCurrentGoal.pose.theta);
        ui->goalUuid->setNum(static_cast<int>(clientInfo.robotCurrentGoal.uuid));

        // set hardware fields
        ui->batteryVoltage->setNum(clientInfo.hardwareStatus.batteryVoltage);
        ui->motorsOn->setNum(clientInfo.hardwareStatus.motorOn);
    }
}

