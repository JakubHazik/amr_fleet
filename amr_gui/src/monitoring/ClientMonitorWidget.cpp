//
// Created by jakub on 22. 3. 2020.
//

#include <amr_gui/monitoring/ClientMonitorWidget.h>
#include <ui_ClientMonitorWidget.h>

#include <tf/transform_datatypes.h>

namespace amr_gui {

    ClientMonitorWidget::ClientMonitorWidget(QWidget *parent)
            : QWidget(parent),
              ui(new Ui::ClientMonitorWidget) {

        ui->setupUi(this);

        taskNames = {
            {amr_msgs::TaskId::NO_TASK, "No task"},
            {amr_msgs::TaskId::DO_NOTHING, "Do nothing"},
            {amr_msgs::TaskId::PERFORM_WAYPOINTS, "Perform waypoints"},
            {amr_msgs::TaskId::CHARGE_BATTERY, "Charge battery"},
            {amr_msgs::TaskId::WAIT_FOR_USER_ACK, "Wait for user ack"},
            {amr_msgs::TaskId::TELEOPERATION, "Teleoperation"},
        };

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

        // set current task
        ui->taskId->setText(taskNames.at(clientInfo.currentTask.taskId.id));
        ui->taskTimeout->setNum(clientInfo.currentTask.timeout);

        // set hardware fields
        ui->batteryVoltage->setNum(clientInfo.hardwareStatus.batteryVoltage);
        ui->motorsOn->setNum(clientInfo.hardwareStatus.motorOn);
    }
}

