//
// Created by jakub on 22. 3. 2020.
//

#include <amr_monitor/qt_widget/MonitorWidget.h>
#include <ui_MonitorWidget.h>




namespace amr_gui {

    MonitorWidget::MonitorWidget(QWidget *parent)
        :   QWidget(parent),
            ui(new Ui::MonitorWidget){

        ui->setupUi(this);

        ros::NodeHandle nh("/");
        clientsInfoSub = nh.subscribe("/client_info", 5, &MonitorWidget::clientInfoCb, this);

        connect(this, SIGNAL(updateClientSignal(amr_msgs::ClientInfo)), this, SLOT(updateClientSlot(amr_msgs::ClientInfo)));
    }

    void MonitorWidget::clientInfoCb(const amr_msgs::ClientInfo::Ptr& clientInfoMsg) {
        std::lock_guard<std::mutex> lock(clientInfoMtx);
        clientInfoData[clientInfoMsg->clientId] = clientInfoMsg;
    }

    void MonitorWidget::updateWidget() {
        for (const auto &msg: clientInfoData) {


//            ui->poseX->setNum(msg.second->poseWithCovariance.pose.pose.position.x);
//            ui->poseY->setNum(msg.second->poseWithCovariance.pose.pose.position.y);
        }
    }

    void MonitorWidget::updateClientSlot(amr_msgs::ClientInfo clientInfo) {

    }


}   // amr_gui namespace

//PLUGINLIB_EXPORT_CLASS(amr_gui::MonitorWidget, rqt_gui_cpp::Plugin)