//
// Created by jakub on 24. 3. 2020.
//

#include <amr_gui/control/ControlWidget.h>

#include <ui_ControlWidget.h>

#include <QDebug>

namespace amr_gui {

    ControlWidget::ControlWidget(QWidget *parent)
            : QWidget(parent),
              ui(new Ui::ControlWidget) {

        // qt setup
        ui->setupUi(this);

        connect(ui->widgetJoy, &JoyPad::xChanged, this, [this](double x){
            controlInterface.setTeleopSpeed(ui->widgetJoy->y(), -x);
        });
        connect(ui->widgetJoy, &JoyPad::yChanged, this, [this](double y){
            controlInterface.setTeleopSpeed(y, -ui->widgetJoy->x());
        });
    }

    void ControlWidget::on_applyCommandBtn_clicked() {
        auto commandLine = ui->commandInput->text();
        auto splitStr = commandLine.split(QRegExp("\s+"));

        auto cmd = splitStr.first();
        splitStr.pop_front();
        if (cmd == "") {

        }

        ui->commandInput->setText("");  // clear input
    }

    void ControlWidget::on_reachabilitySetBtn_clicked() {
        auto nodeUuid = ui->reachabilityUuid->text().toUInt();
        auto reachability = ui->reachabilityCheckbox->isChecked();
        auto result = controlInterface.setNodeReachability(nodeUuid, reachability);
        ui->commandOutput->append(QString(result.second.c_str()));
        ui->reachabilityUuid->setText("");
        ui->reachabilityCheckbox->setChecked(false);
    }

    void ControlWidget::on_startTeleopBtn_clicked() {
        // start teleop
        auto clientId = ui->teleopClientId->text();
        auto result = controlInterface.startTeleopClient(clientId.toStdString());
        ui->commandOutput->append(QString(result.second.c_str()));
        ui->teleopClientId->setDisabled(true);
        ui->startTeleopBtn->setDisabled(true);
        ui->stopTeleopBtn->setDisabled(false);
        ui->widgetJoy->setDisabled(false);
    }

    void ControlWidget::on_stopTeleopBtn_clicked() {
        auto clientId = ui->teleopClientId->text();
        auto result = controlInterface.stopTeleopClient(clientId.toStdString());
        ui->commandOutput->append(QString(result.second.c_str()));
        ui->teleopClientId->setDisabled(false);
        ui->startTeleopBtn->setDisabled(false);
        ui->stopTeleopBtn->setDisabled(true);
        ui->widgetJoy->setDisabled(true);
    }

} // amr_gui namespace
