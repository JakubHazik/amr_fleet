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

        connect(ui->applyCommandBtn, SIGNAL(released()), this, SLOT(applyCommandBtnSlot()));
        connect(ui->reachabilitySetBtn, SIGNAL(released()), this, SLOT(setReachabilitySlot()));
    }

    void ControlWidget::applyCommandBtnSlot() {
        auto commandLine = ui->commandInput->text();
        auto splitStr = commandLine.split(QRegExp("\s+"));

        auto cmd = splitStr.first();
        splitStr.pop_front();
        if (cmd == "") {

        }

        ui->commandInput->setText("");  // clear input
    }

    void ControlWidget::setReachabilitySlot() {
        auto nodeUuid = ui->reachabilityUuid->text().toUInt();
        auto reachability = ui->reachabilityCheckbox->isChecked();
        auto result = controlInterface.setNodeReachability(nodeUuid, reachability);
        ui->commandOutput->append(QString(result.second.c_str()));
        ui->reachabilityUuid->setText("");
        ui->reachabilityCheckbox->setChecked(false);
    }

} // amr_gui namespace
