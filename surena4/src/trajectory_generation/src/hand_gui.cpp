#include "hand_gui.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include "ui_hand_gui.h"

hand_gui::hand_gui(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::hand_gui)
{
    ui->setupUi(this);
}

hand_gui::~hand_gui()
{
    delete ui;
}
