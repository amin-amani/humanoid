#ifndef HAND_GUI_H
#define HAND_GUI_H

#include <QMainWindow>
#include <rqt_gui_cpp/plugin.h>
//#include <hand_gui.h>
#include <QWidget>


namespace Ui {
class hand_gui;
}

class hand_gui : public QMainWindow
{
    Q_OBJECT

public:
    explicit hand_gui(QWidget *parent = 0);
    ~hand_gui();

private:
    Ui::hand_gui *ui;
};

#endif // HAND_GUI_H
