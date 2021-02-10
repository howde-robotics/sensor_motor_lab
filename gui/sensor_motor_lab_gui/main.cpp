#include "mainwindow.h"
#include "roscomm.h"
#include <QApplication>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "UI");
  QApplication a(argc, argv);
  MainWindow w;
  w.show();

  return a.exec();
}
