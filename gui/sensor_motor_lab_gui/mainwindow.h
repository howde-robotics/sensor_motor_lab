#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QThread>
#include <QVector>

#include <chrono>
#include <thread>

#include "subscribernode.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT
public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

signals:
  void init();

//public slots:
//  void slotDisplayMotorFb(float);
//  void slotDisplaySensorFb(float);

private slots:
  void slot_motorFb(float);
  void slot_sensorFb(float);
  void on_sendCmdButton_clicked();

private:
  Ui::MainWindow *ui;
  ros::NodeHandle nh_;
  SubscriberNode *p_subscriber_node_;

  QThread *p_subcriber_node_thread_;

  const int plot_size_ = 100;
  QVector<double> motor_fb_plot_x_, motor_fb_plot_y_,
                  sensor_fb_plot_x_, sensor_fb_plot_y_;

  void printStringTextBrowser(QString QStr);
};

#endif // MAINWINDOW_H
