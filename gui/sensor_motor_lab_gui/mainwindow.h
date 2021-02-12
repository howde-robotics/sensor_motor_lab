#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QThread>
#include <QVector>

#include <chrono>
#include <thread>

#include "subscribernode.h"
#include "publishernode.h"
#include "qcustomplot.h"

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
  void sigSendCmd(float);

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
  PublisherNode *p_publisher_node_;

  QThread *p_subcriber_node_thread_;
  QThread *p_publisher_node_thread_;

  const int plot_size_ = 100;
  const double plot_time_scale_ = 0.05;
  QVector<double> motor_fb_plot_x_, motor_fb_plot_y_,
                  sensor_fb_plot_x_, sensor_fb_plot_y_;

  void printStringTextBrowser(QString QStr);
  void setupPlot(QCustomPlot *plot, QVector<double> &plot_x, QVector<double> &plot_y, QString y_axis_str, int plot_size, double time_scale);
};

#endif // MAINWINDOW_H
