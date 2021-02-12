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
  void sigSendCmd1(float);

private slots:
  void slot_motor1Fb(float);
  void slot_sensor1Fb(float);
  void slot_motor2Fb(float);
  void slot_sensor2Fb(float);
  void slot_motor3Fb(float);
  void slot_sensor3Fb(float);

  void on_sendCmd1Button_clicked();

private:
  Ui::MainWindow *ui;
  ros::NodeHandle nh_;
  SubscriberNode *p_subscriber_node_;
  PublisherNode *p_publisher_node_;

  QThread *p_subcriber_node_thread_;
  QThread *p_publisher_node_thread_;

  const int plot_size_ = 100;
  const double plot_time_scale_ = 0.05;
  QVector<double> motor1_fb_plot_x_, motor1_fb_plot_y_,
                  sensor1_fb_plot_x_, sensor1_fb_plot_y_,
                  motor2_fb_plot_x_, motor2_fb_plot_y_,
                  sensor2_fb_plot_x_, sensor2_fb_plot_y_,
                  motor3_fb_plot_x_, motor3_fb_plot_y_,
                  sensor3_fb_plot_x_, sensor3_fb_plot_y_;

  QString textBrowserStr_;
  std::chrono::time_point<std::chrono::system_clock> start_time_;

  void printStringTextBrowser(QString QStr);
  void setupPlot(QCustomPlot *plot, QVector<double> &plot_x, QVector<double> &plot_y, QString y_axis_str, int plot_size, double time_scale);
  void drawPlot(QCustomPlot* plot, QVector<double> &plot_x, QVector<double> &plot_y, float sig);
};

#endif // MAINWINDOW_H
