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
  void sigSendMotorSelection(int);

private slots:
  void slot_motor1Fb(float);
  void slot_sensor1Fb(float);

  void on_sendCmd1Button_clicked();
  void on_motor1RadioButton_clicked();
  void on_motor2RadioButton_clicked();
  void on_motor3RadioButton_clicked();

private:
  Ui::MainWindow *ui;
  ros::NodeHandle nh_;
  SubscriberNode *p_subscriber_node_;
  PublisherNode *p_publisher_node_;

  QThread p_subcriber_node_thread_;
  QThread p_publisher_node_thread_;

  const int plot_size_ = 100;
  const double plot_time_scale_ = 0.05;
  const double y_scale_ = 0.01;
  QVector<double> motor1_fb_plot_x_, motor1_fb_plot_y_,
                  sensor1_fb_plot_x_, sensor1_fb_plot_y_;

  const int maxTextBrowserSize_ = 1000;
  QString textBrowserStr_;
  std::chrono::time_point<std::chrono::system_clock> start_time_;

  enum motorSelection {
    motor1,
    motor2,
    motor3
  };
  int curr_motor_selection = motorSelection::motor1;

  void printStringTextBrowser(QString QStr);
  static void setupPlot(QCustomPlot *plot, QVector<double> &plot_x, QVector<double> &plot_y, QString y_axis_str, int plot_size, double time_scale, double y_scale);
  static void drawPlot(QCustomPlot* plot, QVector<double> &plot_x, QVector<double> &plot_y, float sig);
  void processCmdButton(QLineEdit* line_edit, QString motor_id, void (MainWindow::* signal)(float));
  void processRadioButton(QString motor_selection_str, int motor_selection, void (MainWindow:: *signal)(int));
};

#endif // MAINWINDOW_H
