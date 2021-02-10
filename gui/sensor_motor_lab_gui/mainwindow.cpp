#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  // Plotting
  motor_fb_plot_x_ = QVector<double>(plot_size_, 0.0);
  motor_fb_plot_y_ = QVector<double>(plot_size_, 0.0);
  ui->motorPlot1->addGraph();
  ui->motorPlot1->xAxis->setLabel("time");
  ui->motorPlot1->yAxis->setLabel("motor position");
  ui->motorPlot1->xAxis->setRange(0, plot_size_);
  ui->motorPlot1->yAxis->setRange(0, plot_size_);
  sensor_fb_plot_x_ = QVector<double>(plot_size_, 0.0);
  sensor_fb_plot_y_ = QVector<double>(plot_size_, 0.0);
  ui->sensorPlot1->addGraph();
  ui->sensorPlot1->xAxis->setLabel("time");
  ui->sensorPlot1->yAxis->setLabel("sensor position");
  ui->sensorPlot1->xAxis->setRange(0, plot_size_);
  ui->sensorPlot1->yAxis->setRange(0, plot_size_);
  for (int i = 0; i < plot_size_; ++i) {
    motor_fb_plot_x_[i] = i;
    sensor_fb_plot_x_[i] = i;
  }

  // ROS
  p_subscriber_node_ = new SubscriberNode(&nh_);
  connect(this, SIGNAL(init()), p_subscriber_node_, SLOT(slotStartSubs()));
  p_subcriber_node_thread_ = new QThread();
  connect(p_subscriber_node_, SIGNAL(finished()), p_subcriber_node_thread_, SLOT(quit()));
  p_subscriber_node_->moveToThread(p_subcriber_node_thread_);
  connect(p_subscriber_node_, SIGNAL(sigMotorFb(float)), this, SLOT(slot_motorFb(float)));
  connect(p_subscriber_node_, SIGNAL(sigSensorFb(float)), this, SLOT(slot_sensorFb(float)));
  p_subcriber_node_thread_->start();
  sleep(1);
  emit init();
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::printStringTextBrowser(QString QStr) {
  ui->textBrowser->setText(QStr);
}

void MainWindow::slot_motorFb(float sig) {
  motor_fb_plot_y_.pop_front();
  motor_fb_plot_y_.push_back(double(sig));
  ui->motorPlot1->graph(0)->setData(motor_fb_plot_x_, motor_fb_plot_y_);
  ui->motorPlot1->replot();
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
}

void MainWindow::slot_sensorFb(float sig) {
  sensor_fb_plot_y_.pop_front();
  sensor_fb_plot_y_.push_back(double(sig));
  ui->sensorPlot1->graph(0)->setData(sensor_fb_plot_x_, sensor_fb_plot_y_);
  ui->sensorPlot1->replot();
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
}

void MainWindow::on_sendCmdButton_clicked()
{
  QString inputStr = ui->cmdLineEdit->text();
  printStringTextBrowser(inputStr);
}
