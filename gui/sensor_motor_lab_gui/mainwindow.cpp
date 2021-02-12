#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  // Plotting
  setupPlot(ui->motorPlot1, motor_fb_plot_x_, motor_fb_plot_y_, "motor pos", plot_size_, plot_time_scale_);
  setupPlot(ui->sensorPlot1, sensor_fb_plot_x_, sensor_fb_plot_y_, "sensor pos", plot_size_, plot_time_scale_);

  // ROS
  p_publisher_node_ = new PublisherNode(&nh_);
  connect(this, SIGNAL(sigSendCmd(float)), p_publisher_node_, SLOT(slotPubCmd(float)));
  p_publisher_node_thread_ = new QThread();
  connect(p_publisher_node_, SIGNAL(finished()), p_publisher_node_thread_, SLOT(quit()));
  p_publisher_node_->moveToThread(p_publisher_node_thread_);

  p_subscriber_node_ = new SubscriberNode(&nh_);
  connect(this, SIGNAL(init()), p_subscriber_node_, SLOT(slotStartSubs()));
  p_subcriber_node_thread_ = new QThread();
  connect(p_subscriber_node_, SIGNAL(finished()), p_subcriber_node_thread_, SLOT(quit()));
  p_subscriber_node_->moveToThread(p_subcriber_node_thread_);
  connect(p_subscriber_node_, SIGNAL(sigMotorFb(float)), this, SLOT(slot_motorFb(float)));
  connect(p_subscriber_node_, SIGNAL(sigSensorFb(float)), this, SLOT(slot_sensorFb(float)));

  p_subcriber_node_thread_->start();
  p_publisher_node_thread_->start();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
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
  printStringTextBrowser("Command sent to motor. Value: " + inputStr + "\n");
  emit sigSendCmd(inputStr.toFloat());
}

void MainWindow::setupPlot(QCustomPlot *plot, QVector<double> &plot_x, QVector<double> &plot_y, QString y_axis_str, int plot_size, double time_scale) {
  plot_x = QVector<double>(plot_size, 0.0);
  plot_y = QVector<double>(plot_size, 0.0);
  plot->addGraph();
  plot->xAxis->setLabel("time (s)");
  plot->yAxis->setLabel(y_axis_str);
  plot->xAxis->setRange(0, plot_size * time_scale);
  plot->yAxis->setRange(0, plot_size);
  for (int i = 0; i < plot_size; ++i) {
    plot_x[i] = i * time_scale;
  }
}
