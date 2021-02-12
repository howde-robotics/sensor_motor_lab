#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  start_time_ = std::chrono::system_clock::now();

  // Plotting
  setupPlot(ui->motorPlot1, motor1_fb_plot_x_, motor1_fb_plot_y_, "motor1 pos", plot_size_, plot_time_scale_);
  setupPlot(ui->sensorPlot1, sensor1_fb_plot_x_, sensor1_fb_plot_y_, "sensor1 pos", plot_size_, plot_time_scale_);
  setupPlot(ui->motorPlot2, motor2_fb_plot_x_, motor2_fb_plot_y_, "motor2 pos", plot_size_, plot_time_scale_);
  setupPlot(ui->sensorPlot2, sensor2_fb_plot_x_, sensor2_fb_plot_y_, "sensor2 pos", plot_size_, plot_time_scale_);
  setupPlot(ui->motorPlot3, motor3_fb_plot_x_, motor3_fb_plot_y_, "motor3 pos", plot_size_, plot_time_scale_);
  setupPlot(ui->sensorPlot3, sensor3_fb_plot_x_, sensor3_fb_plot_y_, "sensor3 pos", plot_size_, plot_time_scale_);

  // ROS
  p_publisher_node_ = new PublisherNode(&nh_);
  connect(this, SIGNAL(sigSendCmd1(float)), p_publisher_node_, SLOT(slotPubCmd1(float)));
  p_publisher_node_thread_ = new QThread();
  connect(p_publisher_node_, SIGNAL(finished()), p_publisher_node_thread_, SLOT(quit()));
  p_publisher_node_->moveToThread(p_publisher_node_thread_);

  p_subscriber_node_ = new SubscriberNode(&nh_);
  connect(this, SIGNAL(init()), p_subscriber_node_, SLOT(slotStartSubs()));
  p_subcriber_node_thread_ = new QThread();
  connect(p_subscriber_node_, SIGNAL(finished()), p_subcriber_node_thread_, SLOT(quit()));
  p_subscriber_node_->moveToThread(p_subcriber_node_thread_);
  connect(p_subscriber_node_, SIGNAL(sigMotor1Fb(float)), this, SLOT(slot_motor1Fb(float)));
  connect(p_subscriber_node_, SIGNAL(sigSensor1Fb(float)), this, SLOT(slot_sensor1Fb(float)));
  connect(p_subscriber_node_, SIGNAL(sigMotor2Fb(float)), this, SLOT(slot_motor2Fb(float)));
  connect(p_subscriber_node_, SIGNAL(sigSensor2Fb(float)), this, SLOT(slot_sensor2Fb(float)));
  connect(p_subscriber_node_, SIGNAL(sigMotor3Fb(float)), this, SLOT(slot_motor3Fb(float)));
  connect(p_subscriber_node_, SIGNAL(sigSensor3Fb(float)), this, SLOT(slot_sensor3Fb(float)));


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

void MainWindow::drawPlot(QCustomPlot* plot, QVector<double> &plot_x, QVector<double> &plot_y, float sig) {
  plot_y.pop_front();
  plot_y.push_back(double(sig));
  plot->graph(0)->setData(plot_x, plot_y);
  plot->replot();
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
}

void MainWindow::slot_motor1Fb(float sig) {
  drawPlot(ui->motorPlot1, motor1_fb_plot_x_, motor1_fb_plot_y_, sig);
}

void MainWindow::slot_sensor1Fb(float sig) {
  drawPlot(ui->sensorPlot1, sensor1_fb_plot_y_, sensor1_fb_plot_y_, sig);
}

void MainWindow::slot_motor2Fb(float sig) {
  drawPlot(ui->motorPlot2, motor2_fb_plot_x_, motor2_fb_plot_y_, sig);
}

void MainWindow::slot_sensor2Fb(float sig) {
  drawPlot(ui->sensorPlot2, sensor2_fb_plot_x_, sensor2_fb_plot_y_, sig);
}

void MainWindow::slot_motor3Fb(float sig) {
  drawPlot(ui->motorPlot3, motor3_fb_plot_x_, motor3_fb_plot_y_, sig);
}

void MainWindow::slot_sensor3Fb(float sig) {
  drawPlot(ui->sensorPlot3, sensor3_fb_plot_x_, sensor3_fb_plot_y_, sig);
}

void MainWindow::on_sendCmd1Button_clicked()
{
  QString inputStr = ui->cmd1LineEdit->text();
  QString timeStr = QString::number((std::chrono::system_clock::now() - start_time_).count());
  QString toPrintStr = "[" + timeStr + "]: Command sent to motor 1 with Value: " + inputStr + "\n";
  textBrowserStr_ = toPrintStr + textBrowserStr_;
  printStringTextBrowser(textBrowserStr_);
  emit sigSendCmd1(inputStr.toFloat());
}
