#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  start_time_ = std::chrono::system_clock::now();

  // Plotting
  setupPlot(ui->motorPlot1, motor1_fb_plot_x_, motor1_fb_plot_y_, "motor1 pos", plot_size_, plot_time_scale_, x_scale_);
  setupPlot(ui->sensorPlot1, sensor1_fb_plot_x_, sensor1_fb_plot_y_, "sensor1 pos", plot_size_, plot_time_scale_, x_scale_);
  setupPlot(ui->motorPlot2, motor2_fb_plot_x_, motor2_fb_plot_y_, "motor2 pos", plot_size_, plot_time_scale_, x_scale_);
  setupPlot(ui->sensorPlot2, sensor2_fb_plot_x_, sensor2_fb_plot_y_, "sensor2 pos", plot_size_, plot_time_scale_, x_scale_);
  setupPlot(ui->motorPlot3, motor3_fb_plot_x_, motor3_fb_plot_y_, "motor3 pos", plot_size_, plot_time_scale_, x_scale_);
  setupPlot(ui->sensorPlot3, sensor3_fb_plot_x_, sensor3_fb_plot_y_, "sensor3 pos", plot_size_, plot_time_scale_, x_scale_);

  // ROS
  p_publisher_node_ = new PublisherNode(&nh_);
  connect(this, SIGNAL(sigSendCmd1(float)), p_publisher_node_, SLOT(slotPubCmd1(float)));
  connect(this, SIGNAL(sigSendCmd2(float)), p_publisher_node_, SLOT(slotPubCmd2(float)));
  connect(this, SIGNAL(sigSendCmd3(float)), p_publisher_node_, SLOT(slotPubCmd3(float)));
  connect(p_publisher_node_, SIGNAL(finished()), &p_publisher_node_thread_, SLOT(quit()));
  p_publisher_node_->moveToThread(&p_publisher_node_thread_);

  p_subscriber_node_ = new SubscriberNode(&nh_);
  connect(this, SIGNAL(init()), p_subscriber_node_, SLOT(slotStartSubs()));
  connect(p_subscriber_node_, SIGNAL(sigMotor1Fb(float)), this, SLOT(slot_motor1Fb(float)));
  connect(p_subscriber_node_, SIGNAL(sigSensor1Fb(float)), this, SLOT(slot_sensor1Fb(float)));
  connect(p_subscriber_node_, SIGNAL(sigMotor2Fb(float)), this, SLOT(slot_motor2Fb(float)));
  connect(p_subscriber_node_, SIGNAL(sigSensor2Fb(float)), this, SLOT(slot_sensor2Fb(float)));
  connect(p_subscriber_node_, SIGNAL(sigMotor3Fb(float)), this, SLOT(slot_motor3Fb(float)));
  connect(p_subscriber_node_, SIGNAL(sigSensor3Fb(float)), this, SLOT(slot_sensor3Fb(float)));
  connect(p_subscriber_node_, SIGNAL(finished()), &p_subcriber_node_thread_, SLOT(quit()));
  p_subscriber_node_->moveToThread(&p_subcriber_node_thread_);

  p_subcriber_node_thread_.start();
  p_publisher_node_thread_.start();
  emit init();
}

void MainWindow::printStringTextBrowser(QString toPrintStr) {
  if (textBrowserStr_.size() > maxTextBrowserSize_) {
    textBrowserStr_.resize(maxTextBrowserSize_);
  }
  textBrowserStr_ = toPrintStr + textBrowserStr_;
  ui->textBrowser->setText(textBrowserStr_);
}

void MainWindow::setupPlot(QCustomPlot *plot, QVector<double> &plot_x, QVector<double> &plot_y, QString y_axis_str, int plot_size, double time_scale, double x_scale) {
  plot_x = QVector<double>(plot_size, 0.0);
  plot_y = QVector<double>(plot_size, 0.0);
  plot->addGraph();
  plot->xAxis->setLabel("time (s)");
  plot->yAxis->setLabel(y_axis_str);
  plot->xAxis->setRange(0, plot_size * time_scale);
  plot->yAxis->setRange(0, plot_size * x_scale);
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

void MainWindow::processCmdButton(QLineEdit* line_edit, QString motor_id, void (MainWindow::* sig)(float)) {
  QString inputStr = line_edit->text();
  float input;
  QString timeStr = QString::number((std::chrono::system_clock::now() - start_time_).count());

  bool conversion_ok;
  input = inputStr.toFloat(&conversion_ok);
  if (!conversion_ok) {
    QString errorStr = "[" + timeStr + "]: Error, not a number\n";
    printStringTextBrowser(errorStr);
    return;
  }

  if (input < 0.0 || input > 1.0) {
    QString errorStr = "[" + timeStr + "]: Value out of bound, insert number within 0 and 1\n";
    printStringTextBrowser(errorStr);
    return;
  }

  QString toPrintStr = "[" + timeStr + "]: Command sent to motor " + motor_id + " with value: " + inputStr + "\n";
  printStringTextBrowser(toPrintStr);
  emit (this->*sig)(input);
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
  processCmdButton(ui->cmd1LineEdit, "1", &MainWindow::sigSendCmd1);
}

void MainWindow::on_sendCmd2Button_clicked()
{
  processCmdButton(ui->cmd2LineEdit, "2", &MainWindow::sigSendCmd2);
}

void MainWindow::on_sendCmd3Button_clicked()
{
  processCmdButton(ui->cmd3LineEdit, "3", &MainWindow::sigSendCmd3);
}

MainWindow::~MainWindow() {
  delete ui;
  delete p_publisher_node_;
  delete p_subscriber_node_;
}
