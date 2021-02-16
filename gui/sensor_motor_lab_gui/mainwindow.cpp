#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  start_time_ = std::chrono::system_clock::now();

  // Plotting
  setupPlot(ui->motorPlot1, motor1_fb_plot_x_, motor1_fb_plot_y_, "motor1 pos", plot_size_, plot_time_scale_, y_scale_);
  setupPlot(ui->sensorPlot1, sensor1_fb_plot_x_, sensor1_fb_plot_y_, "sensor1 pos", plot_size_, plot_time_scale_, y_scale_);

  // ROS
  p_publisher_node_ = new PublisherNode(&nh_);
  connect(this, SIGNAL(sigSendCmd1(float)), p_publisher_node_, SLOT(slotPubCmd1(float)));
  connect(this, SIGNAL(sigSendMotorSelection(int)), p_publisher_node_, SLOT(slotPubMotorSelection(int)));
  connect(p_publisher_node_, SIGNAL(finished()), &p_publisher_node_thread_, SLOT(quit()));
  p_publisher_node_->moveToThread(&p_publisher_node_thread_);

  p_subscriber_node_ = new SubscriberNode(&nh_);
  connect(this, SIGNAL(init()), p_subscriber_node_, SLOT(slotStartSubs()));
  connect(p_subscriber_node_, SIGNAL(sigMotor1Fb(float)), this, SLOT(slot_motor1Fb(float)));
  connect(p_subscriber_node_, SIGNAL(sigSensor1Fb(float)), this, SLOT(slot_sensor1Fb(float)));
  connect(p_subscriber_node_, SIGNAL(finished()), &p_subcriber_node_thread_, SLOT(quit()));
  p_subscriber_node_->moveToThread(&p_subcriber_node_thread_);

  // Start processes
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

void MainWindow::setupPlot(QCustomPlot *plot, QVector<double> &plot_x, QVector<double> &plot_y, QString y_axis_str, int plot_size, double time_scale, double y_scale) {
  plot_x = QVector<double>(plot_size, 0.0);
  plot_y = QVector<double>(plot_size, 0.0);
  plot->addGraph();
  plot->xAxis->setLabel("time (s)");
  plot->yAxis->setLabel(y_axis_str);
  plot->xAxis->setRange(0, plot_size * time_scale);
  plot->yAxis->setRange(0, plot_size * y_scale);
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

void MainWindow::processRadioButton(QString motor_selection_str, int motor_selection, void (MainWindow:: *sig)(int)) {
  curr_motor_selection = motor_selection;
  QString timeStr = QString::number((std::chrono::system_clock::now() - start_time_).count());
  QString toPrintStr = "[" + timeStr + "]: Motor " + motor_selection_str + " selected\n";
  printStringTextBrowser(toPrintStr);
  emit (this->*sig)(motor_selection);

  switch (motor_selection) {
    case motorSelection::motor1:
      setupPlot(ui->motorPlot1, motor1_fb_plot_x_, motor1_fb_plot_y_, "motor1 pos", plot_size_, plot_time_scale_, 0.01);
      setupPlot(ui->sensorPlot1, sensor1_fb_plot_x_, sensor1_fb_plot_y_, "sensor1 pos", plot_size_, plot_time_scale_, 0.01);
      break;
    case motorSelection::motor2:
      setupPlot(ui->motorPlot1, motor1_fb_plot_x_, motor1_fb_plot_y_, "motor2 pos", plot_size_, plot_time_scale_, 0.1);
      setupPlot(ui->sensorPlot1, sensor1_fb_plot_x_, sensor1_fb_plot_y_, "sensor2 pos", plot_size_, plot_time_scale_, 0.1);
      break;
    case motorSelection::motor3:
      setupPlot(ui->motorPlot1, motor1_fb_plot_x_, motor1_fb_plot_y_, "motor3 pos", plot_size_, plot_time_scale_, 1);
      setupPlot(ui->sensorPlot1, sensor1_fb_plot_x_, sensor1_fb_plot_y_, "sensor3 pos", plot_size_, plot_time_scale_, 1);
      break;
  }
}


void MainWindow::slot_motor1Fb(float sig) {
  drawPlot(ui->motorPlot1, motor1_fb_plot_x_, motor1_fb_plot_y_, sig);
}

void MainWindow::slot_sensor1Fb(float sig) {
  drawPlot(ui->sensorPlot1, sensor1_fb_plot_y_, sensor1_fb_plot_y_, sig);
}

void MainWindow::on_sendCmd1Button_clicked()
{
  processCmdButton(ui->cmd1LineEdit, QString::number(curr_motor_selection+1), &MainWindow::sigSendCmd1);
}

void MainWindow::on_motor1RadioButton_clicked()
{
  processRadioButton("1", motorSelection::motor1, &MainWindow::sigSendMotorSelection);
}

void MainWindow::on_motor2RadioButton_clicked()
{
  processRadioButton("2", motorSelection::motor2, &MainWindow::sigSendMotorSelection);
}

void MainWindow::on_motor3RadioButton_clicked()
{
  processRadioButton("3", motorSelection::motor3, &MainWindow::sigSendMotorSelection);
}

MainWindow::~MainWindow() {
  delete p_publisher_node_;
  delete p_subscriber_node_;
  delete ui;
}
