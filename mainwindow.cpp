#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)  // 初始化UI对象
    , m_communicator(new Communicator(this))
{
    // 加载Qt Designer设计的UI
    ui->setupUi(this);

    // ========== 初始化控件 ==========
    // 填充串口列表（Windows）
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
        ui->cbx_SerialPort->addItem(info.portName());
    }

    // 初始化按钮状态
    ui->btn_Start->setEnabled(true);
    ui->btn_Stop->setEnabled(false);

    // ========== 绑定信号槽 ==========
    // 按钮点击信号
    connect(ui->btn_Start, &QPushButton::clicked, this, &MainWindow::onStartBtnClicked);
    connect(ui->btn_Stop, &QPushButton::clicked, this, &MainWindow::onStopBtnClicked);
    connect(ui->btn_BrowseFile, &QPushButton::clicked, this, &MainWindow::onBrowseFileBtnClicked);

    // 通讯器信号
    connect(m_communicator, &Communicator::dataReady, this, &MainWindow::onDataReady);
    connect(m_communicator, &Communicator::communicateRecoder, this, &MainWindow::onCommunicateRecoder);
    connect(m_communicator, &Communicator::stateChanged, this, &MainWindow::onStateChanged);
}

MainWindow::~MainWindow()
{
    delete ui;  // 释放UI对象
}

// ========== 按钮点击槽函数 ==========
void MainWindow::onStartBtnClicked()
{
    // 构建配置
    Communicator::Config config = buildConfig();

    // 确定通讯类型
    Communicator::CommunicationType type;
    if (ui->radioButton_file->isChecked()) {
        type = Communicator::CommunicationType::File;
    } else if (ui->radioButton_Tcp->isChecked()) {
        type = Communicator::CommunicationType::TcpClient;
    } else {
        type = Communicator::CommunicationType::SerialPort;
    }

    // 启动通讯
    m_communicator->startCommunication(type, config);
}

void MainWindow::onStopBtnClicked()
{
    m_communicator->stopCommunication();
}

void MainWindow::onBrowseFileBtnClicked()
{
    QString filePath = QFileDialog::getOpenFileName(this, "选择B2b卫星数据文件", "", "所有文件 (*.*);;二进制文件 (*.bin)");
    if (!filePath.isEmpty()) {
        ui->editFilePath->setText(filePath);
    }
}

// ========== 通讯器信号槽函数 ==========
void MainWindow::onDataReady(const QByteArray &rawData)
{
    // 显示十六进制数据
    QString hexData = rawData.toHex(' ').toUpper();
    ui->te_HexData->append(QString("[%1] 接收数据：%2 (长度：%3字节)")
                          .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"))
                          .arg(hexData)
                          .arg(rawData.size()));

    // 自动滚动到底部
    QScrollBar *scroll = ui->te_HexData->verticalScrollBar();
    scroll->setValue(scroll->maximum());
}

void MainWindow::onCommunicateRecoder(const QString &comMsg)
{
    ui->te_Log->append(QString("[%1] 【Communicator】%2")
                      .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"))
                      .arg(comMsg));

    // 自动滚动到底部
    QScrollBar *scroll = ui->te_Log->verticalScrollBar();
    scroll->setValue(scroll->maximum());
}

void MainWindow::onStateChanged(bool isRunning)
{
    // 更新按钮状态
    ui->btn_Start->setEnabled(!isRunning);
    ui->btn_Stop->setEnabled(isRunning);

}

// ========== 构建配置参数 ==========
Communicator::Config MainWindow::buildConfig()
{
    Communicator::Config config;

    // 文件模式配置
    config.filePath = ui->editFilePath->text();
    config.readBlockSize = ui->spinBlockSize->value();
    config.readInterval = ui->spinReadInterval->value();

    // TCP客户端配置
    config.tcpIp = ui->editIcpIp->text();
    config.tcpPort = ui->spinIcpPort->value();

    // 串口配置
    config.serialPortName = ui->cbx_SerialPort->currentText();
    config.baudRate = ui->cbx_BaudRate->currentText().toInt();
    config.dataBits = QSerialPort::Data8;
    config.parity = QSerialPort::NoParity;
    config.stopBits = QSerialPort::OneStop;
    config.flowControl = QSerialPort::NoFlowControl;

    return config;
}
