#include "mainwindow.h"

//已完成通讯层

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)  // 初始化UI对象
    , m_communicator(new Communicator(this))
    , m_receiver(new Receiver(this))
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
    connect(ui->btn_ClearLog, &QPushButton::clicked, this, &MainWindow::onClearLogBtnClicked);
    connect(ui->btn_ClearData, &QPushButton::clicked, this, &MainWindow::onClearDataBtnClicked);

    // 通讯器信号
    connect(m_communicator, &Communicator::dataReady, this, &MainWindow::onDataReady);
    connect(m_communicator, &Communicator::communicateLog, this, &MainWindow::onCommunicateLog);
    connect(m_communicator, &Communicator::stateChanged, this, &MainWindow::onStateChanged);

    // 接收器信号
    connect(m_receiver, &Receiver::frameReady, this, &MainWindow::onFrameReady);
    connect(m_receiver, &Receiver::receiveLog, this, &MainWindow::onReceiveLog);
    m_receiver->setDemodParam (1207.14e6, 20.46e6, 8);
    m_receiver->init();
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

void MainWindow::onClearLogBtnClicked (){
    ui->te_Log->clear ();
}

void MainWindow::onClearDataBtnClicked (){
    ui->te_Data->clear ();
}

// ========== 通讯器信号槽函数 ==========
void MainWindow::onDataReady(const QByteArray &rawData)
{
    // 1. 原始数据传递给接收层
    m_receiver->onRawDataReceived (rawData);

    // 2. 限制单条数据显示长度（比如仅显示前100字节）
    const int MAX_SHOW_BYTES = 10; // 可根据需求调整
    QByteArray showData = rawData.left(MAX_SHOW_BYTES);
    QString hexData = showData.toHex(' ').toUpper();
    // 若原始数据超长，添加省略标记
    if (rawData.size() > MAX_SHOW_BYTES) {
        hexData += " ... (剩余 " + QString::number(rawData.size() - MAX_SHOW_BYTES) + " 字节)";
    }

    // 3. 格式化显示（仅显示截断后的数据）
    ui->te_Data->append(QString("[%1] 原始数据 | 长度：%2字节 | 16进制：%3 ")
                          .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"))
                          .arg(rawData.size())
                          .arg(hexData));

    // 自动滚动到底部
//    QScrollBar *scroll = ui->te_Data->verticalScrollBar();
//    scroll->setValue(scroll->maximum());
}

void MainWindow::onCommunicateLog(const QString &logMsg)
{
    ui->te_Log->append(QString("[%1] 【Communicator】%2")
                      .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"))
                      .arg(logMsg));

    // 自动滚动到底部
//    QScrollBar *scroll = ui->te_Log->verticalScrollBar();
//    scroll->setValue(scroll->maximum());
}

void MainWindow::onStateChanged(bool isRunning)
{
    // 更新按钮状态
    ui->btn_Start->setEnabled(!isRunning);
    ui->btn_Stop->setEnabled(isRunning);

}

// ========== 接收器信号槽函数 ============
void MainWindow::onFrameReady(const QByteArray &rawFrame, uint8_t prn, uint8_t msgType) {
    // 结构化展示完整帧信息（直观易读）
        QString frameHex = rawFrame.toHex(' ').toUpper();
        QString msgTypeDesc;
        // 按GB/T 39414.5-2024解析信息类型含义
        switch (msgType) {
        case 10: msgTypeDesc = "星历参数"; break;
        case 30: msgTypeDesc = "钟差/电离层参数"; break;
        case 40: msgTypeDesc = "历书/BGTO参数"; break;
        default: msgTypeDesc = "未知类型(" + QString::number(msgType) + ")";
        }

        ui->te_Data->append(QString(
            "=====================================\n"
            "[%1] 解析到有效B2b帧\n"
            "PRN号：%2 | 信息类型：%3 (%4)\n"
            "帧长度：%5字节 | 帧数据（16进制）：\n%6\n"
            "=====================================")
            .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"))
            .arg(prn)
            .arg(msgType)
            .arg(msgTypeDesc)
            .arg(rawFrame.size())
            .arg(frameHex));

        // 自动滚动到底部
//        QScrollBar *scroll = ui->te_Data->verticalScrollBar();
//        scroll->setValue(scroll->maximum());
}

void MainWindow::onReceiveLog(const QString &logMsg)
{
    ui->te_Log->append(QString("[%1] 【Receiver】%2")
                      .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"))
                      .arg(logMsg));

    // 自动滚动到底部
//    QScrollBar *scroll = ui->te_Log->verticalScrollBar();
//    scroll->setValue(scroll->maximum());
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
