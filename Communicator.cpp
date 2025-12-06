#include "Communicator.h"
#include <QFileInfo>
#include <QDebug>

/**
 * @brief 构造函数实现
 * @param parent 父对象
 */
Communicator::Communicator(QObject *parent)
    : QObject(parent)
    , m_currentType(CommunicationType::File) // 默认初始化为文件模式（未启动）
{
    // 初始化文件读取定时器
    m_fileReadTimer = new QTimer(this);
    m_fileReadTimer->setSingleShot(false); // 重复触发模式
    connect(m_fileReadTimer, &QTimer::timeout, this, &Communicator::onFileReadTimerTimeout);
}

/**
 * @brief 析构函数实现
 * @details 停止通讯并释放所有资源
 */
Communicator::~Communicator()
{
    stopCommunication();
}

/**
 * @brief 启动通讯实现
 * @param type 通讯类型
 * @param config 配置参数
 * @return 启动结果
 */
bool Communicator::startCommunication(CommunicationType type, const Config &config)
{
    // 先停止当前通讯（如果正在运行）
    if (m_isRunning) {
        stopCommunication();
    }

    // 保存当前配置和类型
    m_currentConfig = config;
    m_currentType = type;

    // 根据类型初始化对应通讯方式
    bool initSuccess = false;
    switch (type) {
    case CommunicationType::File:
        initSuccess = initFileCommunication(config);
        break;
    case CommunicationType::TcpClient:
        initSuccess = initTcpClientCommunication(config);
        break;
    case CommunicationType::SerialPort:
        initSuccess = initSerialPortCommunication(config);
        break;
    default:
        emit communicateRecoder("不支持的通讯类型");
        return false;
    }

    // 更新运行状态并发送信号
    m_isRunning = initSuccess;
    emit stateChanged(m_isRunning);
    if (!initSuccess) {
        emit communicateRecoder(QString("启动%1模式失败").arg(QString::fromStdString(std::to_string(static_cast<int>(type)))));
    }

    return initSuccess;
}

/**
 * @brief 停止通讯实现
 */
void Communicator::stopCommunication()
{
    if (!m_isRunning) {
        return;
    }

    // 停止所有定时器
    if (m_fileReadTimer) {
        m_fileReadTimer->stop();
    }

    // 释放所有通讯资源
    releaseAllResources();

    // 更新状态并发送信号
    m_isRunning = false;
    emit stateChanged(false);
    emit communicateRecoder("通讯已停止");
}

/**
 * @brief 文件模式初始化
 * @param config 配置参数
 * @return 初始化结果
 */
bool Communicator::initFileCommunication(const Config &config)
{
    emit communicateRecoder("初始化文件通讯");
    // 检查文件路径是否有效
    if (config.filePath.isEmpty()) {
        emit communicateRecoder("文件路径为空");
        return false;
    }

    // 创建文件对象
    m_file = new QFile(config.filePath, this);
    if (!m_file->open(QIODevice::ReadOnly)) {
        emit communicateRecoder(QString("文件打开失败：%1").arg(m_file->errorString()));
        delete m_file;
        m_file = nullptr;
        return false;
    }

    // 启动文件读取定时器
    m_fileReadTimer->setInterval(config.readInterval);
    m_fileReadTimer->start();

    emit communicateRecoder(QString("文件模式启动成功，路径：%1").arg(config.filePath));
    return true;
}

/**
 * @brief TCP客户端模式初始化
 * @param config 配置参数
 * @return 初始化结果
 */
bool Communicator::initTcpClientCommunication(const Config &config)
{
    emit communicateRecoder("初始化Tcp通讯");
    // 创建TCP客户端套接字
    m_tcpSocket = new QTcpSocket(this);

    // 绑定信号槽
    connect(
            m_tcpSocket,
            &QTcpSocket::connected,
            this,
            &Communicator::onTcpClientConnected);
    connect(
            m_tcpSocket,
            SIGNAL(error(QAbstractSocket::SocketError)),
            this,
            SLOT(onTcpClientError(QAbstractSocket::SocketError)));
    connect(
            m_tcpSocket,
            &QTcpSocket::readyRead,
            this,
            &Communicator::onTcpDataReady);
    connect(
            m_tcpSocket,
            &QTcpSocket::disconnected,
            this,
            [this]() {
                emit communicateRecoder("TCP客户端已断开连接");
                stopCommunication();});

    // 连接到TCP服务器
    m_tcpSocket->connectToHost(config.tcpIp, config.tcpPort);

    // 等待连接（非阻塞，实际连接结果由onTcpClientConnected/onTcpClientError处理）
    return true; // 连接请求已发送，实际结果异步反馈
}


/**
 * @brief 串口模式初始化
 * @param config 配置参数
 * @return 初始化结果
 */
bool Communicator::initSerialPortCommunication(const Config &config)
{
    emit communicateRecoder("初始化串口通讯");
    // 检查串口号是否有效
    if (config.serialPortName.isEmpty()) {
        emit communicateRecoder("串口号为空");
        return false;
    }

    // 创建串口对象
    m_serialPort = new QSerialPort(this);
    m_serialPort->setPortName(config.serialPortName);
    m_serialPort->setBaudRate(config.baudRate);
    m_serialPort->setDataBits(config.dataBits);
    m_serialPort->setParity(config.parity);
    m_serialPort->setStopBits(config.stopBits);
    m_serialPort->setFlowControl(config.flowControl);

    // 打开串口（读写模式）
    if (!m_serialPort->open(QIODevice::ReadWrite)) {
        emit communicateRecoder(QString("串口打开失败：%1").arg(m_serialPort->errorString()));
        delete m_serialPort;
        m_serialPort = nullptr;
        return false;
    }

    // 绑定信号槽
    connect(m_serialPort, &QSerialPort::readyRead, this, &Communicator::onSerialPortDataReady);
    connect(m_serialPort, QOverload<QSerialPort::SerialPortError>::of(&QSerialPort::errorOccurred),
            this, &Communicator::onSerialPortError);

    emit communicateRecoder(QString("串口启动成功，端口：%1，波特率：%2").arg(config.serialPortName).arg(config.baudRate));
    return true;
}

/**
 * @brief 释放所有通讯资源
 */
void Communicator::releaseAllResources()
{
    emit communicateRecoder("释放所有资源");
    // 释放文件资源
    if (m_file) {
        m_file->close();
        m_file->deleteLater();
        m_file = nullptr;
    }

    // 释放TCP资源
    if (m_tcpSocket) {
        m_tcpSocket->disconnectFromHost();
        m_tcpSocket->deleteLater();
        m_tcpSocket = nullptr;
    }

    // 释放串口资源
    if (m_serialPort) {
        m_serialPort->close();
        m_serialPort->deleteLater();
        m_serialPort = nullptr;
    }
}

// ========== 槽函数实现 ==========

/**
 * @brief 文件读取定时器超时槽函数
 */
void Communicator::onFileReadTimerTimeout()
{
    if (!m_file || !m_file->isOpen()) {
        stopCommunication();
        return;
    }

    // 读取指定大小的字节数据
    QByteArray rawData = m_file->read(m_currentConfig.readBlockSize);
    if (rawData.isEmpty()) {
        // 读取到文件末尾
        if (m_file->atEnd()) {
            emit communicateRecoder("文件读取完毕");
            stopCommunication();
        } else {
            emit communicateRecoder(QString("文件读取失败：%1").arg(m_file->errorString()));
        }
        return;
    }

    // 发送读取到的原始数据
    emit dataReady(rawData);
}

/**
 * @brief TCP客户端连接成功槽函数
 */
void Communicator::onTcpClientConnected()
{
    emit communicateRecoder(QString("TCP客户端连接成功：%1:%2")
                       .arg(m_currentConfig.tcpIp).arg(m_currentConfig.tcpPort));
}

/**
 * @brief TCP客户端错误槽函数
 * @param socketError 错误码
 */
void Communicator::onTcpClientError(QAbstractSocket::SocketError socketError)
{
    Q_UNUSED(socketError)
    emit communicateRecoder(QString("TCP客户端错误：%1").arg(m_tcpSocket->errorString()));
    stopCommunication();
}

/**
 * @brief TCP数据就绪槽函数
 */
void Communicator::onTcpDataReady()
{
    if (!m_tcpSocket || !m_tcpSocket->isOpen()) {
        return;
    }

    // 读取所有可用数据
    QByteArray rawData = m_tcpSocket->readAll();
    if (!rawData.isEmpty()) {
        emit dataReady(rawData);
    }
}


/**
 * @brief TCP客户端断开连接槽函数
 */
void Communicator::onTcpClientDisconnected()
{
    emit communicateRecoder("TCP客户端已断开连接");
    if (m_tcpSocket) {
        m_tcpSocket->deleteLater();
        m_tcpSocket = nullptr;
    }
}

/**
 * @brief 串口数据就绪槽函数
 */
void Communicator::onSerialPortDataReady()
{
    if (!m_serialPort || !m_serialPort->isOpen()) {
        return;
    }

    // 读取所有可用串口数据
    QByteArray rawData = m_serialPort->readAll();
    if (!rawData.isEmpty()) {
        emit dataReady(rawData);
    }
}

/**
 * @brief 串口错误槽函数
 * @param error 错误码
 */
void Communicator::onSerialPortError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::NoError) {
        return;
    }

    emit communicateRecoder(QString("串口错误：%1").arg(m_serialPort->errorString()));
    stopCommunication();
}
