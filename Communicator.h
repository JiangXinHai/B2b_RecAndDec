#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

#include <QObject>
#include <QFile>
#include <QTcpSocket>
#include <QTcpServer>
#include <QSerialPort>
#include <QTimer>
#include <QByteArray>

/**
 * @class Communicator
 * @brief 通讯层核心类，统一封装文件、TCP服务器（客户端模式）、串口三种种数据输入方式
 * @details 对外提供统一的启动/停止接口，内部根据配置适配不同通讯方式，
 *          原始数据通过dataReady信号对外发送，通讯层日志通过comMsg信号反馈
 * @author 江鑫海
 * @date 2025-12-05
 */
class Communicator : public QObject
{
    Q_OBJECT
public:
    /**
     * @enum CommunicationType
     * @brief 通讯类型枚举，定义支持的数据源类型
     */
    enum class CommunicationType {
        File,           // 文件读取模式
        TcpClient,      // TCP客户端模式
        SerialPort      // 串口通信模式
    };
    Q_ENUM(CommunicationType)  // 注册枚举，支持QT元对象系统

    /**
     * @struct Config
     * @brief 通讯配置结构体，存储不同通讯方式的配置参数
     * @details 不同通讯类型仅使用对应字段，未使用字段不影响功能
     */
    struct Config {
        // 文件模式配置
        QString filePath;        // 文件路径
        int readBlockSize = 1024;// 每次读取字节数（默认1024）
        int readInterval = 100;  // 读取间隔（ms，模拟实时流，默认100）

        // TCP模式配置
        QString tcpIp = "127.0.0.1"; // TCP服务器IP（客户端模式）
        quint16 tcpPort = 8888;      // TCP端口（默认8888）

        // 串口模式配置
        QString serialPortName;      // 串口号（如"COM3"）
        qint32 baudRate = 9600;      // 波特率（默认9600）
        QSerialPort::DataBits dataBits = QSerialPort::Data8; // 数据位（默认8位）
        QSerialPort::Parity parity = QSerialPort::NoParity; // 校验位（默认无）
        QSerialPort::StopBits stopBits = QSerialPort::OneStop; // 停止位（默认1位）
        QSerialPort::FlowControl flowControl = QSerialPort::NoFlowControl; // 流控（默认无）
    };

    /**
     * @brief 构造函数
     * @param parent 父对象，用于QT父子对象内存管理
     */
    explicit Communicator(QObject *parent = nullptr);

    /**
     * @brief 析构函数
     * @details 确保所有通讯资源被正确释放
     */
    ~Communicator() override;

    /**
     * @brief 启动通讯
     * @param type 通讯类型（File/TcpClient/SerialPort）
     * @param config 对应类型的配置参数
     * @return bool 启动成功返回true，失败返回false
     */
    bool startCommunication(CommunicationType type, const Config &config);

    /**
     * @brief 停止通讯
     * @details 停止数据读取/监听，释放对应通讯资源
     */
    void stopCommunication();

    /**
     * @brief 获取当前通讯类型
     * @return CommunicationType 当前激活的通讯类型，未启动返回无效值
     */
    CommunicationType currentType() const { return m_currentType; }

signals:
    /**
     * @brief 原始数据就绪信号
     * @param rawData 读取到的原始字节数据
     * @details 所有通讯方式读取到数据后均触发此信号，对外提供统一数据接口
     */
    void dataReady(const QByteArray &rawData);

    /**
     * @brief 通讯器日志信号
     * @param comMsg 错误描述信息
     * @details 通讯过程中所有日志均通过此信号对外反馈
     */
    void communicateRecoder(const QString &comMsg);

    /**
     * @brief 通讯状态变化信号
     * @param isRunning true=通讯中，false=已停止
     */
    void stateChanged(bool isRunning);

private slots:
    // ========== 文件模式槽函数 ==========
    /**
     * @brief 定时读取文件数据的槽函数
     * @details 由m_fileReadTimer触发，模拟实时数据流读取
     */
    void onFileReadTimerTimeout();

    // ========== TCP模式槽函数 ==========
    /**
     * @brief TCP客户端连接成功槽函数
     * @details 客户端模式下，连接到服务器后触发
     */
    void onTcpClientConnected();

    /**
     * @brief TCP客户端连接失败槽函数
     * @details 客户端模式下，连接服务器失败时触发
     */
    void onTcpClientError(QAbstractSocket::SocketError socketError);

    /**
     * @brief TCP数据就绪槽函数
     * @details 客户端模式下，有数据可读时触发
     */
    void onTcpDataReady();

    /**
     * @brief TCP客户端断开连接槽函数
     * @details 客户端模式下，已连接的客户端断开时触发
     */
    void onTcpClientDisconnected();

    // ========== 串口模式槽函数 ==========
    /**
     * @brief 串口数据就绪槽函数
     * @details 串口模式下，有数据可读时触发
     */
    void onSerialPortDataReady();

    /**
     * @brief 串口错误槽函数
     * @param error 串口错误码
     */
    void onSerialPortError(QSerialPort::SerialPortError error);

private:
    /**
     * @brief 初始化文件通讯资源
     * @param config 文件配置参数
     * @return bool 初始化成功返回true，失败返回false
     */
    bool initFileCommunication(const Config &config);

    /**
     * @brief 初始化TCP客户端通讯资源
     * @param config TCP配置参数
     * @return bool 初始化成功返回true，失败返回false
     */
    bool initTcpClientCommunication(const Config &config);

    /**
     * @brief 初始化串口通讯资源
     * @param config 串口配置参数
     * @return bool 初始化成功返回true，失败返回false
     */
    bool initSerialPortCommunication(const Config &config);

    /**
     * @brief 释放所有通讯资源
     * @details 重置所有成员变量，关闭并删除通讯对象
     */
    void releaseAllResources();

    // 核心成员变量
    CommunicationType m_currentType;  // 当前通讯类型
    Config m_currentConfig;           // 当前通讯配置
    bool m_isRunning = false;         // 通讯是否正在运行

    // 文件模式成员
    QFile *m_file = nullptr;          // 文件对象
    QTimer *m_fileReadTimer = nullptr;// 文件读取定时器（模拟实时流）

    // TCP模式成员
    QTcpSocket *m_tcpSocket = nullptr;    // TCP套接字（客户端/已连接的客户端）

    // 串口模式成员
    QSerialPort *m_serialPort = nullptr;  // 串口对象
};

#endif // COMMUNICATOR_H
