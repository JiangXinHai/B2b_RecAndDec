#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPortInfo>
#include <QDateTime>
#include <QScrollBar>
#include <QFileDialog>

// 引入Qt Designer生成的UI头文件
#include "ui_mainwindow.h"

#include "Communicator.h"

#include "Receiver.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

private slots:
    // 按钮点击槽函数
    void onStartBtnClicked();
    void onStopBtnClicked();
    void onBrowseFileBtnClicked();
    void onClearLogBtnClicked();
    void onClearDataBtnClicked();

    // 通讯器信号槽函数
    void onDataReady(const QByteArray &rawData);
    void onCommunicateLog(const QString &logMsg);
    void onStateChanged(bool isRunning);

    // 接收器信号槽函数
    void onFrameReady(const QByteArray &rawFrame, uint8_t prn, uint8_t msgType);
    void onReceiveLog(const QString &logMsg);

private:
    // 构建配置参数（根据选中的通讯类型）
    Communicator::Config buildConfig();

    // Qt Designer生成的UI对象（核心）
    Ui::MainWindow *ui;

    // 通讯器核心对象
    Communicator *m_communicator;

    // 接收器核心对象
    Receiver *m_receiver;
};

#endif // MAINWINDOW_H
