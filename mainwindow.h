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

    // 通讯器信号槽函数
    void onDataReady(const QByteArray &rawData);
    void onErrorOccurred(const QString &errorMsg);
    void onStateChanged(bool isRunning);

private:
    // 构建配置参数（根据选中的通讯类型）
    Communicator::Config buildConfig();

    // Qt Designer生成的UI对象（核心）
    Ui::MainWindow *ui;

    // 通讯器核心对象
    Communicator *m_communicator;
};

#endif // MAINWINDOW_H
