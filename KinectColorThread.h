#pragma once
#ifndef KINECT_COLOR_THREAD_H
#define KINECT_COLOR_THREAD_H

#include <QThread>
#include <QMutex>
#include <iostream>
#include <Kinect.h>

using namespace std;

class KinectColorThread : public QThread
{
    Q_OBJECT
public:
    KinectColorThread(QObject* parent, IKinectSensor* iKinectSensor) :QThread(parent), m_stop(false) {
        pSensor = iKinectSensor;
    }
    // 其他函数和变量
    void run() override;
    string generateFilenameBasedOnUTC();
    void stop() { m_stop.store(1); };
    void stopKinect();

signals:
    // 定义信号，例如当新数据可用时
    void colorFrameReady(const QPixmap& image);




private:
    // 其他私有成员变量
    QAtomicInteger<int> m_stop;
    int WIDTH = 1920;
    int HEIGHT = 1080;
    int BUFFERSIZE = WIDTH * HEIGHT * 4;
    IKinectSensor* pSensor =nullptr;
};





#endif // KINECT_COLOR_THREAD_H
