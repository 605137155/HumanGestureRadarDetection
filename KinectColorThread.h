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
    // ���������ͱ���
    void run() override;
    string generateFilenameBasedOnUTC();
    void stop() { m_stop.store(1); };
    void stopKinect();

signals:
    // �����źţ����統�����ݿ���ʱ
    void colorFrameReady(const QPixmap& image);




private:
    // ����˽�г�Ա����
    QAtomicInteger<int> m_stop;
    int WIDTH = 1920;
    int HEIGHT = 1080;
    int BUFFERSIZE = WIDTH * HEIGHT * 4;
    IKinectSensor* pSensor =nullptr;
};





#endif // KINECT_COLOR_THREAD_H
