#pragma once
#ifndef KINECT_THREAD_H
#define KINECT_THREAD_H
#include <QThread>
#include <QMutex>
#include <iostream>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QAtomicInteger>
#include <Kinect.h>
#include "FileSaveObject.h"
#include <k4a/k4a.h>
#include <k4abt.h>
#include <qdebug.h>
#include <QImage>
#include <QVector3D>
using namespace std;

class KinectThread : public QThread
{
    Q_OBJECT
public:
    KinectThread(QObject* parent);
    ~KinectThread();
    // 其他函数和变量
    void run() override;
    string generateFilenameBasedOnUTC();
    void stop() { m_stop.store(1);
    qDebug() << "tracker_address" << &(this->tracker);
    k4abt_tracker_shutdown(this->tracker);
    qDebug() << "device_address" << &(this->device);
    k4a_device_stop_cameras(this->device);
    k4a_device_close(this->device);
    };
    void stopKinect();
    void setMinDepth(float min);
    void setMaxDepth(float max);
    void setMinX(float min);
    void setMaxX(float max);
    void setMinY(float min);
    void setMaxY(float max);
signals:
    // 定义信号，例如当新数据可用时
    void bodyFrameReady(QVector<QVector<QVector<GLfloat>>> bodyFrameData, QVector<uint32_t> allBodyIndexs, QVector<QVector3D> depthPointClounds);
    void writeFile(std::string, std::vector<std::string>);
    void writeFileForDepth(std::string, std::vector<std::string>);
    void colorFrameReady(QPixmap pixmap);
    void dataShow(QString s);

private:
    QMutex mutex;
    // 其他私有成员变量
    QAtomicInteger<int> m_stop;
    //IKinectSensor* pSensor;
    QThread* t;
    FileSaveObject* fs;
    QThread* t2;//保存深度信息的线程
    FileSaveObject* ds;//保存深度信息的文件对象
    int bodies;
    std::string filename;
    std::string depthfilename;
    QMutex lock;
public:
    k4abt_tracker_t tracker;
    //k4a_transformation_t transformation;
    k4a_device_t device;
    QString trackerMode;
    float minDepth = 0.0;
    float maxDepth = 20.0;
    bool saveDepth = false;
    float minX = -20.0;
    float maxX = 20.0;
    float minY = -20.0;
    float maxY = 20.0;
    QString folderPath;

};





#endif // KINECT_THREAD_H
