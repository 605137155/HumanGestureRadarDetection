#pragma once
#pragma execution_character_set("utf-8")
#include <QtWidgets/QMainWindow>
#include "ui_TheUI.h"
#include "KinectThread.h"
#include "KinectColorThread.h"
#include "RadarSerialThread.h"
#include "MyOpenGLWidget.h"
#include "AllOpenGLWidget.h"
#include "RadarOpenGLWidget.h"
#include <Q3DScatter>
#include <QSpinBox>
#include <QTcpSocket>
#include <QTcpServer>
#include <k4a/k4a.h>
#include <k4abt.h>

class GestureForRelease : public QMainWindow
{
    Q_OBJECT
public:
    GestureForRelease(QWidget* parent = nullptr);
    ~GestureForRelease();
    AllOpenGLWidget* allOpenGLWidget;


private:
    Ui::TheUI ui;
    KinectThread* kinectThread;
    //KinectColorThread* kinectColorThread;
    RadarSerialThread* radarSerialThread;
    MyOpenGLWidget* myOpenGLWidget;
    MyOpenGLWidget* kinectOpenGLWidget;
    //IKinectSensor* iKinectSensor;
    k4a_device_t AzureKinectDevice;
    RadarOpenGLWidget* radarOpenGLWidget;
    QWidget* scatterContainer;
    QComboBox* comboBox;
    QSpinBox* spinBox;
    QtDataVisualization::QScatterDataProxy* proxy;
    QtDataVisualization::QScatter3DSeries* series;
    QTcpServer *server;
    QTcpSocket *socket;
    //int kinectOpenResult;

public slots:
    void PushButtonClicked();
    void startKinect();
    void stopKinect();
    void onNewDataAvailable(const QString& data);
    void onErrorOccurred(const QString& error);
    void startRadar();
    void stopRadar();
    void updateColor(const QPixmap& image);
    void updateDirection(const QString& x, const QString& y, const QString& z);
    void startAll();
    void stopAll();
    void updataRadarPointCloud(const QQueue<QString> data);
    void updataBodyFrame(const QVector < QVector<QVector<GLfloat>>>& data, const QVector<uint32_t> &allBodyIndexs, QVector<QVector3D> depthPointClounds);
    void addRadarData(const QQueue<QString> pointCloudBuffer);
    void onComBoBoxSelectionSelection(int index);
    void setTimerThreshold(int ms);
    void openFileDialog();
    void sendDataToClient();
    void showClientData();
    void onNewConnection();
    void setMinDepth(QString s);
    void setMaxDepth(QString s);
    void setDepthBool(int state);
    void setMinX(QString s);
    void setMaxX(QString s);
    void setMinY(QString s);
    void setMaxY(QString s);
    void setFilePath();

signals:
    void replay(QString filepath);

};

