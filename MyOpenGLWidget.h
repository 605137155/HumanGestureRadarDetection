#pragma once
// your_opengl_widget.h
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QPainter>
#include <QVector3D>
#include <QMutex>
class MyOpenGLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    // ...
    float rotationX = 15.0f;
    float rotationY = 3.0f;
    float zoomFactor = -11.0f;
    float translateX = 0.16f;
    float translateY = 1.35f;
    bool isRigheMousePressed = false;
    QString name = "zhangminghui";
    MyOpenGLWidget(QWidget* parent = nullptr);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void drawTextAt3DPosition(float x, float y, float z, const QString& text);
    void drawAxisLabels();
    void enterEvent(QEvent* event) override;
    void leaveEvent(QEvent* event) override;
    //void mouseDoubleClickEvent(QMouseEvent* event) override; //双击弹出窗口，


signals:
    // 定义信号，例如当新数据可用时
    void mouseMove(const QString& x, const QString& y, const QString& z);

public slots:
    void updateBodyFrame(QVector<QVector<QVector<GLfloat>>> bodyFrameData, QVector<uint32_t> allBodyIndexs, QVector<QVector3D> dp);
    void updateBodyFrameQstring(QString data);
    void setMinDepth(float min);
    void setMaxDepth(float max);

private:

    QVector<QVector<QVector<GLfloat>>> currentBodyFrame = { {
            // 头部
            { -0.226682, -0.582183 ,2.81891},
            {-0.248509 ,-0.269394 ,2.82 },
           { -0.268584, 0.0352062 ,2.80859 },
            {-0.264985, 0.165643 ,2.78738 },
            {-0.421449 ,-0.058754 ,2.76588 },
            {-0.469522, -0.274122 ,2.74691 },
            {-0.425525 ,-0.502156 ,2.6875 },
            {-0.404146 ,-0.586188 ,2.67672 },
            {-0.0943585, -0.063671 ,2.81429 },
            {-0.0572729, -0.320044 ,2.79906 },
            {-0.04906 ,-0.530838 ,2.73629  },
            {-0.0619439, -0.594917 ,2.73123 },
            {-0.298247, -0.573424 ,2.77504 },
            {-0.313118, -0.882475 ,2.74431  },
            {-0.303004, -1.2263 ,2.82261 },
            {-0.312514, -1.31536 ,2.71921 },
            {-0.149291 ,-0.576055, 2.79035 },
            {-0.118626 ,-0.889024 ,2.77635 },
            {-0.149781 ,-1.25434, 2.88951 },
            {-0.118615, -1.3305 ,2.79462 },
            {-0.263883, -0.0398698, 2.8139},
            {-0.395244 ,-0.661109 ,2.67475},
            {-0.36311, -0.568187, 2.68183 },
            {-0.0486456, -0.671356, 2.71506},
            {-0.0989873, -0.591273 ,2.70306 },
            {-0.149781 ,-1.25434, 2.88951 },
            {-0.118615, -1.3305 ,2.79462 },
            {-0.263883, -0.0398698, 2.8139},
            {-0.395244 ,-0.661109 ,2.67475},
            {-0.36311, -0.568187, 2.68183 },
            {-0.0486456, -0.671356, 2.71506},
            {-0.0989873, -0.591273 ,2.70306 }
        } };
    QVector<uint32_t> bodyIndexs = {0};
    QVector<QVector<QVector<GLfloat>>> clusterResultBodyFrame = {};

    QPoint lastMousePos;
    // Define connections between joints
    int connections[31][2] = {
        {1,0},{2,0},{3,1},{4,1},{5,4},{6,5},{7,6},{8,7},{9,8},{10,7},
        {11,2},{12,11},{13,12},{14,13},{15,14},{16,15},{17,14},{18,0},{19,18},{20,19},
        {21,20},{22,0},{23,22},{24,23},{25,24},{26,3},{27,26},{28,26},{29,26},{30,26},
        {21,26}
    };
    bool hover;
    QVector<QVector3D> depthPointClounds;
    float minDepth = 0.0;
    float maxDepth = 20.0;
    QMutex lock;

};

