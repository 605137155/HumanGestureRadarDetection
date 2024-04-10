#pragma once
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QPainter>
#include <QQueue>


class RadarOpenGLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    // ...
    float rotationX = 11.0f;
    float rotationY = -3.0f;
    float zoomFactor = -11.0f;
    float translateX = 0.0f;
    float translateY = 0.766902f;
    bool isRigheMousePressed = false;
    QString name = "theRadarWidget";
    RadarOpenGLWidget(QWidget* parent = nullptr);

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
signals:
    // 定义信号，例如当新数据可用时
    void mouseMove(const QString& x, const QString& y, const QString& z);
public slots:
    void updateRadarPoints(QQueue<QString> pBuffer);

private:
    QString currentRadarPoints;
    QPoint lastMousePos;
    QQueue<QString> pointCloudBuffer;
    bool hover = false;
};

