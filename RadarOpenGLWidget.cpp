#include "RadarOpenGLWidget.h"
#include <QOpenGlFunctions>
#include <iostream>
#include<QDebug>
#include <GL/glu.h>
#include "ui_TheUI.h"
#include <QMouseEvent>
#include <QWheelEvent>
#include <QVector>
#include <QRandomGenerator>
#pragma execution_character_set("utf-8")

void RadarOpenGLWidget::drawAxisLabels() {
    // X轴刻度
    for (float i = -3.0f; i <= 4.0f; i += 1.0f) {
        drawTextAt3DPosition(i, -2.0f, 0.0f, QString::number(i));
    }

    // Y轴刻度
    for (float i = -2.0f; i <= 1.5f; i += 1.0f) {
        drawTextAt3DPosition(-4.3f, i, 0.0f, QString::number(i));
    }

    // Z轴刻度
    for (float i = 0.0f; i < 10.0f; i += 2.0f) {
        drawTextAt3DPosition(-4.0f, -2.3f, i, QString::number(i));
    }
}

void drawAxisGrid_radar() {
    glColor3f(0.5f, 0.5f, 0.5f); // 灰色刻度线

    // X轴刻度
    for (float i = -4.0f; i <= 4.0f; i += 1.0f) {
        glBegin(GL_LINES);
        glVertex3f(i, -2.0f, 0.0f); // 小下方刻度
        glVertex3f(i, 1.5f, 0.0f);  // 小上方刻度
        glVertex3f(i, -2.0f, 0.0f); // 
        glVertex3f(i, -2.0f, 10.0f);  // 
        glEnd();
    }

    // Y轴刻度
    for (float i = -2.0f; i <= 1.5f; i += 0.5f) {
        glBegin(GL_LINES);
        glVertex3f(-4.0f, i, 0.0f); // 小左方刻度
        glVertex3f(4.f, i, 0.0f);  // 小右方刻度
        glVertex3f(-4.0f, i, 0.0f); // 
        glVertex3f(-4.f, i, 10.0f);  // 
        glEnd();
    }

    // Z轴刻度
    for (float i = 0.0f; i <= 10.0f; i += 1.0f) {
        glBegin(GL_LINES);
        glVertex3f(-4.0f, -2.0f, i); // 
        glVertex3f(-4.0f, 1.5f, i);  // 
        glVertex3f(-4.0f, -2.0f, i); // 
        glVertex3f(4.0f, -2.0f, i);  // 
        glEnd();
    }

    // 画X, Y, Z轴线
    glColor3f(1.0f, 0.0f, 0.0f); // 红色X轴
    glBegin(GL_LINES);
    glVertex3f(-4.0f, -2.0f, 0.0f);
    glVertex3f(4.0f, -2.0f, 0.0f);
    glEnd();

    glColor3f(0.0f, 1.0f, 0.0f); // 绿色Y轴
    glBegin(GL_LINES);
    glVertex3f(-4.0f, -2.0f, 0.0f);
    glVertex3f(-4.0f, 1.5f, 0.0f);
    glEnd();

    glColor3f(0.0f, 0.0f, 1.0f); // 蓝色Z轴
    glBegin(GL_LINES);
    glVertex3f(-4.0f, -2.0f, 0.0f);
    glVertex3f(-4.0f, -2.0f, 10.0f);
    glEnd();
}


RadarOpenGLWidget::RadarOpenGLWidget(QWidget* parent) :QOpenGLWidget(parent) {

}
void RadarOpenGLWidget::initializeGL()
{

    // OpenGL initialization code
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);


}


void RadarOpenGLWidget::resizeGL(int w, int h)
{
    // Resize event handling
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (float)w / h, 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}
void RadarOpenGLWidget::paintGL()
{

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glTranslatef(translateX, translateY, zoomFactor);
    qDebug() << translateX << translateY;
    glRotatef(rotationX, 1, 0, 0);
    glRotatef(rotationY, 0, 1, 0);

    drawAxisGrid_radar();
    drawAxisLabels();

    // Draw points
    glColor3f(1, 1, 1); // white
    glPointSize(5);
    glBegin(GL_POINTS);

    //绘制雷达点云
        //动态分配内存
    
    //int count__ = 0;
    for (const QString& pointsFrame : pointCloudBuffer) {
        QStringList lines = pointsFrame.split("\n");
        int numPoints = lines.size();
        GLfloat(*points)[3] = new GLfloat[numPoints][3];
        qDebug()<<"窗口绘制！！！！！！";
        for (int i = 0; i < numPoints; ++i) {
            QStringList coords = lines[i].split(" ");
            if (coords.size() == 3) {
                //雷达的xyz,对应相机空间的xzy， 而openglwidget的空间与相机空间相同，故而把雷达的xyz的z和y对调再投射到openglwidget中
                std::vector<std::vector<float>> matrix = { {0.95203198f,0.02803822f,-0.12880588f},{0.05550489f,0.67611176f,0.30317671f},{0.0844849f,0.00466166f,0.98018513f} };
                std::vector<float> vector = { coords[0].toFloat(),coords[2].toFloat(),coords[1].toFloat() };//把雷达的z放到相机空间的y //把雷达的y放到相机空间的z

                
                std::vector<float> vector2 = { 0.00613292f,-0.4213969f,0.02359021f };
                std::vector<float>result(3, 0.0f);
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++) {
                        result[i] += matrix[i][j] * vector[j];
                    }
                for (int i = 0; i < 3; ++i) {
                    result[i] = result[i] + vector2[i];
                }
                if (result[0] >= 4.0 || result[0] <= -4.0)
                    continue;

                if (result[1] >= 1.5 || result[1] <= -2.0)
                    continue;
                if (result[2] >= 10.0 || result[2] <= 0.0)
                    continue;

                points[i][0] = result[0];
                points[i][1] = result[1];
                points[i][2] = result[2];
                
                /*points[i][0] = coords[0].toFloat();
                points[i][1] = coords[2].toFloat();
                points[i][2] = coords[1].toFloat();*/
                //count__++;
                if (points[i][1] < -0.7f) {
                    glColor3f(1.0f, 0.0f, 0.0f);
                }
                else if(points[i][1]<-0.3f){
                    glColor3f(0.0f, 1.0f, 0.0f);
                }
                else {
                    glColor3f(1.0f, 1.0f, 0.0f);
                }
                //qDebug() << "normal corddd::" << vector;
                /*if (vector[0]==0&&vector[1]==0&&vector[2]==0)
                    qDebug() << " zero corssss::!!!" ;*/
                glVertex3fv(points[i]);
                points[i][1] = -2;
                glColor3f(1.0f, 1.0f, 1.0f);
                glVertex3fv(points[i]);
            }
            else {
                //qDebug() << "strange corddd::"<<coords;
            }
            

        }


        /*for (int i = 0; i < numPoints; ++i) {
            delete[] points[i];
        }*/
        delete[] points;




    }
    //qDebug() << "point_num::" << count__;
    glEnd();

    QPainter painter(this);
    if (hover) {
        painter.setPen(QPen(Qt::red, 1));
    }
    else {
        painter.setPen(QPen(Qt::black, 1));
    }
    painter.drawRect(rect().adjusted(1, 1, -1, -1));

}

void RadarOpenGLWidget::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::RightButton) {
        lastMousePos = event->pos();
        isRigheMousePressed = true;
    }
    else
        lastMousePos = event->pos();
}

void RadarOpenGLWidget::mouseMoveEvent(QMouseEvent* event) {
    int dx = event->x() - lastMousePos.x();
    int dy = event->y() - lastMousePos.y();

    if (event->buttons() & Qt::LeftButton) {
        rotationX += dy;
        rotationY += dx;
        update();
        qDebug() << QString::number(rotationX, 'f', 2);
        qDebug() << QString::number(rotationY, 'f', 2);
        emit mouseMove(QString::number(rotationX, 'f', 2), QString::number(rotationY, 'f', 2), QString::number(zoomFactor, 'f', 2));
    }
    if (isRigheMousePressed) {
        //如果正在右键拖动，则计算平移量并更新translateX
        translateX += dx * 0.05f;//平移度调整
        translateY -= dy * 0.05f;//平移度调整
        update();
        qDebug() << QString::number(translateX, 'f', 2);
        qDebug() << QString::number(translateY, 'f', 2);
    }

    lastMousePos = event->pos();
}

void RadarOpenGLWidget::wheelEvent(QWheelEvent* event) {

    qDebug() << QString::number(zoomFactor, 'f', 2);
    zoomFactor += event->angleDelta().y() / 120.0f;
    update();
    qDebug() << QString::number(zoomFactor, 'f', 2);
    emit mouseMove(QString::number(rotationX, 'f', 2), QString::number(rotationY, 'f', 2), QString::number(zoomFactor, 'f', 2));

}





void RadarOpenGLWidget::updateRadarPoints(QQueue<QString> pBuffer) {

    //使用newFrame更新OpenGL Widget
    this->pointCloudBuffer = pBuffer;
    update();
    qDebug() << "雷达窗口接收到了数据！";
}

void RadarOpenGLWidget::drawTextAt3DPosition(float x, float y, float z, const QString& text) {
    // Convert 3D coordinates to 2D screen coordinates
    GLdouble model[16], proj[16];
    GLint view[4];
    glGetDoublev(GL_MODELVIEW_MATRIX, model);
    glGetDoublev(GL_PROJECTION_MATRIX, proj);
    glGetIntegerv(GL_VIEWPORT, view);
    GLdouble screenX, screenY, screenZ;
    gluProject(x, y, z, model, proj, view, &screenX, &screenY, &screenZ);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::TextAntialiasing);
    painter.setPen(Qt::white);
    painter.drawText(screenX, height() - screenY, text);  // Note the height()-screenY to invert Y for QPainter
    painter.end();
}


void RadarOpenGLWidget::enterEvent(QEvent* event) {

    Q_UNUSED(event);
    hover = true;
    resize(width() + 50, height() + 50);
    update();
    raise();
}
void RadarOpenGLWidget::leaveEvent(QEvent* event) {

    Q_UNUSED(event);
    hover = false;
    resize(width() - 50, height() - 50);
    update();
}