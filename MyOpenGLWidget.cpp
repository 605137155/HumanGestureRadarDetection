#include "MyOpenGLWidget.h"
#include <QOpenGlFunctions>
#include <iostream>
#include<QDebug>
#include <GL/glu.h>
#include "ui_TheUI.h"
#include <QMouseEvent>
#include <QWheelEvent>
#include <QVector>
#include <QRandomGenerator>
#include <QVector3D>

int pointSize = 32;
void drawAxisGrid() {
    glColor3f(0.5f, 0.5f, 0.5f); // ��ɫ�̶���

    // X��̶�
    for (float i = -4.0f; i <= 4.0f; i += 1.0f) {
        glBegin(GL_LINES);
        glVertex3f(i, -2.0f, 0.0f); // С�·��̶�
        glVertex3f(i, 1.5f, 0.0f);  // С�Ϸ��̶�
        glVertex3f(i, -2.0f, 0.0f); // 
        glVertex3f(i, -2.0f, 10.0f);  // 
        glEnd();
    }

    // Y��̶�
    for (float i = -2.0f; i <= 1.5f; i += 0.5f) {
        glBegin(GL_LINES);
        glVertex3f(-4.0f, i, 0.0f); // С�󷽿̶�
        glVertex3f(4.f, i, 0.0f);  // С�ҷ��̶�
        glVertex3f(-4.0f, i, 0.0f); // 
        glVertex3f(-4.f, i, 10.0f);  // 
        glEnd();
    }

    // Z��̶�
    for (float i = 0.0f; i <= 10.0f; i += 1.0f) {
        glBegin(GL_LINES);
        glVertex3f(-4.0f, -2.0f, i); // 
        glVertex3f(-4.0f, 1.5f, i);  // 
        glVertex3f(-4.0f, -2.0f, i); // 
        glVertex3f(4.0f, -2.0f, i);  // 
        glEnd();
    }

    // ��X, Y, Z����
    glColor3f(1.0f, 0.0f, 0.0f); // ��ɫX��
    glBegin(GL_LINES);
    glVertex3f(-4.0f, -2.0f, 0.0f);
    glVertex3f(4.0f, -2.0f, 0.0f);
    glEnd();

    glColor3f(0.0f, 1.0f, 0.0f); // ��ɫY��
    glBegin(GL_LINES);
    glVertex3f(-4.0f, -2.0f, 0.0f);
    glVertex3f(-4.0f, 1.5f, 0.0f);
    glEnd();

    glColor3f(0.0f, 0.0f, 1.0f); // ��ɫZ��
    glBegin(GL_LINES);
    glVertex3f(-4.0f, -2.0f, 0.0f);
    glVertex3f(-4.0f, -2.0f, 10.0f);
    glEnd();
}
QVector<QVector<GLfloat>> generateRandomData() {
    QVector<QVector<GLfloat>> data;

    // 25 rows
    for (int i = 0; i < pointSize; ++i) {
        QVector<GLfloat> row;

        // 3 columns
        for (int j = 0; j < 3; ++j) {
            // Generate a random float between 0 and 1
            GLfloat randomValue = static_cast<GLfloat>(QRandomGenerator::global()->bounded(1.0));
            row.append(randomValue);
        }
        data.append(row);
    }
    return data;
}


MyOpenGLWidget::MyOpenGLWidget(QWidget* parent) :QOpenGLWidget(parent), hover(false) {

}
void MyOpenGLWidget::initializeGL()
{

    //initialization code
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);


}


void MyOpenGLWidget::resizeGL(int w, int h)
{
    // Resize event handling
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, (float)w / h, 0.1, 100.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}
void MyOpenGLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    glTranslatef(translateX, translateY, zoomFactor);
    glRotatef(rotationX, 1, 0, 0);
    glRotatef(rotationY, 0, 1, 0);

    drawAxisGrid();
    drawAxisLabels();

    
    //������ȵ���
    glPointSize(1);
    glBegin(GL_POINTS);
    //int count = 0;
    for (const auto& point : depthPointClounds) {
        float depth = point.z();
        //if (depth < minDepth || depth >maxDepth) continue;
        float colorIntensity = 1.0 - (depth / 10.0f);
        glColor3f(colorIntensity, colorIntensity, colorIntensity);
        GLfloat vertex[3] = { point.x(), point.y(), point.z() };
        glVertex3f(point.x(), point.y(), point.z());
        //qDebug() << "pointvalue::"<< point.x() << point.y() << point.z();
        //qDebug() << "points_Z" << point.z();
    }
    glEnd();

    //���Ʋɼ����������
    QVector<QVector<QVector<GLfloat>>>  allBodyFrame = currentBodyFrame;
    for (int num_body = 0; num_body < allBodyFrame.size(); num_body++) {
        QVector<QVector<GLfloat>> bodyFrame = allBodyFrame[num_body];
        GLfloat joint[32][3];
        // Draw Joints
        switch (bodyIndexs[num_body]) {
        case 0:glColor3f(0, 0, 0); break;// white
        case 1:glColor3f(1, 0, 0); break;// 
        case 2:glColor3f(0, 1, 0); break;// 
        case 3:glColor3f(1, 1, 0); break;// 
        case 4:glColor3f(0, 0, 1); break;// 
        case 5:glColor3f(1, 0, 1); break;// 
        case 6:glColor3f(0, 1, 1); break;// 
        default: glColor3f(1, 1, 1); break;// 
        }
        //glColor3f(1, 1, 1); // white
        glPointSize(4);
        glBegin(GL_POINTS);
        for (int i = 0; i < pointSize; i++) {
            joint[i][0] = bodyFrame[i][0];
            joint[i][1] = bodyFrame[i][1];
            joint[i][2] = bodyFrame[i][2];
            glVertex3fv(joint[i]);
        }
        glEnd();
        glColor3f(0, 1, 0); // green
        glBegin(GL_LINES);
        for (int i = 0; i < sizeof(connections) / sizeof(connections[0]); i++) {
            glVertex3fv(joint[connections[i][0]]);
            glVertex3fv(joint[connections[i][1]]);
        }
        glEnd();
    }

    //���ƾ���������
    QVector<QVector<QVector<GLfloat>>>  resultBody = clusterResultBodyFrame;
    for (int num_body = 0; num_body < resultBody.size(); num_body++) {
        QVector<QVector<GLfloat>> bodyFrame = resultBody[num_body];
        GLfloat joint[32][3];
        // Draw Joints
        glColor3f(1, 1, 1); // white
        glPointSize(5);
        glBegin(GL_POINTS);
        for (int i = 0; i < pointSize; i++) {

            joint[i][0] = bodyFrame[i][0];
            joint[i][1] = bodyFrame[i][1];
            joint[i][2] = bodyFrame[i][2];
            glVertex3fv(joint[i]);
        }
        glEnd();


        glColor3f(0, 1, 0); // green
        glBegin(GL_LINES);
        for (int i = 0; i < sizeof(connections) / sizeof(connections[0]); i++) {
            glVertex3fv(joint[connections[i][0]]);
            glVertex3fv(joint[connections[i][1]]);
        }
        glEnd();
    }

    

    QPainter painter(this);
    if (hover) {
        painter.setPen(QPen(Qt::red, 1));
    }
    else {
        painter.setPen(QPen(Qt::black, 1));
    }
    painter.drawRect(rect().adjusted(1, 1, -1, -1));

}

void MyOpenGLWidget::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::RightButton) {
        lastMousePos = event->pos();
        isRigheMousePressed = true;
    }
    else
        lastMousePos = event->pos();
}

void MyOpenGLWidget::mouseMoveEvent(QMouseEvent* event) {
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
        //��������Ҽ��϶��������ƽ����������translateX
        translateX += dx * 0.05f;//ƽ�ƶȵ���
        translateY -= dy * 0.05f;//ƽ�ƶȵ���
        update();
        qDebug() << QString::number(translateX, 'f', 2);
        qDebug() << QString::number(translateY, 'f', 2);
    }
    lastMousePos = event->pos();
}

void MyOpenGLWidget::wheelEvent(QWheelEvent* event) {

    qDebug() << QString::number(zoomFactor, 'f', 2);
    zoomFactor += event->angleDelta().y() / 120.0f;
    update();
    qDebug() << QString::number(zoomFactor, 'f', 2);
    emit mouseMove(QString::number(rotationX, 'f', 2), QString::number(rotationY, 'f', 2), QString::number(zoomFactor, 'f', 2));

}





void MyOpenGLWidget::updateBodyFrame(QVector<QVector<QVector<GLfloat>>> newFrame, QVector<uint32_t> allBodyIndexs, QVector<QVector3D> dp) {

    //ʹ��newFrame����OpenGL Widget
    this->currentBodyFrame = newFrame;
    this->bodyIndexs = allBodyIndexs;
    depthPointClounds = dp;
    update();
}

void MyOpenGLWidget::updateBodyFrameQstring(QString data) {

    QStringList stringList = data.split(" ");
    if (!stringList.isEmpty()) { stringList.removeFirst(); }

    int n = stringList.size() / (25 * 4); // ���� n*25*4 ���� stringList �Ĵ�С
    QVector<QVector<QVector<GLfloat>>> result;
    for (int i = 0; i < n; ++i) {
        QVector<QVector<GLfloat>> body;

        for (int j = 0; j < 25; ++j) {
            QVector<GLfloat> point;

            for (int k = 0; k < 3; ++k) {
                QString value = stringList[i * 25 * 4 + j * 4 + k];
                bool ok;
                GLfloat floatValue = value.toFloat(&ok);

                if (ok) {
                    point.append(floatValue);
                }
                else {
                    // ������Ч�ĸ�����
                    qDebug() << "��Ч�ĸ�������" << value;
                }
            }

            body.append(point);
        }

        result.append(body);
    }

    this->clusterResultBodyFrame = result;
    qDebug() << result;
    //qDebug() << "�а�����";
    update();
}


void MyOpenGLWidget::drawTextAt3DPosition(float x, float y, float z, const QString& text) {
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

void MyOpenGLWidget::drawAxisLabels() {
    // X��̶�
    for (float i = -3.0f; i <= 4.0f; i += 1.0f) {
        drawTextAt3DPosition(i, -2.0f, 0.0f, QString::number(i));
    }

    // Y��̶�
    for (float i = -2.0f; i <= 1.5f; i += 1.0f) {
        drawTextAt3DPosition(-4.3f, i, 0.0f, QString::number(i));
    }

    // Z��̶�
    for (float i = 0.0f; i < 10.0f; i += 2.0f) {
        drawTextAt3DPosition(-4.0f, -2.3f, i, QString::number(i));
    }
}
void MyOpenGLWidget::enterEvent(QEvent* event) {

    Q_UNUSED(event);
    hover = true;
    resize(width()+50, height()+ 50);
    update();
    raise();
}
void MyOpenGLWidget::leaveEvent(QEvent* event) {

    Q_UNUSED(event);
    hover = false;
    resize(width() - 50, height() - 50);
    update();
}

void MyOpenGLWidget::setMinDepth(float min) {
    
    lock.lock();
    minDepth = min;
    lock.unlock();
}

void MyOpenGLWidget::setMaxDepth(float max) {
    lock.lock();
    maxDepth = max;
    lock.unlock();
}

//void MyOpenGLWidget::mouseDoubleClickEvent(QMouseEvent* event){
//    if (event->button() == Qt::LeftButton) {
//        QWidget* newWindow = new QWidget;
//        this->setParent(newWindow);
//        newWindow->setAttribute(Qt::WA_DeleteOnClose);
//        newWindow->setWindowTitle("����1");
//        newWindow->resize(1920, 1920);
//        newWindow->show();
//    }
//}