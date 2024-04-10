#include "GestureForRelease.h"
#include <QtWidgets/QApplication>
#include <QTextCodec>
//#include <vld.h>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    qRegisterMetaType<QVector<QVector<QVector<GLfloat>>>>("QVector<QVector<QVector<GLfloat>>>");//ע��������������ڲۻ��ƴ���
    qRegisterMetaType<QVector<qint16>>("QVector<qint16>");//ע��������������ڲۻ��ƴ���
    qRegisterMetaType<QVector<QString>>("QVector<QString>");//ע��������������ڲۻ��ƴ���
    qRegisterMetaType<QVector<uint32_t>>("QVector<uint32_t>");//ע��������������ڲۻ��ƴ���
    qRegisterMetaType<QVector<QVector3D>>("QVector<QVector3D>");
    qRegisterMetaType<std::string>("std::string");//ע��������������ڲۻ��ƴ��� 
    qRegisterMetaType<std::vector<std::string>>("std::vector<std::string>");
    GestureForRelease w;
    w.setWindowTitle("��ά������̬ʶ�����");
    QIcon icon("./image/air.ico");
    w.setWindowIcon(icon);
    w.show();
    //QCoreApplication::processEvents();
    return a.exec();
}
