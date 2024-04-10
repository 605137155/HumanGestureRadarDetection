#include "GestureForRelease.h"
#include <QtWidgets/QApplication>
#include <QTextCodec>
//#include <vld.h>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    qRegisterMetaType<QVector<QVector<QVector<GLfloat>>>>("QVector<QVector<QVector<GLfloat>>>");//注册非内置类型用于槽机制传递
    qRegisterMetaType<QVector<qint16>>("QVector<qint16>");//注册非内置类型用于槽机制传递
    qRegisterMetaType<QVector<QString>>("QVector<QString>");//注册非内置类型用于槽机制传递
    qRegisterMetaType<QVector<uint32_t>>("QVector<uint32_t>");//注册非内置类型用于槽机制传递
    qRegisterMetaType<QVector<QVector3D>>("QVector<QVector3D>");
    qRegisterMetaType<std::string>("std::string");//注册非内置类型用于槽机制传递 
    qRegisterMetaType<std::vector<std::string>>("std::vector<std::string>");
    GestureForRelease w;
    w.setWindowTitle("三维人体姿态识别软件");
    QIcon icon("./image/air.ico");
    w.setWindowIcon(icon);
    w.show();
    //QCoreApplication::processEvents();
    return a.exec();
}
