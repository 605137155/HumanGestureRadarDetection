#include <QtCore/QCoreApplication>
#include "TcpServer.h"
//#include "FrameServer.h"



int main(int argc, char* argv[]) {
    QCoreApplication a(argc, argv);

    TcpServer server = new TcpServer();
    if (!server.listen(QHostAddress("127.0.0.1"), 5010)) {
        qDebug() << "Server failed to start:" << server.errorString();
        return -1;
    }
    qDebug() << "Server started.";


    return a.exec();
}


