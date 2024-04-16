#include <QTcpServer>
#include <QTcpSocket>
#include <QByteArray>
#include <QObject>
#include <QDateTime>
#include <QTimer>
#include <QDebug>

class FrameServer : public QObject {
    Q_OBJECT
public:
    FrameServer(quint16 port, QObject* parent = nullptr) : QObject(parent), frameCount(0) {
        connect(&server, &QTcpServer::newConnection, this, &FrameServer::handleNewConnection);
        if (!server.listen(QHostAddress::Any, port)) {
            qDebug() << "Server could not start!";
        }
        startStatisticsTimer();
    }

private slots:
    void handleNewConnection() {
        QTcpSocket* clientConnection = server.nextPendingConnection();
        connect(clientConnection, &QTcpSocket::disconnected, clientConnection, &QTcpSocket::deleteLater);
        connect(clientConnection, &QTcpSocket::readyRead, this, &FrameServer::readFrame);
    }

    void readFrame() {
        QTcpSocket* clientConnection = qobject_cast<QTcpSocket*>(sender());
        QByteArray frameData = clientConnection->readAll();
        frameCount += 1; // 假设每次readAll()获取的都是完整的一帧
        // 这里可以根据需要对frameData进行进一步处理
    }

    void printStatistics() {
        qDebug() << "Frames received in last second:" << frameCount;
        frameCount = 0; // 重置计数器
    }

private:
    QTcpServer server;
    int frameCount;

    void startStatisticsTimer() {
        QTimer* timer = new QTimer(this);
        connect(timer, &QTimer::timeout, this, &FrameServer::printStatistics);
        timer->start(1000); // 每秒触发一次
    }
};

