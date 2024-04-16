#include <QTcpServer>
#include <QTcpSocket>
//#include <QCoreApplication>
#include <QDebug>
#include <QFile>
#include <QTextStream>
#include <QTimer>
#include <QtEndian>
#include <QThread>
class TcpServer : public QTcpServer {
    Q_OBJECT
public:
    TcpServer(QObject* parent = nullptr) : QTcpServer(parent) {
        startTimer(1000); // �޸�Ϊÿx�����һ�� ���ﵥλΪ���룬1000Ϊ1��
        logFile.setFileName("speedLog.txt"); // ��־�ļ���
        if (!logFile.open(QIODevice::Append | QIODevice::Text)) {
            qDebug() << "Cannot open log file for writing.";
        }
        startStatisticsTimer();



    }
    ~TcpServer() {
        logFile.close();
    }

protected:
    void incomingConnection(qintptr socketDescriptor) override {
        QTcpSocket* socket = new QTcpSocket(this);
        if (socket->setSocketDescriptor(socketDescriptor)) {
            //�յ����ݵĻ�����ִ�����溯��
            connect(socket, &QTcpSocket::readyRead, this, [this, socket]() {
                QByteArray data = socket->readAll();
                //qDebug() << data.toHex();
                // ������Ը�����Ҫ��frameData���н�һ������
                frameCount += 1;
                receivedBytes += data.size();
                bool res = checkFrameValid(data);
                count_all +=1;
                qDebug() << "coult_all:" << count_all;
                });
            connect(socket, &QTcpSocket::disconnected, socket, &QTcpSocket::deleteLater);
        }
        else {
            delete socket;
        }
    }

    void timerEvent(QTimerEvent* event) override {
        Q_UNUSED(event);
        if (receivedBytes > 0) {
            double speedKbps = receivedBytes / 1024.0 / 2.0; // ת��ΪKB/s��ע�����10��Ϊ������10�����һ��
            qDebug() << "Received" << receivedBytes << "bytes in last 2 seconds.";
            qDebug() << "Speed:" << speedKbps << "KB/s";
            // д���ļ�
            QTextStream out(&logFile);
            out << "Received " << receivedBytes << " bytes in last 2 seconds.\n";
            out << "Speed: " << speedKbps << " KB/s\n";
            receivedBytes = 0;
            printStatistics();
        }
    }

    void printStatistics() {
        qDebug() << "Frames received in last second:" << frameCount;
        frameCount = 0; // ���ü�����
    }

    bool checkFrameValid(QByteArray data) {
        //quint32 frameLength = qFromLittleEndian<quint32>(reinterpret_cast<const uchar*>(QByteArray::fromHex(data.mid(4, 8)).constData()));
        //qDebug() << "framelength:"<<frameLength<<"data.size:"<<data.size();
        //qDebug() << data.right(1);
        if (data.right(1) != QByteArray::fromHex("EE"))
        {
            qDebug() << "not ee::" << data.right(1);
            QThread::sleep(5);
        }

        return true;
    }

private:
    qint64 receivedBytes = 0;
    QFile logFile;
    int frameCount = 0;
    int count_all = 0;
    void startStatisticsTimer() {
        QTimer* timer = new QTimer(this);
        connect(timer, &QTimer::timeout, this, &TcpServer::printStatistics, Qt::UniqueConnection);
        //timer->start(2000); // ÿ�봥��һ��
    }
};