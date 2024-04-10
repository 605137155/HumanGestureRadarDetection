#ifndef RADARSERIALTHREAD_H
#define RADARSERIALTHREAD_H

#include <QThread>
#include <QSerialPort>
#include <iostream>
#include <QQueue>
#include <QMutex>
#include "FileSaveObject.h"
#include <QFile>
using namespace std;

class RadarSerialThread : public QThread
{
    Q_OBJECT

public:
    RadarSerialThread(QObject* parent);
    //RadarSerialThread();
    ~RadarSerialThread();
    void startCollection();
    string generateFilenameBasedOnUTC();
    void stop();
    void setPortName(QString pname);
signals:
    void newDataAvailable(const QString& data);
    void errorOccurred(const QString& error);
    void pointCloundReady(QQueue<QString> pcBuffer);
    void pointCloundReady_(QString data);
    void writeFile(QString filename, QVector<QString> HexDatas, QVector<QString> PointDatas, QVector<qint16> nums);
    void simulate();
protected:
    void run() override;
private slots:
    void internalStartCollection();
    void readData();
    void saveDataToFile();
    void saveNumToFile();
    void saveHexDataToFile();
    void onTimerTimeout();
    void replayData();


public slots:
    void setFrameThreshold(qint16 frameThreshold);
    void onTimerSetThreshold(int ms);
    int replayTrigger(QString filename);
    int getThreshold();
    void replaySlider(int percent);
private:
    QString processReceivedData(const QByteArray& data, QString& coordinates, QString& wtime, qint16& lastNum);
    QString replayProcess(QByteArray hexdata);
    qint32 frameThreshold;
    qint16 frameCounter;
    QString coordinatesAccumulated;
    QSerialPort* m_serial;
    QMutex lock;
    bool m_collecting;
    QVector<QString> m_dataBuffer;
    QVector<QString> m_stringBuffer;
    QAtomicInteger<int> m_stop;
    QQueue<QString> pointCloudBuffer;
    QVector<qint16> num_buffer;
    QByteArray completeFrameBuffer;
    QTimer* timer;
    bool shouldSendData;
    int timerThresold;
    QVector<QByteArray> replayframes;
    QVector<QString> replaySliderFrames;
    bool shouldReplay;
    string filename;
    QFile file;
    int numPoints;
    QThread* t;
    FileSaveObject* fs;
public:
    QString folderPath;

};

#endif // RADARSERIALTHREAD_H