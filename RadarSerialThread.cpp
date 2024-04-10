#include "RadarSerialThread.h"
#include <QFile>
#include <QTextStream>
#include <cmath>
#include <QDebug>
#include <QtEndian>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <QDateTime>
#include <QDataStream>
#include <QTimer>
#include <fstream>
#include <QCoreApplication>
#pragma execution_character_set("utf-8")


const double PI = 3.1415926;
double sind(double angleInDegrees) {
    return sin(angleInDegrees * PI / 180.0);
}
double cosd(double angleInDegrees) {
    return cos(angleInDegrees * PI / 180.0);
}

RadarSerialThread::RadarSerialThread(QObject* parent)
    : QThread(parent), m_serial(nullptr), m_collecting(false), shouldSendData(false)
{
    m_serial = new QSerialPort(this);
    frameCounter = 0;
    frameThreshold = 1;
    completeFrameBuffer = "";
    shouldReplay = false;
    connect(m_serial, &QSerialPort::readyRead, this, &RadarSerialThread::readData);
    m_serial->setPortName("COM3");  // �����޸�Ϊ��Ĵ�������
    m_serial->setBaudRate(2000000);
    m_serial->setDataBits(QSerialPort::Data8);
    m_serial->setStopBits(QSerialPort::OneStop);
    m_serial->setParity(QSerialPort::NoParity);
    m_serial->setFlowControl(QSerialPort::NoFlowControl);
    //m_serial->setReadBufferSize(16*1024);

    //��ʱ���������Ҫʱ���ۻ�֡�Ļ������޸ļ�ʱ����ʱ�伴��
    timer = new QTimer(this);
    timer->setInterval(1000);
    connect(timer, &QTimer::timeout, this, &RadarSerialThread::onTimerTimeout);

    //д�ļ������߳�
    fs = new FileSaveObject();
    t = new QThread();
    fs->moveToThread(t);
    connect(this, &RadarSerialThread::writeFile, fs, &FileSaveObject::saveFile);
    

}

RadarSerialThread::~RadarSerialThread() {
    if (m_serial) {
        m_serial->close();
        qDebug() << "guanbixigouleida";
        m_serial->deleteLater();
    }

    if (t) {
        t->quit();
        t->wait();
        qDebug()<<"�˳��߳�";
        delete fs;
        delete t;
        //t->deleteLater();
        
        
    }
}

void RadarSerialThread::run() {

    m_stop = 0;
    filename = generateFilenameBasedOnUTC();
    filename = folderPath.toStdString()+"\n"+ filename;
    qDebug() << QString::fromStdString(filename);
    QFile file(QString::fromStdString(filename));
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        //out.setCodec("UTF-8");
        QString send = QString::fromUtf8("[10:48:58.563]������41 54 2B 53 54 41 52 54 0A �� ");
        file.write(send.toUtf8() + "\n");
        file.close();
        qDebug() << "The first line of File Saved.";
    }
    


    t->start();
    startCollection();
}

void RadarSerialThread::startCollection() {
    QMetaObject::invokeMethod(this, "internalStartCollection", Qt::QueuedConnection);
}



void RadarSerialThread::internalStartCollection() {

    m_dataBuffer.clear();
    m_stringBuffer.clear();
    m_collecting = true;
    qDebug() << "���״�!!";
    if (m_serial) {
        auto state = m_serial->open(QIODevice::ReadWrite);
        m_serial->write("AT+NUMTLV=01\n");
    }
    timer->start();
    exec();
}









void RadarSerialThread::readData() {
    if (!m_stop.load())
    {
        QByteArray data = m_serial->readAll();
        QByteArray hexdata = data.toHex().toUpper();
        qDebug() << "This hexData:" << hexdata;
        QByteArray frameHead = hexdata.mid(0, 4);
        qDebug() << "This frameHead:" << frameHead;
        if (frameHead == "55AA") {
            //"55AA"˵����֡ͷ��������һ����֡
            qDebug() << "Last intack frame:" << completeFrameBuffer;
            if (completeFrameBuffer != "") {
                QString coordinates;
                QString lastFrameTime;
                qint16 lastNum;
                QString processedData = processReceivedData(completeFrameBuffer, coordinates, lastFrameTime, lastNum);
                if (processedData != "failed") {
                    //m_dataBuffer.append(hexdata);
                    //ÿ�����ַ���һ���ո�, ��sscom�����ʽһ��
                    QByteArray spaceHexData;
                    for (int i = 0; i < completeFrameBuffer.size(); i += 2) {
                        QByteArray hexPair = completeFrameBuffer.mid(i, 2);
                        spaceHexData += hexPair + ' ';
                    }
                    QString stringData = QString::fromUtf8(spaceHexData);
                    m_stringBuffer.append(lastFrameTime + QString::fromUtf8("�ա���") + stringData);
                    m_dataBuffer.append(lastFrameTime +"\n"+ coordinates);
                    num_buffer.append(lastNum);
                    //�ɼ������ڴ���� �ɼ�����10000����Ͱ�����д���ĵ���
                    qDebug() << "numPoints:"<< numPoints;
                    if (numPoints >= 300) {
                        QVector<QString> copiedStringBuffer = m_stringBuffer;
                        QVector<QString> copiedDataBuffer = m_dataBuffer;
                        QVector<qint16> copiedNumBuffer = num_buffer;
                        emit writeFile(QString::fromStdString(filename), copiedStringBuffer, copiedDataBuffer, copiedNumBuffer);
                        m_stringBuffer.clear();
                        copiedDataBuffer.clear();
                        copiedNumBuffer.clear();
                        numPoints = 0;
                    }

                    qDebug() << "coordinates____:" << coordinates;
                    //���͸�allopenglwidget����
                    pointCloudBuffer.enqueue(coordinates);

                    //���֡���ۻ�����ֵ��or��ʱ���������ˣ��򽫵���װ�벢����
                    if (pointCloudBuffer.size() > frameThreshold||shouldSendData) {
                        if (shouldSendData)  
                            qDebug() << "���㷢��";
                        while (pointCloudBuffer.size() > frameThreshold) {
                            pointCloudBuffer.dequeue();
                        }

                        //QVector<QString> vector(pointCloudBuffer.begin(), pointCloudBuffer.end()); //����̬���ݷ�ʽ�洢����ʾ���ݣ�����˵�������˷�һЩ���ܣ��ʲ���
                        emit pointCloundReady(pointCloudBuffer);
                        //pointCloudBuffer.clear();
                        shouldSendData = false;
                    }
                    else if (pointCloudBuffer.size() == frameThreshold || shouldSendData) {
                        //QVector<QString> vector(pointCloudBuffer.begin(), pointCloudBuffer.end());
                        emit pointCloundReady(pointCloudBuffer);
                        //pointCloudBuffer.clear();
                        shouldSendData = false;
                    }
                    qDebug() << "framethreashold:" << frameThreshold;

                    completeFrameBuffer = hexdata;
                }
                //����ź���������UI�й۲�֡�ģ����͸�textbrowser����Ŀǰ�����õ�
                emit newDataAvailable(completeFrameBuffer);
                
            }
            else {
                completeFrameBuffer += hexdata;
            }
        }
        else if (frameHead == "4154")
        {
            if (hexdata == "41542B4F4B3D310D0A")
            {
                //m_dataBuffer.append(hexdata);
                m_serial->write("AT+PROG=02\n");
                qDebug() << "sending:AT+PROG=02\n" << frameHead;
                emit newDataAvailable(hexdata);
            }
            else if (hexdata == "41542B4F4B3D30320D0A") {
                //m_dataBuffer.append(hexdata);
                m_serial->write("AT+START\n");
                qDebug() << "sending:AT+START\n" << frameHead;
                emit newDataAvailable(hexdata);
            }
        }
        else {
            completeFrameBuffer += hexdata;
        }
    }
}


QString RadarSerialThread::processReceivedData(const QByteArray& data, QString& coordinates, QString& worldTime, qint16& lastNum) {
    //qDebug() << "start to analysis data��";
    //qDebug() << "data size:"<<data.size();
    // ����֡ͷ  
    QByteArray frameHead = data.mid(0, 4);
    //qDebug() << data;
    //if (frameHead != "55AA") {
    //    
    //    //qDebug() << "head wrong:"<<frameHead;
    //    return "failed";
    //}
    //qDebug() << "frameHead:" << frameHead;
    // ����֡����  
    //if (data.size() < 24) {
    //    //qDebug() << "lengthwrong";
    //    return "failed";
    //}
    //qDebug() << "FrameLength(hex):" << data.mid(4, 8);//���½���ת��
    quint32 frameLength = qFromLittleEndian<quint32>(reinterpret_cast<const uchar*>(QByteArray::fromHex(data.mid(4, 8)).constData()));
    //qDebug() << frameLength;
    //qDebug() << "FrameLength:" << frameLength;
    quint16 TimeInterval = qFromBigEndian<quint16>(reinterpret_cast<const uchar*>(QByteArray::fromHex(data.mid(12, 4)).constData())); //���ģʽת����֡ʱ�䣬��λms
    //0001 1110 0000 0000
    // ���� TLV ����������  
    //qDebug() << "TimeInterval:"<< TimeInterval;

    QByteArray NumTLVs = data.mid(16, 2);
    //qDebug() << "NumTLVs:" << NumTLVs;
    QByteArray type = data.mid(18, 2);
    //if (type != "01") {
    //    //qDebug() << "type wrong:"<<type;
    //    return "failed";
    //}
    //qDebug() << "type:" << type;
    qint16 targetNum = *reinterpret_cast<const qint16*>(QByteArray::fromHex(data.mid(20, 4)).constData());
    qDebug() << "targetNum:" << targetNum;
    QString wTime;
    QDateTime current = QDateTime::currentDateTime();
    //wTime += "[" + QString(current.toString("yyyy - MM - dd HH : mm:ss.zzz")) + "]�ա���";
    wTime += "[" + QString(current.toString("HH:mm:ss.zzz")) + "]";
    worldTime = wTime;
    //outputText += QString(frameHead) + ' '+ QString::number(frameLength) + ' '+ QString::number(TimeInterval) +' ' + QString(NumTLVs) + ' ' + QString(type) + ' ' + QString::number(targetNum);
    QByteArray tlvData = data.mid(24);
    // ���� TLV ����
    if (tlvData.size() != targetNum * 18) {
        //qDebug() << "Fail frame";
        //return "failed";
    }
    else
        qDebug() << "Success frame";
    int count_point = 0;
    QString outputText;
    //�����г���targetnum�������ĵ�û����ʾ��Ϊ����ʾ����ĵ㡣�������Ҫ����10000�Ļ�targetnum��
    for (int i = 0; i < 10000; i++) {
        QByteArray idx1 = tlvData.mid(i * 18, 4);
        QByteArray idx2 = tlvData.mid(i * 18 + 4, 2);
        QByteArray idx3 = tlvData.mid(i * 18 + 6, 2);
        QByteArray idx4 = tlvData.mid(i * 18 + 8, 2);
        QByteArray powABS = tlvData.mid(i * 18 + 10, 8);

        //���½���ת�����ǳ�Σ�գ��ǳ�С�ģ���Ҫȷ����ȫ�˽�Ҫ���е�λ�������ڴ沼�֣�   һ������ֱ�Ӳ���������or�ڴ沼�ֵ������������Ҫ��һ�����ݵĶ����Ʊ�ʾת��Ϊ��һ������or��ĳ�ַ�ʽ�����ڴ��е�λ���ڶ�����ײ�Ӳ������йأ�����ϤҲ��׸�˽�
        qint16 num1 = *reinterpret_cast<const qint16*>(QByteArray::fromHex(tlvData.mid(i * 18, 2)).constData());
        qint8 num2 = *reinterpret_cast<const qint8*>(QByteArray::fromHex(tlvData.mid(i * 18 + 4, 2)).constData());
        qint8 num3 = *reinterpret_cast<const qint8*>(QByteArray::fromHex(tlvData.mid(i * 18 + 6, 2)).constData());
        qint8 num4 = *reinterpret_cast<const qint8*>(QByteArray::fromHex(tlvData.mid(i * 18 + 8, 2)).constData());

        //qDebug() << "num1-4:" << num1 << num2 << num3 << num4;
        double num_1 = static_cast<int>(num1) * 0.05;
        double num_2;
        if (num2 >= 0 && num2 <= 31)
            num_2 = static_cast<int>(num2) * 0.1042;
        else
            num_2 = (static_cast<int>(num2) - 64) * 0.1042;
        double num_3;
        if (num3 >= 0 && num3 <= 63)
        {
            double aa = static_cast<double>(num3) / 64.0;
            //qDebug() << "aa:" << aa;
            num_3 = asin(aa) * 180 / PI;
        }
        else {
            double aa = (static_cast<double>(num3) - 128) / 64.0;
            //qDebug() << "aa:" << aa;
            num_3 = asin(aa) * 180 / PI;
        }

        double num_4;
        if (num4 >= 0 && num4 <= 63) {
            double aa = static_cast<double>(num4) / 64.0;
            //qDebug() << "aa:" << aa;
            num_4 = asin(aa) * 180 / PI;
        }
        else
        {

            double aa = (static_cast<double>(num4) - 128) / 64.0;
            //qDebug() << "aa:" << aa;
            num_4 = asin(aa) * 180 / PI;
        }
        //qDebug() <<"num1-4_:" << num_1 << num_2 << num_3 << num_4;
        double x = num_1 * cosd(num_4) * sind(num_3);
        double y = num_1 * cosd(num_4) * cosd(num_3);
        double z = num_1 * sind(num_4);

        if (idx1 == "" || idx2 == "" || idx3 == "" || idx4 == "") {
            //outputText += "\n";
            break;

        }

        qDebug() << "Idx1:" << idx1 << "Idx2:" << idx2 << "Idx3:" << idx3 << "Idx4:" << idx4 << "PowABS:" << powABS << "x:" << x << "y:" << y << "z:" << z;
        qDebug() << "(" << x << "," << y << "," << z << ")";
        coordinates += QString::number(x) + ' ' + QString::number(y) + ' ' + QString::number(z) + '\n';
        //outputText += '\n' + QString(idx1) + ' ' + QString(idx2) + ' ' + QString(idx3) + ' ' + QString(idx4) + ' ' + QString(powABS) + ' ' + QString::number(x) + ' ' + QString::number(y) + ' ' + QString::number(z);
        outputText += '\n' + QString::number(x) + ' ' + QString::number(y) + ' ' + QString::number(z);
        count_point++;
        //outputText += '('+ QString::number(x) + ',' + QString::number(y) + ',' + QString::number(z)+')'+ '\n';
    }
    //outputText += "\n";
    qDebug() << "caijishuliang::" << count_point;
    lastNum = count_point;
    numPoints += count_point;
    return wTime + ' ' + QString::number(count_point) + outputText;
}

void RadarSerialThread::saveDataToFile() {
    //string filename = generateFilenameBasedOnUTC();
    QFile file(QString::fromStdString(filename));
    if (file.open(QIODevice::WriteOnly)) {
        for (int i = 0; i < m_dataBuffer.size(); i++) {
            file.write(m_dataBuffer[i].toUtf8());
        }
        file.close();
    }
    qDebug() << "File Saved.";
}

void RadarSerialThread::saveNumToFile() {
    QString filename = "num.txt";
    QFile file(filename);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QDataStream out(&file);
        qint16 sum = 0;
        for (int i = 0; i < num_buffer.size(); i++) {
            qint16 a = num_buffer[i];
            sum += a;
            out << QString::number(a).toLocal8Bit() << "\n";
        }

        out << "sum:" << QString::number(sum).toLocal8Bit();
        file.close();
    }
    qDebug() << "Sum File Saved.";
}


void RadarSerialThread::saveHexDataToFile() {
    //string filename = "hex_" + generateFilenameBasedOnUTC();
    QFile file(QString::fromStdString(filename));
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        //QTextStream out(&file);
        //out.setCodec("UTF-8");
        QString send = QString::fromLocal8Bit("[10:48:58.563]������41 54 2B 53 54 41 52 54 0A �� ");
        file.write(send.toLocal8Bit() + "\n");
        for (int i = 0; i < m_stringBuffer.size(); i++) {
            file.write(m_stringBuffer[i].toLocal8Bit() + "\n");
            //out << m_stringBuffer[i] + "\n";
        }
        file.close();
    }
    qDebug() << "File Saved.";
}

string RadarSerialThread::generateFilenameBasedOnUTC() {
    // ��ȡ��ǰ����ʱ��
    std::time_t currentTime = std::time(nullptr);
    std::tm* localTime = std::localtime(&currentTime);
    // ʹ�� std::ostringstream �����ļ���
    std::ostringstream filenameStream;
    filenameStream << "output_Radar_"
        << (localTime->tm_year + 1900) << "-"
        << std::setw(2) << std::setfill('0') << (localTime->tm_mon + 1) << "-"
        << std::setw(2) << std::setfill('0') << localTime->tm_mday << "_"
        << std::setw(2) << std::setfill('0') << localTime->tm_hour << "-"
        << std::setw(2) << std::setfill('0') << localTime->tm_min << "-"
        << std::setw(2) << std::setfill('0') << localTime->tm_sec
        << ".txt";  // ���Ը�����Ҫ������չ��
    return filenameStream.str();
}

void RadarSerialThread::setFrameThreshold(qint16 Threshold) {
    qDebug() << "����";
    lock.lock();
    frameThreshold = Threshold;
    if (shouldReplay) {
        timer->setInterval(1);
        shouldSendData = false;
    }
    else
    {
        timer->setInterval(1000000);//��ʱ������úܴ󣬶�����ͣ�˼�ʱ����ʹ�䲻�ᱻ������ʹ��shouldSendDataһֱΪfalse
        //timer->stop();
    }
    lock.unlock();
    qDebug() << "�ѽ���";
}

void RadarSerialThread::stop() {
    m_stop.store(1);
    m_collecting = false;
    qDebug() << "stop Radar Serial!!";
    m_serial->write("AT\n");
    QByteArray data1 = m_serial->readAll();
    qDebug() << "EndData:" << data1.toHex().toUpper();
    //saveDataToFile();
    //saveHexDataToFile();
    //saveNumToFile();
    timer->stop();
}

void RadarSerialThread::setPortName(QString pname) {
    m_serial->setPortName(pname);
}

void RadarSerialThread::onTimerTimeout(){
    if (shouldSendData)
        qDebug() << "��ʱ�������˷�������,ʱ������"<< std::time(nullptr);
    //shouldSendData = true;
    if (shouldReplay){
        replayData();
        }
    //shouldSendData = true;

}

void RadarSerialThread::onTimerSetThreshold(int ms) {
    timer->setInterval(ms);
    frameThreshold = 65534;
    shouldSendData = true;
    timer->start();
    qDebug()<<"�޸ĵ�ʱ������" << ms;
}



int RadarSerialThread::replayTrigger(QString filename) {

    //qDebug() << "�طÿ�ʼʱ�����ֵ��" << frameThreshold;
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "failed to open the file";
        return 1;
    }
    QTextStream in(&file);
    while (!in.atEnd()) {
        //qDebug() << "ѭ����ʼʱ�����ֵ��"<< frameThreshold;
        QString line = in.readLine();
        QString processedLine = line.mid(17).remove(QChar(' '));
        //qDebug() << processedLine;
        QByteArray byteArray = QByteArray::fromHex(processedLine.toUtf8());
        replayframes.append(byteArray.toHex().toUpper());
        
    }
    file.close();

    //��ȡ�ط��������
    shouldReplay = true;
    frameThreshold = 1;
    timer->setInterval(30);
    timer->start();
    return 0;
}

int RadarSerialThread::getThreshold() {
    return frameThreshold;
}


void RadarSerialThread::replayData() {

    //lock.lock();
    qDebug() << "replayframes.size():"<<replayframes.size();
    if (replayframes.size()>0 && shouldReplay) {

        //����
        emit simulate();
        QByteArray hexdata = replayframes.takeFirst();
        qDebug() << "This hexData:" << hexdata;
        QByteArray frameHead = hexdata.mid(0, 4);
        qDebug() << "This frameHead:" << frameHead;
        if (frameHead == "55AA") {
            //"55AA"˵����֡ͷ��������һ����֡
            qDebug() << "Last intack frame:" << completeFrameBuffer;
            if (completeFrameBuffer != "") {
                QString coordinates;
                QString lastFrameTime;
                qint16 lastNum;
                QString processedData = processReceivedData(completeFrameBuffer, coordinates, lastFrameTime, lastNum);
                if (processedData != "failed") {
                    qDebug() << "coordinates____:" << coordinates;
                    //���͸�allopenglwidget����
                    pointCloudBuffer.enqueue(coordinates);
                    replaySliderFrames.append(coordinates);
                    num_buffer.append(lastNum);
                    //���֡���ۻ�����ֵ��or��ʱ���������ˣ��򽫵���װ�벢����
                    /*if (pointCloudBuffer.size() > frameThreshold || shouldSendData)*/ 
                    {

                        if (shouldSendData) {
                            //�����ʱ���ʱ����������ɶҲ���ɣ�ֱ�ӷ��͵�ǰ���е�֡������յ�ǰ�����ۻ�֡
                            qDebug() << "����000";
                            //QVector<QString> vector(pointCloudBuffer.begin(), pointCloudBuffer.end());
                            emit pointCloundReady(pointCloudBuffer);
                            qDebug() << "����1111";
                            //Ȼ�����
                            pointCloudBuffer.clear();
                        }
                        else if (pointCloudBuffer.size() >= frameThreshold){
                            //�����֡�ۻ���������ȥ������������֡
                            while (pointCloudBuffer.size() > frameThreshold) {
                                pointCloudBuffer.dequeue();
                            }
                            //Ȼ���ٷ���
                            //QVector<QString> vector(pointCloudBuffer.begin(), pointCloudBuffer.end());
                            emit pointCloundReady(pointCloudBuffer);
                        }


                        //shouldSendData = false;
                    }

                    //qDebug() << "framethreashold:" << frameThreshold;

                    completeFrameBuffer = hexdata;
                }
                //����ź���������UI�й۲�֡�ģ����͸�textbrowser����Ŀǰ�����õ�
                emit newDataAvailable(completeFrameBuffer);

            }
            else {
                completeFrameBuffer += hexdata;
            }
        }
        else if (frameHead == "4154")
        {
            if (hexdata == "41542B4F4B3D310D0A")
            {
                //m_dataBuffer.append(hexdata);
                m_serial->write("AT+PROG=02\n");
                qDebug() << "sending:AT+PROG=02\n" << frameHead;
                emit newDataAvailable(hexdata);
            }
            else if (hexdata == "41542B4F4B3D30320D0A") {
                //m_dataBuffer.append(hexdata);
                m_serial->write("AT+START\n");
                qDebug() << "sending:AT+START\n" << frameHead;
                emit newDataAvailable(hexdata);
            }
        }
        else {
            completeFrameBuffer += hexdata;
        }

    }
    if (replayframes.size() == 0) {
        shouldReplay = false;
        frameThreshold = 1;
    }
    //lock.unlock();
}


void RadarSerialThread::replaySlider(int percent){
    int k = int(replaySliderFrames.size() * (percent / 100.0));
    qDebug() << "k:" << k;
    if(replaySliderFrames.size()>0){
        
        if(k>= frameThreshold){
            QVector<QString> subVector= replaySliderFrames.mid(k - frameThreshold, frameThreshold);
    

            QQueue<QString> subQueue;
            foreach(const QString & item, subVector) {
                subQueue.enqueue(item);
            }
            //= QQueue<QString>::fromVector(subVector);
            //QVector<QString> vector(subQueue.begin(), subQueue.end());
            emit pointCloundReady(subQueue);



        }
        else{
            QVector<QString> subVector = replaySliderFrames.mid(0, k);
            QQueue<QString> subQueue;
            //QQueue<QString> subQueue;
            foreach(const QString & item, subVector) {
                subQueue.enqueue(item);
            }
            //= QQueue<QString>::fromVector(subVector);
            // 
            //emit pointCloundReady(subQueue);
            //QVector<QString> vector(subQueue.begin(), subQueue.end());
            emit pointCloundReady(subQueue);
        
        }


    }

}


QString RadarSerialThread::replayProcess(QByteArray hexdata) {
    //qDebug() << "This hexData:" << hexdata;
    QByteArray frameHead = hexdata.mid(0, 4);
    //qDebug() << "This frameHead:" << frameHead;
    if (frameHead == "55AA") {
        //"55AA"˵����֡ͷ��������һ����֡
        //qDebug() << "Last intack frame:" << completeFrameBuffer;
        if (completeFrameBuffer != "") {
            QString coordinates;
            QString lastFrameTime;
            qint16 lastNum;
            QString processedData = processReceivedData(completeFrameBuffer, coordinates, lastFrameTime, lastNum);
            if (processedData != "failed") {
                //qDebug() << "coordinates____:" << coordinates;
                //���͸�allopenglwidget����
                return coordinates;
                //pointCloudBuffer.enqueue(coordinates);
                //num_buffer.enqueue(lastNum);
                //���֡���ۻ�����ֵ��or��ʱ���������ˣ��򽫵���װ�벢����
                /*if (pointCloudBuffer.size() > frameThreshold || shouldSendData)*/

                //qDebug() << "framethreashold:" << frameThreshold;

                completeFrameBuffer = hexdata;
            }
            //����ź���������UI�й۲�֡�ģ����͸�textbrowser����Ŀǰ�����õ�
            //emit newDataAvailable(completeFrameBuffer);

        }
        else {
            completeFrameBuffer += hexdata;
        }
    }
    else if (frameHead == "4154")
    {
        //if (hexdata == "41542B4F4B3D310D0A")
        //{
        //    //m_dataBuffer.append(hexdata);
        //    m_serial->write("AT+PROG=02\n");
        //    qDebug() << "sending:AT+PROG=02\n" << frameHead;
        //    emit newDataAvailable(hexdata);
        //}
        //else if (hexdata == "41542B4F4B3D30320D0A") {
        //    //m_dataBuffer.append(hexdata);
        //    m_serial->write("AT+START\n");
        //    qDebug() << "sending:AT+START\n" << frameHead;
        //    emit newDataAvailable(hexdata);
        //}
    }
    else {
        completeFrameBuffer += hexdata;
    }

    return "";
}