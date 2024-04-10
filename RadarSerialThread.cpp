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
    m_serial->setPortName("COM3");  // 这里修改为你的串口名称
    m_serial->setBaudRate(2000000);
    m_serial->setDataBits(QSerialPort::Data8);
    m_serial->setStopBits(QSerialPort::OneStop);
    m_serial->setParity(QSerialPort::NoParity);
    m_serial->setFlowControl(QSerialPort::NoFlowControl);
    //m_serial->setReadBufferSize(16*1024);

    //计时器，如果需要时间累积帧的话，则修改计时器的时间即可
    timer = new QTimer(this);
    timer->setInterval(1000);
    connect(timer, &QTimer::timeout, this, &RadarSerialThread::onTimerTimeout);

    //写文件的子线程
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
        qDebug()<<"退出线程";
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
        QString send = QString::fromUtf8("[10:48:58.563]发→◇41 54 2B 53 54 41 52 54 0A □ ");
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
    qDebug() << "打开雷达!!";
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
            //"55AA"说明是帧头，则处理上一个满帧
            qDebug() << "Last intack frame:" << completeFrameBuffer;
            if (completeFrameBuffer != "") {
                QString coordinates;
                QString lastFrameTime;
                qint16 lastNum;
                QString processedData = processReceivedData(completeFrameBuffer, coordinates, lastFrameTime, lastNum);
                if (processedData != "failed") {
                    //m_dataBuffer.append(hexdata);
                    //每两个字符空一个空格, 与sscom保存格式一致
                    QByteArray spaceHexData;
                    for (int i = 0; i < completeFrameBuffer.size(); i += 2) {
                        QByteArray hexPair = completeFrameBuffer.mid(i, 2);
                        spaceHexData += hexPair + ' ';
                    }
                    QString stringData = QString::fromUtf8(spaceHexData);
                    m_stringBuffer.append(lastFrameTime + QString::fromUtf8("收←◆") + stringData);
                    m_dataBuffer.append(lastFrameTime +"\n"+ coordinates);
                    num_buffer.append(lastNum);
                    //采集数据内存控制 采集超过10000个点就把数据写进文档里
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
                    //发送给allopenglwidget窗口
                    pointCloudBuffer.enqueue(coordinates);

                    //如果帧数累积到阈值，or定时器被触发了，则将点云装入并发送
                    if (pointCloudBuffer.size() > frameThreshold||shouldSendData) {
                        if (shouldSendData)  
                            qDebug() << "到点发送";
                        while (pointCloudBuffer.size() > frameThreshold) {
                            pointCloudBuffer.dequeue();
                        }

                        //QVector<QString> vector(pointCloudBuffer.begin(), pointCloudBuffer.end()); //按动态数据方式存储和显示数据，但据说这样会浪费一些性能，故不用
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
                //这个信号是用来在UI中观察帧的，发送给textbrowser，但目前不会用到
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
    //qDebug() << "start to analysis data！";
    //qDebug() << "data size:"<<data.size();
    // 解析帧头  
    QByteArray frameHead = data.mid(0, 4);
    //qDebug() << data;
    //if (frameHead != "55AA") {
    //    
    //    //qDebug() << "head wrong:"<<frameHead;
    //    return "failed";
    //}
    //qDebug() << "frameHead:" << frameHead;
    // 解析帧长度  
    //if (data.size() < 24) {
    //    //qDebug() << "lengthwrong";
    //    return "failed";
    //}
    //qDebug() << "FrameLength(hex):" << data.mid(4, 8);//重新解释转换
    quint32 frameLength = qFromLittleEndian<quint32>(reinterpret_cast<const uchar*>(QByteArray::fromHex(data.mid(4, 8)).constData()));
    //qDebug() << frameLength;
    //qDebug() << "FrameLength:" << frameLength;
    quint16 TimeInterval = qFromBigEndian<quint16>(reinterpret_cast<const uchar*>(QByteArray::fromHex(data.mid(12, 4)).constData())); //大端模式转换的帧时间，单位ms
    //0001 1110 0000 0000
    // 解析 TLV 个数和类型  
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
    //wTime += "[" + QString(current.toString("yyyy - MM - dd HH : mm:ss.zzz")) + "]收←◆";
    wTime += "[" + QString(current.toString("HH:mm:ss.zzz")) + "]";
    worldTime = wTime;
    //outputText += QString(frameHead) + ' '+ QString::number(frameLength) + ' '+ QString::number(TimeInterval) +' ' + QString(NumTLVs) + ' ' + QString(type) + ' ' + QString::number(targetNum);
    QByteArray tlvData = data.mid(24);
    // 处理 TLV 数据
    if (tlvData.size() != targetNum * 18) {
        //qDebug() << "Fail frame";
        //return "failed";
    }
    else
        qDebug() << "Success frame";
    int count_point = 0;
    QString outputText;
    //避免有超出targetnum的数量的点没有显示（为了显示更多的点。如果不需要，则将10000改回targetnum）
    for (int i = 0; i < 10000; i++) {
        QByteArray idx1 = tlvData.mid(i * 18, 4);
        QByteArray idx2 = tlvData.mid(i * 18 + 4, 2);
        QByteArray idx3 = tlvData.mid(i * 18 + 6, 2);
        QByteArray idx4 = tlvData.mid(i * 18 + 8, 2);
        QByteArray powABS = tlvData.mid(i * 18 + 10, 8);

        //重新解释转换（非常危险，非常小心，需要确保完全了解要进行的位操作和内存布局）   一般用于直接操作二进制or内存布局的情况，比如需要将一个数据的二进制表示转换为另一种类型or以某种方式操纵内存中的位，第二种与底层硬件编程有关，不熟悉也不赘了解
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
        QString send = QString::fromLocal8Bit("[10:48:58.563]发→◇41 54 2B 53 54 41 52 54 0A □ ");
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
    // 获取当前世界时间
    std::time_t currentTime = std::time(nullptr);
    std::tm* localTime = std::localtime(&currentTime);
    // 使用 std::ostringstream 创建文件名
    std::ostringstream filenameStream;
    filenameStream << "output_Radar_"
        << (localTime->tm_year + 1900) << "-"
        << std::setw(2) << std::setfill('0') << (localTime->tm_mon + 1) << "-"
        << std::setw(2) << std::setfill('0') << localTime->tm_mday << "_"
        << std::setw(2) << std::setfill('0') << localTime->tm_hour << "-"
        << std::setw(2) << std::setfill('0') << localTime->tm_min << "-"
        << std::setw(2) << std::setfill('0') << localTime->tm_sec
        << ".txt";  // 可以根据需要更改扩展名
    return filenameStream.str();
}

void RadarSerialThread::setFrameThreshold(qint16 Threshold) {
    qDebug() << "已锁";
    lock.lock();
    frameThreshold = Threshold;
    if (shouldReplay) {
        timer->setInterval(1);
        shouldSendData = false;
    }
    else
    {
        timer->setInterval(1000000);//让时间间隔变得很大，而且暂停了计时器，使其不会被触发，使得shouldSendData一直为false
        //timer->stop();
    }
    lock.unlock();
    qDebug() << "已解锁";
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
        qDebug() << "计时器触发了发送数据,时间间隔："<< std::time(nullptr);
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
    qDebug()<<"修改的时间间隔：" << ms;
}



int RadarSerialThread::replayTrigger(QString filename) {

    //qDebug() << "回访开始时候的阈值：" << frameThreshold;
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "failed to open the file";
        return 1;
    }
    QTextStream in(&file);
    while (!in.atEnd()) {
        //qDebug() << "循环开始时候的阈值："<< frameThreshold;
        QString line = in.readLine();
        QString processedLine = line.mid(17).remove(QChar(' '));
        //qDebug() << processedLine;
        QByteArray byteArray = QByteArray::fromHex(processedLine.toUtf8());
        replayframes.append(byteArray.toHex().toUpper());
        
    }
    file.close();

    //读取回放数据完毕
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

        //处理
        emit simulate();
        QByteArray hexdata = replayframes.takeFirst();
        qDebug() << "This hexData:" << hexdata;
        QByteArray frameHead = hexdata.mid(0, 4);
        qDebug() << "This frameHead:" << frameHead;
        if (frameHead == "55AA") {
            //"55AA"说明是帧头，则处理上一个满帧
            qDebug() << "Last intack frame:" << completeFrameBuffer;
            if (completeFrameBuffer != "") {
                QString coordinates;
                QString lastFrameTime;
                qint16 lastNum;
                QString processedData = processReceivedData(completeFrameBuffer, coordinates, lastFrameTime, lastNum);
                if (processedData != "failed") {
                    qDebug() << "coordinates____:" << coordinates;
                    //发送给allopenglwidget窗口
                    pointCloudBuffer.enqueue(coordinates);
                    replaySliderFrames.append(coordinates);
                    num_buffer.append(lastNum);
                    //如果帧数累积到阈值，or定时器被触发了，则将点云装入并发送
                    /*if (pointCloudBuffer.size() > frameThreshold || shouldSendData)*/ 
                    {

                        if (shouldSendData) {
                            //如果是时间计时器触发，则啥也不干，直接发送当前所有的帧，并清空当前所有累积帧
                            qDebug() << "发送000";
                            //QVector<QString> vector(pointCloudBuffer.begin(), pointCloudBuffer.end());
                            emit pointCloundReady(pointCloudBuffer);
                            qDebug() << "发送1111";
                            //然后清空
                            pointCloudBuffer.clear();
                        }
                        else if (pointCloudBuffer.size() >= frameThreshold){
                            //如果是帧累积触发，则去除超出数量的帧
                            while (pointCloudBuffer.size() > frameThreshold) {
                                pointCloudBuffer.dequeue();
                            }
                            //然后再发送
                            //QVector<QString> vector(pointCloudBuffer.begin(), pointCloudBuffer.end());
                            emit pointCloundReady(pointCloudBuffer);
                        }


                        //shouldSendData = false;
                    }

                    //qDebug() << "framethreashold:" << frameThreshold;

                    completeFrameBuffer = hexdata;
                }
                //这个信号是用来在UI中观察帧的，发送给textbrowser，但目前不会用到
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
        //"55AA"说明是帧头，则处理上一个满帧
        //qDebug() << "Last intack frame:" << completeFrameBuffer;
        if (completeFrameBuffer != "") {
            QString coordinates;
            QString lastFrameTime;
            qint16 lastNum;
            QString processedData = processReceivedData(completeFrameBuffer, coordinates, lastFrameTime, lastNum);
            if (processedData != "failed") {
                //qDebug() << "coordinates____:" << coordinates;
                //发送给allopenglwidget窗口
                return coordinates;
                //pointCloudBuffer.enqueue(coordinates);
                //num_buffer.enqueue(lastNum);
                //如果帧数累积到阈值，or定时器被触发了，则将点云装入并发送
                /*if (pointCloudBuffer.size() > frameThreshold || shouldSendData)*/

                //qDebug() << "framethreashold:" << frameThreshold;

                completeFrameBuffer = hexdata;
            }
            //这个信号是用来在UI中观察帧的，发送给textbrowser，但目前不会用到
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