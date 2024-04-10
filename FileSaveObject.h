#pragma once
#ifndef FILESAVEOBJECT_H
#define FILESAVEOBJECT_H
#include <QObject>
#include <QQueue>

class FileSaveObject : public QObject
{
    Q_OBJECT
public:
    explicit FileSaveObject(QObject* parent = nullptr);
    ~FileSaveObject();



public slots:
    void saveFile(QString filename, QVector<QString> HexDatas, QVector<QString> PointDatas, QVector<qint16> nums);
    void saveFileForKinect(std::string filename, std::vector<std::string> dataToSave);

};

#endif // FILESAVEOBJECT_H
