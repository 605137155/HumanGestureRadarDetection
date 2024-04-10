#include "FileSaveObject.h"
#include <QFile>
#include<QDebug>
#include<iomanip>
#include<sstream>
#include<iostream>
#include<fstream>
using namespace std;
#pragma execution_character_set("utf-8")

FileSaveObject::FileSaveObject(QObject* parent){}

FileSaveObject::~FileSaveObject(){}

void FileSaveObject::saveFile(QString filename, QVector<QString> HexDatas,
	QVector<QString> PointDatas, QVector<qint16> nums) {
	QStringList fn = filename.split("\n");
    QFile file(fn[0] +"/" + "HEX" + fn[1]);

	
	QFile file_raw_cords(fn[0] +"/" + "Raw_" + fn[1]);
	QFile file_correct_cords(fn[0] +"/" +"Correct_" + fn[1]);
	//qDebug() << "filename_:"<<filename;
    if (file.open(QIODevice::ReadWrite| QIODevice::Append | QIODevice::Text)&&
		file_raw_cords.open(QIODevice::ReadWrite | QIODevice::Append | QIODevice::Text)&&
		file_correct_cords.open(QIODevice::ReadWrite | QIODevice::Append | QIODevice::Text)) {
        //out.setCodec("UTF-8");
        /*QString send = QString::fromLocal8Bit("[10:48:58.563]������41 54 2B 53 54 41 52 54 0A �� ");
        file.write(send.toLocal8Bit() + "\n");*/
		//HEX���ݱ���
        for (int i = 0; i < HexDatas.size(); i++) {
            file.write(HexDatas[i].toUtf8() + "\n");
            //out << m_stringBuffer[i] + "\n";
        }

		//��ά�������ݱ���
		std::vector<std::vector<float>> matrix = { {-1.08930402f,-0.00126659f,-0.05982419f},
					{0.04921241f,1.10069102f,-0.24182825f},
					{0.06662979f,0.21614036f,0.95915324f} };
		std::vector<float> vector2 = { 0.00788764f,-0.11529322f,0.08330746f };
		for(QString& data:PointDatas) {
			//��ÿ�е�ʱ����͵�����ֿ�
			QStringList lines = data.split("\n");
			int size = lines.size();//��һ����ʱ��,ʣ��Ϊ������
			QString timestamp = lines[0];//ʱ��
			QString cords_raw;
			QString cords_correct;
			//������
			for (int i = 1; i < size; i++) {//�ӵ�һ���㿪ʼ
				QStringList cord = lines[i].split(" ");
				if (cord.size() == 3) {
					std::vector<float> result(3, 0.0f);
					std::vector<float> vector = { cord[0].toFloat(),cord[2].toFloat(),cord[1].toFloat() };//���״��y��z�Ե���Ӧ������ռ��y��z
					for (int i = 0; i < 3; i++)
						for (int j = 0; j < 3; j++) {
							result[i] += matrix[i][j] * vector[j];
						}
					for (int i = 0; i < 3; ++i) {
						result[i] = result[i] + vector2[i];
					}
					//if (result[0] >= 4.0 || result[0] <= -4.0)
					//	continue;
					//if (result[1] >= 1.5 || result[1] <= -2.0)
					//	continue;
					//if (result[2] >= 10.0 || result[2] <= 0.0)
					//	continue;
					cords_raw += (QString::number(cord[0].toFloat()) + " " + QString::number(cord[2].toFloat()) + " " + QString::number(cord[1].toFloat()) + " ");
					cords_correct += (QString::number(-result[0]) + " " + QString::number(result[1]) + " " + QString::number(result[2]) + " ");
				}
			}
			//δ�궨��������
			file_raw_cords.write(timestamp.toUtf8() +" "+ cords_raw.toUtf8() + "\n");
			//�궨�ĵ�������
			file_correct_cords.write(timestamp.toUtf8() +" "+ cords_correct.toUtf8() + "\n");

		}
		

        file.close();
		file_raw_cords.close();
		file_correct_cords.close();

    }
    qDebug() << "File Saved.";

}


void FileSaveObject::saveFileForKinect(std::string filename, std::vector<std::string> dataToSave){

	//qDebug() << "start to save!";
	std::ofstream file(filename, std::ios::app); //std::ios::app��ʾһ�����ļ���ʱ�����ļ��Ĵ�ģʽ�ı�־����app����append����д����ʾ׷��
	//qDebug() << "start to save!";
	//����ļ��Ƿ��
	if (file.is_open()) {
		for (const string& line : dataToSave) {
			file << line << endl;
		}
		file.close();
		std::cout << "Kinect������" << endl;

	}
	else {
		std::cerr << "failed to open the file" << endl;
	}

}