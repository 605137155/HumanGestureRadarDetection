#define _CRT_SECURE_NO_WARNINGS
#include"KinectColorThread.h"
#include<Kinect.h>
#include<iostream>
#include<ctime>
#include<chrono>
#include<conio.h>
#include<iomanip>
#include<sstream>
#include<QOpenGLWidget>
#include<QOpenGLFunctions>
#include<qdebug.h>

using namespace std;

string KinectColorThread::generateFilenameBasedOnUTC() {
	// ��ȡ��ǰ����ʱ��
	std::time_t currentTime = std::time(nullptr);
	std::tm* localTime = std::localtime(&currentTime);
	// ʹ�� std::ostringstream �����ļ���
	std::ostringstream filenameStream;
	filenameStream << "output_Body_"
		<< (localTime->tm_year + 1900) << "-"
		<< std::setw(2) << std::setfill('0') << (localTime->tm_mon + 1) << "-"
		<< std::setw(2) << std::setfill('0') << localTime->tm_mday << "_"
		<< std::setw(2) << std::setfill('0') << localTime->tm_hour << "-"
		<< std::setw(2) << std::setfill('0') << localTime->tm_min << "-"
		<< std::setw(2) << std::setfill('0') << localTime->tm_sec
		<< ".txt";  // ���Ը�����Ҫ������չ��
	return filenameStream.str();

}


void KinectColorThread::run() {

	m_stop = 0;
	// 2a. Get frame source
	IColorFrameSource* pFrameSource = nullptr;
	pSensor->get_ColorFrameSource(&pFrameSource);


	// 3a. get frame reader
	IColorFrameReader* pFrameReader = nullptr;
	pFrameSource->OpenReader(&pFrameReader);

	//release frame source
	pFrameSource->Release();
	pFrameSource = nullptr;
	qDebug() << "��ʼrgbcolor���";

	while (!m_stop.load())
	{


		// 4a. Get last frame
		IColorFrame* pFrame = nullptr;

		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
		{
			// get color data
			QByteArray buffer(BUFFERSIZE, 0);
			pFrame->CopyConvertedFrameDataToArray(BUFFERSIZE, (BYTE*)buffer.data(), ColorImageFormat_Bgra);
			QImage image((uchar*)buffer.constData(), WIDTH, HEIGHT, QImage::Format_ARGB32);
			emit colorFrameReady(QPixmap::fromImage(image));
			// 4e. release frame
			pFrame->Release();
		}
	}



	// 3b. release frame reader
	std::cout << "Release frame reader" << endl;
	pFrameReader->Release();
	pFrameReader = nullptr;


}


void KinectColorThread::stopKinect() {
	stop();
}

