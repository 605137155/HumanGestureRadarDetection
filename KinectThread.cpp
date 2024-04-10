#define _CRT_SECURE_NO_WARNINGS
#include"KinectThread.h"
#include<Kinect.h>
#include<fstream>
#include<iostream>
#include<ctime>
#include<chrono>
#include<conio.h>
#include<iomanip>
#include<sstream>
#include<QOpenGLWidget>
#include<QOpenGLFunctions>
#include<qdebug.h>
#include<opencv2/opencv.hpp>

string a;
using namespace std;
#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
		a +=error;\
		qDebug()<<QString::fromStdString(a);	\
		emit dataShow(QString::fromStdString(a)); \
        exit(1);                                                                                         \
    }     

ostream& operator<<(ostream& rOS, const CameraSpacePoint& rPos)
{
	rOS << "(" << rPos.X << "/" << rPos.Y << "/" << rPos.Z << ")";
	return rOS;
}

uint64_t TmToMilliseconds(std::tm& timeStruct) {
	std::time_t timeT = std::mktime(&timeStruct);
	return static_cast<uint64_t>(timeT) * 1000;
}

std::tm MillisecondsToTm(uint64_t milliseconds) {
	std::time_t timeT = static_cast<std::time_t>(milliseconds / 1000);
	std::tm* timeStruct = std::localtime(&timeT);
	return *timeStruct;
}


string KinectThread::generateFilenameBasedOnUTC() {
	// 获取当前世界时间
	std::time_t currentTime = std::time(nullptr);
	std::tm* localTime = std::localtime(&currentTime);
	// 使用 std::ostringstream 创建文件名
	std::ostringstream filenameStream;
	filenameStream << "output_Body_azure_"
		<< (localTime->tm_year + 1900) << "-"
		<< std::setw(2) << std::setfill('0') << (localTime->tm_mon + 1) << "-"
		<< std::setw(2) << std::setfill('0') << localTime->tm_mday << "_"
		<< std::setw(2) << std::setfill('0') << localTime->tm_hour << "-"
		<< std::setw(2) << std::setfill('0') << localTime->tm_min << "-"
		<< std::setw(2) << std::setfill('0') << localTime->tm_sec
		<< ".txt";  // 可以根据需要更改扩展名
	return filenameStream.str();

}


KinectThread::KinectThread(QObject* parent) : QThread(parent), m_stop(false), bodies(0), tracker(NULL){
	tracker = NULL;
	//transformation = trans;
	device = NULL;

	//写文件的子线程
	fs = new FileSaveObject();
	t = new QThread();
	fs->moveToThread(t);
	connect(this, &KinectThread::writeFile, fs, &FileSaveObject::saveFileForKinect);

	//写深度点云的子线程
	ds = new FileSaveObject();
	t2 = new QThread();
	ds->moveToThread(t2);
	connect(this, &KinectThread::writeFileForDepth, ds, &FileSaveObject::saveFileForKinect);
}
KinectThread::~KinectThread(){
	if (t) {
		t->quit();
		t->wait();
		t2->quit();
		t2->wait();
		delete fs;
		delete t;
		delete ds;
		delete t2;
		//t->deleteLater();
		//这里知道什么时候该删除，暂停后不会有循环事件，故直接delete才能不会泄露内存。
	}
}

void KinectThread::run() {
	t->start();
	t2->start();

	m_stop = 0;
	VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");
	// Start camera. Make sure depth camera is enabled.
	k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
	deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_1080P;
	deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_30;
	deviceConfig.synchronized_images_only = true;//
	VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

	k4a_calibration_t sensor_calibration;
	VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensor_calibration),
		"Get depth camera calibration failed!");

	//tracker = NULL;
	k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;

	if (trackerMode == "CPU")
			tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_CPU;
	else if (trackerMode == "GPU")
			tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU;
	else if (trackerMode == "GPU_CUDA")
			tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU_CUDA;
	else if (trackerMode == "GPU_TENSORRT")
			tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU_TENSORRT;
	else if (trackerMode == "GPU_DIRECTML")
			tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU_DIRECTML;
	
	VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker), "Body tracker initialization failed!");
	//k4a_transformation_depth_image_to_point_cloud
	k4a_transformation_t transformation = k4a_transformation_create(&sensor_calibration);


	// Enter main loop
	size_t uFrameCount = 0;

	//定义要保存的数据
	vector<string> dataToSave;
	vector<string> depthToSave;
	
	//世界时间
	auto now = std::chrono::system_clock::now();
	std::time_t t = std::chrono::system_clock::to_time_t(now);
	std::tm tm = *std::localtime(&t);
	auto initialMs = std::chrono::time_point_cast<std::chrono::milliseconds>(now).time_since_epoch().count();
	uint64_t fullInitialTimeMs = TmToMilliseconds(tm) + (initialMs % 1000);
	INT64 InitialKinectTime = NULL;
	uint64_t InitialAzureKinectTime = NULL;
	//要保存的文件名
	filename = generateFilenameBasedOnUTC();
	filename = folderPath.toStdString() + "/" + filename;
	qDebug() << QString::fromStdString(filename);
	depthfilename = "depth" + generateFilenameBasedOnUTC();
	depthfilename = folderPath.toStdString() + "/" + depthfilename;
	qDebug() << QString::fromStdString(depthfilename);
	// 保存当前帧所有body的变量
	QVector<QVector<QVector<GLfloat>>> allBodyFrameData;
	int i = 0;
	int frames = 0;
	while (!m_stop.load())
	{	

		
		//emit dataShow(QString::number(i++));
		//奥比中光采集
		k4a_capture_t sensor_capture;
		k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);
		
		if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
		{


			//获取各种图像
			k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);
			k4a_image_t colorimage = k4a_capture_get_color_image(sensor_capture);
			k4a_image_t depthimage = k4a_capture_get_depth_image(sensor_capture);
			k4a_capture_release(sensor_capture); // Remember to release the sensor capture once you finish using it
			if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
			{
				// It should never hit timeout when K4A_WAIT_INFINITE is set.
				printf("Error! Add capture to tracker process queue timeout!\n");
				break;
			}
			else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
			{
				//qDebug() << "at there!";
				printf("Error! Add capture to tracker process queue failed!\n");
				break;
			}


			//获取时间戳
			k4abt_frame_t body_frame = NULL;
			k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
			uint64_t timeThisframe = k4abt_frame_get_system_timestamp_nsec(body_frame);
			qDebug() << "timestamp:" << timeThisframe;
			// Azure先保存一个时间戳
			//get timestamp
			if (InitialAzureKinectTime == NULL) {
				InitialAzureKinectTime = timeThisframe;
			}
			uint64_t updatedFullTimeMs = fullInitialTimeMs + (timeThisframe - InitialAzureKinectTime) / 1000000;
			// 获取更新后的 std::tm 结构体和毫秒
			std::tm updatedTm = MillisecondsToTm(updatedFullTimeMs);
			uint64_t updatedMs = updatedFullTimeMs % 1000;
			int FrameHour = updatedTm.tm_hour;
			int FrameMinute = updatedTm.tm_min;
			int FrameSecond = updatedTm.tm_sec;
			uint64_t FrameMillisecond = updatedMs;
			

			

			//采集深度点云
			k4a_image_t pointCloudImage;
			int widthDepth = k4a_image_get_width_pixels(depthimage);
			int heightDepth = k4a_image_get_height_pixels(depthimage);
			k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, widthDepth, heightDepth, widthDepth*3*(int)sizeof(int16_t), &pointCloudImage);
			VERIFY(k4a_transformation_depth_image_to_point_cloud(transformation,
				depthimage,
				K4A_CALIBRATION_TYPE_DEPTH,
				pointCloudImage), "Transform depth image to point clouds failed!");
			int width = k4a_image_get_width_pixels(pointCloudImage);
			int height = k4a_image_get_height_pixels(pointCloudImage);
			int16_t* pointCloudImageBuffer = (int16_t*)k4a_image_get_buffer(pointCloudImage);
			QVector<QVector3D> depthPointClounds; //用于openglwidget显示
			int pointNum = 0;
			stringstream depthStream;
			depthStream << std::setw(2) << std::setfill('0') << FrameHour << ":" << std::setw(2)
				<< std::setfill('0') << FrameMinute << ":" << std::setw(2) << std::setfill('0')
				<< FrameSecond << "." << std::setw(3) << std::setfill('0') << FrameMillisecond << ' ';
			for (int h = 0; h < height; h++)
			{
				for (int w = 0; w < width; w++)
				{
					int pixelIndex = h * width + w;
					k4a_float3_t position = {
						static_cast<float>(pointCloudImageBuffer[3 * pixelIndex + 0]),
						static_cast<float>(pointCloudImageBuffer[3 * pixelIndex + 1]),
						static_cast<float>(pointCloudImageBuffer[3 * pixelIndex + 2]) };
					//invalid z-depth value
					//qDebug() << "max:"<<maxDepth<<"min:"<<minDepth; || position.v[2] > maxDepth ||  position.v[2] < minDepth
					
					//qDebug() << "pointclound";
					float z = position.v[2] * 0.001f;
					if (z == 0 || z > maxDepth || z < minDepth)
					{
						continue;

					}
					float x = -position.v[0] * 0.001f;
					if ( x>maxX || x < minX )
					{
						continue;

					}
					float y = -position.v[1] * 0.001f;
					if (y<minY||y>maxY)
					{
						continue;

					}
					if(saveDepth)
						depthStream<<std::to_string(x) + " " + std::to_string(y)+ " " +std::to_string(z) + " ";
					pointNum++;
					//qDebug() << "the"<<frames<<"帧的"<<"点"<<pointNum<<"的值："<<x<<y<<z;
					QVector3D a = QVector3D(x, y, z);
					depthPointClounds.append(a);
				}
			}
			//qDebug() << QString::fromStdString(depthStream.str());
			depthToSave.push_back(depthStream.str());
			frames++;

			//先发送colorimage
			uint8_t* buffer = k4a_image_get_buffer(colorimage);
			int bufferSize = k4a_image_get_size(colorimage);
			k4a_image_format_t iformat = k4a_image_get_format(colorimage);
			qDebug() << "bufferSize:" << bufferSize <<"format:"<<iformat;
			cv::Mat rawData(1, bufferSize, CV_8UC1, (void*)buffer);
			cv::Mat cvImage = cv::imdecode(rawData, cv::IMREAD_COLOR);
			QImage qImage = QImage(cvImage.data, cvImage.cols, cvImage.rows, cvImage.step,QImage::Format_RGB888).rgbSwapped();
			//qDebug() << "width:" << qImage.width() << "height" << qImage.height(); //1920*1080
			QPixmap p = QPixmap::fromImage(qImage);
			emit colorFrameReady(p);
			k4a_image_release(colorimage);

			
			stringstream ss;
			// 保存当前帧所有body的变量
			QVector<QVector<QVector<GLfloat>>> allBodyFrameData;
			QVector<uint32_t> allBodyIndexs;

			if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
			{	
				// Successfully popped the body tracking result. Start your processing
				uint32_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
				printf("%d bodies are detected!, time: %lld\n", num_bodies, timeThisframe);
				//遍历所有的人
				//如果识别到骨骼，再保存数据，不然不保存
				if (num_bodies > 0)
					ss << std::setw(2) << std::setfill('0') << FrameHour << ":" << std::setw(2) 
					<< std::setfill('0') << FrameMinute << ":" << std::setw(2) << std::setfill('0') 
					<< FrameSecond << "." << std::setw(3) << std::setfill('0') << FrameMillisecond << ' ';
				
				for (uint32_t i = 0; i < num_bodies; i++) {
					QVector<QVector<GLfloat>> bodyFrameData;
					k4abt_skeleton_t skeleton;
					k4abt_frame_get_body_skeleton(body_frame, i, &skeleton);
					//遍历所有关节
					for (int k = 0; k < 32; k++) {
						float b_x = -(skeleton.joints[k].position.xyz.x)/1000.0f;
						float b_y = -(skeleton.joints[k].position.xyz.y)/1000.0f;
						float b_z = (skeleton.joints[k].position.xyz.z)/1000.0f;
						//这里可以开始对关节进行操作
						//保存数据到字符流
						//qDebug() <<"检测到人了:" << b_x << b_y << b_z;
						ss << b_x << ' ' << b_y << ' ' << b_z << ' ' << k << ' ';
						//保存数据到bodyFrameData
						bodyFrameData.append({b_x, b_y, b_z});
						emit dataShow(QString::number(b_x)+QString(" and ") + QString::number(b_y));
					}
					allBodyFrameData.append(bodyFrameData);
					//body的idx，这里用另一个vector保存
					uint32_t id = k4abt_frame_get_body_id(body_frame, i);
					allBodyIndexs.append(id);
					bodies++;
					qDebug() << "bodies:"<<bodies;
				}
				//这里保存完该帧的所有body后，再将这些点保存或者发送到窗口显示。
				if (allBodyFrameData.size() > 0)
				{
					qDebug() << "ss.str" << QString::fromStdString(ss.str());
					dataToSave.push_back(ss.str());
					emit bodyFrameReady(allBodyFrameData, allBodyIndexs, depthPointClounds);
				}
				else
				{
					emit bodyFrameReady(QVector<QVector<QVector<GLfloat>>>(), QVector<uint32_t>(), depthPointClounds);
				}
				

				//如果数量够多，则写入.TXT文件
				if (bodies > 10)
				{
					qDebug() << "save data to file ";
					emit writeFile(filename, dataToSave);  //这里改为调用，而不是信号机制来保存
					//fs->saveFileForKinect(filename, dataToSave);

					dataToSave.clear();
					bodies = 0;
				}
				if (frames > 10 && saveDepth)
				{
					qDebug() << "save depthdata to file ";
					emit writeFileForDepth(depthfilename, depthToSave);
					qDebug() << "save depthdata to file over!";
					//ds->saveFileForKinect(depthfilename, depthToSave); //这里直接调用，会耗费很多时间，想办法不要直接调用。
					depthToSave.clear();
					frames = 0;
				}


				k4abt_frame_release(body_frame); // Remember to release the body frame once you finish using it
				k4a_image_release(depthimage);
				k4a_image_release(pointCloudImage);

			}
			else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
			{
				//  It should never hit timeout when K4A_WAIT_INFINITE is set.
				printf("Error! Pop body frame result timeout!\n");
				break;
			}
			else
			{
				printf("Pop body frame result failed!\n");
				break;
			}
		}

		else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
		{
			// It should never hit time out when K4A_WAIT_INFINITE is set.
			printf("Error! Get depth frame time out!\n");
			break;
		}
		else
		{
			printf("Get depth capture returned error: %d\n", get_capture_result);
			break;
		}
	}

	
}


void KinectThread::stopKinect() {
	stop();
}

void KinectThread::setMinDepth(float min) {

	lock.lock();
	minDepth = min;
	lock.unlock();
}

void KinectThread::setMaxDepth(float max) {
	lock.lock();
	maxDepth = max;
	lock.unlock();
}

void KinectThread::setMinX(float min) {

	lock.lock();
	minX = min;
	lock.unlock();
}

void KinectThread::setMaxX(float max) {
	lock.lock();
	maxX = max;
	lock.unlock();
}

void KinectThread::setMinY(float min) {

	lock.lock();
	minY = min;
	lock.unlock();
}

void KinectThread::setMaxY(float max) {
	lock.lock();
	maxY = max;
	lock.unlock();
}