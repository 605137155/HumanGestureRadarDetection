#include "GestureForRelease.h"
#include "MyOpenGLWidget.h"
#include "RadarSerialThread.h"
#include "RadarOpenGLWidget.h"
#include "AllOpenGLWidget.h"
#include<QDebug>
#include<QLayout>
#include<QOpenGLWidget>
#include<QtDataVisualization/QScatter3DSeries>
#include<QtDataVisualization/Q3DScatter>
#include <Q3DCamera>
#include <QtDataVisualization/QCategory3DAxis>
#include <QMessageBox>
#include <QScatterDataItem>
#include <QSerialPortInfo>
#include <QFileDialog>

using namespace std;
#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
                                                                                              \
    }   


GestureForRelease::GestureForRelease(QWidget* parent) : QMainWindow(parent), socket(nullptr)
{

    ui.setupUi(this);

    // ��������Kinect�����OpenGLWidget
    kinectThread = new KinectThread(this);
    // ��ȡ�ɵ� QOpenGLWidget �ļ�����״
    QRect oldGeometry = ui.openGLWidget->geometry();
    QString oldObjectName = ui.openGLWidget->objectName();
    //����3d������Ϣ�Ĵ��ڿؼ� MyOpenGLWidget
    myOpenGLWidget = new MyOpenGLWidget(ui.centralWidget);
    QOpenGLWidget* oldOpenGLWidgetPointer;
    // ������ �µ�OpenGL С�����ļ�����״
    myOpenGLWidget->setGeometry(oldGeometry);
    myOpenGLWidget->setObjectName(oldObjectName);
    //���µ� MyOpenGLWidget �滻�ɵ� QOpenGLWidget
    oldOpenGLWidgetPointer = ui.openGLWidget;
    oldOpenGLWidgetPointer->deleteLater();
    ui.openGLWidget = myOpenGLWidget;
    kinectOpenGLWidget = myOpenGLWidget;
    ui.verticalLayout_3->addWidget(ui.openGLWidget, 0);

    //connect(kinectThread, SIGNAL(bodyFrameReady(QVector<QVector<GLfloat>>)),ui.openGLWidget,SLOT(updateBodyFrame(QVector<QVector<GLfloat>>)));  //�����̼߳�ֱ�Ӳ��������̱߳���е�һ����Ҫԭ��
    connect(kinectThread, &KinectThread::bodyFrameReady, this, &GestureForRelease::updataBodyFrame);
    connect(kinectThread, &KinectThread::colorFrameReady, this, &GestureForRelease::updateColor);
    connect(myOpenGLWidget, &MyOpenGLWidget::mouseMove, this, &GestureForRelease::updateDirection);
    connect(kinectThread, &KinectThread::dataShow, this, &GestureForRelease::onNewDataAvailable);

    //Kinect�Կ�����ѡ��
    ui.comboBox_2->addItem("CPU");
    ui.comboBox_2->setItemData(0,"SDK will use CPU only mode to run the tracker",Qt::ToolTipRole);
    ui.comboBox_2->addItem("GPU");
    ui.comboBox_2->setItemData(1, "SDK will use the most appropriate GPU mode for the operating system to run the tracker", Qt::ToolTipRole);
    ui.comboBox_2->addItem("GPU_CUDA");
    ui.comboBox_2->setItemData(2, "SDK will use ONNX Cuda EP to run the tracker", Qt::ToolTipRole);
    ui.comboBox_2->addItem("GPU_TENSORRT");
    ui.comboBox_2->setItemData(3, "DK will use ONNX TensorRT EP to run the tracker", Qt::ToolTipRole);
    ui.comboBox_2->addItem("GPU_DIRECTML");
    ui.comboBox_2->setItemData(4, "SDK will use ONNX DirectML EP to run the tracker (Windows only)", Qt::ToolTipRole);

    //�״���Ʋɼ��߳�
    radarSerialThread = new RadarSerialThread(this);
    connect(radarSerialThread, &RadarSerialThread::newDataAvailable, this, &GestureForRelease::onNewDataAvailable);
    connect(radarSerialThread, &RadarSerialThread::errorOccurred, this, &GestureForRelease::onErrorOccurred);
    connect(radarSerialThread, &RadarSerialThread::pointCloundReady, this, &GestureForRelease::updataRadarPointCloud);
    connect(radarSerialThread, &RadarSerialThread::pointCloundReady, this, &GestureForRelease::addRadarData);

    // ���������״���Ƶ�OpenGLWidget
    // ������ʾ�״���ƵĿؼ�
    QOpenGLWidget* secondOpenGLWidgetPointer;
    secondOpenGLWidgetPointer = ui.openGLWidget_2;
    // �����µ� OpenGLWidget
    radarOpenGLWidget = new RadarOpenGLWidget(ui.centralWidget);
    // ������ OpenGL С�����ļ�����״
    radarOpenGLWidget->setGeometry(ui.openGLWidget_2->geometry());
    radarOpenGLWidget->setObjectName(ui.openGLWidget_2->objectName());
    secondOpenGLWidgetPointer->deleteLater();
    ui.openGLWidget_2 = radarOpenGLWidget;
    ui.verticalLayout_4->addWidget(radarOpenGLWidget, 2);
    ui.verticalLayout_4->removeWidget(ui.label);
    ui.verticalLayout_4->addWidget(ui.label, 2);
    ui.verticalLayout_4->setStretch(1, 1);
    ui.verticalLayout_4->setStretch(2, 1);
    ui.horizontalLayout_3->setStretch(1, 1);
    ui.horizontalLayout_3->setStretch(2, 1);
    connect(radarOpenGLWidget, &RadarOpenGLWidget::mouseMove, this, &GestureForRelease::updateDirection);

    //��ɫ���RGB��ʾ�߳�
    /*kinectColorThread = new KinectColorThread(this, iKinectSensor);
    connect(kinectColorThread, &KinectColorThread::colorFrameReady, this, &GestureForRelease::updateColor);*/
    //�ؼ���������Ӧ����
    ui.label->setScaledContents(true);

    //RGB+�״������ʾ�ռ�
    // ��ȡ�ɵ� QOpenGLWidget �ļ�����״
    QRect oldGeometry_3 = ui.openGLWidget_3->geometry();
    QString oldObjectName_3 = ui.openGLWidget_3->objectName();
    //����3d������Ϣ�Ĵ��ڿؼ� MyOpenGLWidget
    allOpenGLWidget = new AllOpenGLWidget(ui.centralWidget);
    QOpenGLWidget* oldOpenGLWidgetPointer_2;
    // ������ �µ�OpenGL С�����ļ�����״
    allOpenGLWidget->setGeometry(oldGeometry_3);
    allOpenGLWidget->setObjectName(oldObjectName_3);
    //���µ� MyOpenGLWidget �滻�ɵ� QOpenGLWidget
    oldOpenGLWidgetPointer_2 = ui.openGLWidget_3;
    oldOpenGLWidgetPointer_2->deleteLater();
    ui.openGLWidget_3 = allOpenGLWidget;
    ui.verticalLayout_3->addWidget(allOpenGLWidget, 0);
    ui.verticalLayout_3->setStretch(1, 1);
    ui.verticalLayout_3->setStretch(0, 1);
    
    //�ۻ���������
    //������������
    ui.horizontalSlider->setMinimum(1);
    ui.horizontalSlider->setMaximum(30);
    ui.horizontalSlider->setValue(1);
    connect(ui.horizontalSlider, &QSlider::valueChanged, radarSerialThread, &RadarSerialThread::setFrameThreshold);
    QLabel* Label_3 = ui.label_3;
    connect(ui.horizontalSlider, &QSlider::valueChanged, [Label_3](int value) {Label_3->setText(QString::number(value)); });

    //��ʾ��ά���Ƶ�scatter
    //����һ����ά����ϵ
    QtDataVisualization::Q3DScatter* graph = new QtDataVisualization::Q3DScatter();
    //���������λ��
    graph->scene()->activeCamera()->setCameraPreset(QtDataVisualization::Q3DCamera::CameraPresetIsometricRight);
    //������Ӱ��ʾ
    graph->setShadowQuality(QtDataVisualization::QAbstract3DGraph::ShadowQualityNone);
    //���������������
    graph->axisX()->setSegmentCount(int(6));
    graph->axisY()->setSegmentCount(int(6));
    graph->axisZ()->setSegmentCount(int(6));
    /*������������ϵ������*/
    graph->activeTheme()->setType(QtDataVisualization::Q3DTheme::ThemePrimaryColors);
    QtDataVisualization::Q3DTheme* theme = graph->activeTheme();
    theme->setBackgroundEnabled(true);
    theme->setBackgroundColor(Qt::black);
    theme->setGridLineColor(Qt::white);
    theme->setLabelTextColor(Qt::black);
    QtDataVisualization::QValue3DAxis* axis_X = new QtDataVisualization::QValue3DAxis;
    axis_X->setTitle("X Axis");  // �������������
    axis_X->setTitleVisible(true); // ���ñ����Ƿ���ʾ
    axis_X->setRange(-4, 4);   // ����������ȡֵ��Χ
    //axis_X->set;
    QtDataVisualization::QValue3DAxis* axis_Y = new QtDataVisualization::QValue3DAxis;
    axis_Y->setTitle("Y Axis");  // �������������
    axis_Y->setTitleVisible(true); // ���ñ����Ƿ���ʾ
    axis_Y->setRange(-2, 1.5);   // ����������ȡֵ��Χ
    QtDataVisualization::QValue3DAxis* axis_Z = new QtDataVisualization::QValue3DAxis;
    axis_Z->setTitle("Z Axis");  // �������������
    axis_Z->setTitleVisible(true); // ���ñ����Ƿ���ʾ
    axis_Z->setRange(0, 10);   // ����������ȡֵ��Χ
    graph->setAxisX(axis_X);
    graph->setAxisY(axis_Y);
    graph->setAxisZ(axis_Z);
    //graph->setAxisX();
    graph->axisX()->setTitle(QString("X"));
    graph->axisY()->setTitle(QString("Y"));
    graph->axisZ()->setTitle(QString("Z"));
    //������Ӱ
    graph->setShadowQuality(QtDataVisualization::QAbstract3DGraph::ShadowQualityLow);
    //����һ��widget,������ϵ��ӽ�ȥ
    scatterContainer = QWidget::createWindowContainer(graph);
    QWidget* oldWidget = ui.widget;
    ui.widget = scatterContainer;
    oldWidget->deleteLater();
    //�ж��Ƿ�graph(opengl)��ʼ��
    if (!graph->hasContext()) {
        QMessageBox msgBox;
        msgBox.setText("Couldn't initialize the OpenGL context.");
        msgBox.exec();
    }
    //��container��ӵ�ˮƽ������
    ui.verticalLayout_2->addWidget(scatterContainer, 0);
    // ����ɢ��ͼ����
    QtDataVisualization::QScatterDataArray data;
    data << QVector3D(2, 1, 3) << QVector3D(1, 0, 3) << QVector3D(-2, -1, 3);
    // �������ݴ�������������ӵ�������
    proxy = new QtDataVisualization::QScatterDataProxy();
    proxy->addItems(data);
    // ����ϵ�У��������ݴ�����ӵ�ϵ����
    series = new QtDataVisualization::QScatter3DSeries(proxy);
    series->setItemSize(0.05f);
    // ����ϵ�е���ɫ����
    QList<QLinearGradient> gradient;
    QLinearGradient gd1;
    gd1.setColorAt(0, Qt::white);
    gd1.setColorAt(0.2, Qt::white);
    gd1.setColorAt(0.3, Qt::red);
    gd1.setColorAt(0.4, Qt::red);
    gd1.setColorAt(0.5, Qt::yellow);
    gd1.setColorAt(0.6, Qt::yellow);
    gd1.setColorAt(0.7, Qt::green);
    gd1.setColorAt(1.0, Qt::green);
    gradient << gd1;
    theme->setBaseGradients(gradient);
    series->setColorStyle(QtDataVisualization::Q3DTheme::ColorStyleRangeGradient);
    // ��ϵ����ӵ�ͼ����
    graph->addSeries(series);

    //���ںŸ���
    comboBox = ui.comboBox;
    const auto serialPortInfos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo& serialPortInfo : serialPortInfos) {
        comboBox->addItem(serialPortInfo.portName());
    }
    connect(comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onComBoBoxSelectionSelection(int)));

    //��ʱ��������
    spinBox = ui.spinBox;
    spinBox->setRange(1, 1000000);
    spinBox->setValue(20);
    connect(spinBox, SIGNAL(valueChanged(int)), this, SLOT(setTimerThreshold(int)));

    //���ݻط�
    connect(ui.pushButton_10, &QPushButton::clicked, this, &GestureForRelease::openFileDialog);
    connect(this, SIGNAL(replay(QString)), radarSerialThread,  SLOT(replayTrigger(QString)));
    /*QThread* mainThread = QCoreApplication::instance()->thread();
    mainThread->setPriority(QThread::HighestPriority);
    radarSerialThread->setPriority(QThread::LowestPriority);*/


    //�طŹ�������
    ui.horizontalSlider_2->setMinimum(1);
    ui.horizontalSlider_2->setMaximum(99);
    ui.horizontalSlider_2->setValue(1);
    connect(ui.horizontalSlider_2, &QSlider::valueChanged, radarSerialThread, &RadarSerialThread::replaySlider);

    //Socketͨ��
    server = new QTcpServer(this);
    connect(server, &QTcpServer::newConnection, this, &GestureForRelease::onNewConnection);
    qDebug() << "1";
    if (!server->listen(QHostAddress("127.0.0.1"), 5010)) {
        qDebug() << "û����";
    }
    else {
        qDebug() << "������������";
    }
    connect(radarSerialThread, &RadarSerialThread::simulate,this,  &GestureForRelease::sendDataToClient);
    
    //��Ⱦ������
    connect(ui.lineEdit_2, &QLineEdit::textChanged, this, &GestureForRelease::setMinDepth);
    connect(ui.lineEdit_3, &QLineEdit::textChanged, this, &GestureForRelease::setMaxDepth);
    ui.lineEdit_2->setText("0");
    ui.lineEdit_3->setText("20");

    //�����Ϣ�Ƿ񱣴����
    connect(ui.checkBox, &QCheckBox::stateChanged, this, &GestureForRelease::setDepthBool);

    //���������ļ������·��
    connect(ui.pushButton_6, &QPushButton::clicked, this, &GestureForRelease::setFilePath);

    //X�������
    connect(ui.lineEdit_4, &QLineEdit::textChanged, this, &GestureForRelease::setMinX);
    connect(ui.lineEdit_5, &QLineEdit::textChanged, this, &GestureForRelease::setMaxX);
    ui.lineEdit_4->setText("-20");
    ui.lineEdit_5->setText("20");

    //Y�������
    connect(ui.lineEdit_6, &QLineEdit::textChanged, this, &GestureForRelease::setMinY);
    connect(ui.lineEdit_7, &QLineEdit::textChanged, this, &GestureForRelease::setMaxY);
    ui.lineEdit_6->setText("-20");
    ui.lineEdit_7->setText("20");
}



GestureForRelease::~GestureForRelease()
{
    this->stopAll();
    if (radarSerialThread->isRunning()) {
        radarSerialThread->quit();
        radarSerialThread->wait();
    }
    if (kinectThread->isRunning()) {
        kinectThread->quit();
        kinectThread->wait();
    }
    /*if (kinectColorThread->isRunning()) {
        kinectColorThread->quit();
        kinectColorThread->wait();
    }*/

}

void GestureForRelease::PushButtonClicked()
{
    //��ʾ��ת�ǶȰ�ť
    qDebug() << "PushButtonClicked!!!";
    QString pushButtnoShowText = "you has clicked the push button!!";
    auto widget = this->findChild<MyOpenGLWidget*>(QString::fromUtf8("openGLWidget"));
    if (widget) {
        qDebug() << "zhaodao le !!!!";
        QString name = widget->name;
        qDebug() << name;
        float rotationX = widget->rotationX;
        float rotationY = widget->rotationY;
        float zoomFactor = widget->zoomFactor;
        ui.lineEdit->setText(QString::number(rotationX, 'f', 2) + " " + QString::number(rotationY, 'f', 2) + " " + QString::number(zoomFactor, 'f', 2));
    }


    qDebug() << "what the hell is goiny in " << radarSerialThread->getThreshold();


}

//�����״�Ĳ�
void GestureForRelease::startKinect() {
    qDebug() << "start Kinect";
    kinectThread->start();
    kinectThread->trackerMode = ui.comboBox_2->currentText();

}

void GestureForRelease::stopKinect() {
    qDebug() << "stopKinect";
    kinectThread->stop();
    printf("Finished body tracking processing!\n");

    
}

//textBrowser��ʾ֡���ݵĲ�
void GestureForRelease::onNewDataAvailable(const QString& data) {
    ui.textEdit->setText(data);
}

void GestureForRelease::onErrorOccurred(const QString& error) {
    // ���ﴦ��������絯��һ��������Ϣ��
    qDebug() << "fashenglecuowu������";
}

//�����״�Ĳ�
void GestureForRelease::startRadar() {
    radarSerialThread->start();

}

void GestureForRelease::stopRadar() {
    radarSerialThread->stop();
}

//��ʾ���ڽǶ���Ϣ�Ĳ�
void GestureForRelease::updateDirection(const QString& x, const QString& y, const QString& z) {
    ui.lineEdit->setText(x + " " + y + " " + z);
}

//ȫ��������ƵĲ�
void GestureForRelease::startAll() {
    kinectThread->start();
    radarSerialThread->start();
    //radarSerialThread->setPriority(QThread::HighestPriority);
    //kinectColorThread->start();
}

void GestureForRelease::stopAll() {
    radarSerialThread->stop();
    //kinectColorThread->stop();
    kinectThread->stop();
}


//�״���ƻ��Ʋ�
void GestureForRelease::updataRadarPointCloud(const QQueue<QString> data) {

    allOpenGLWidget->updateRadarPoints(data, 1);
    radarOpenGLWidget->updateRadarPoints(data);

}
//Kinect�������Ʋ�
void GestureForRelease::updataBodyFrame(const QVector<QVector<QVector<GLfloat>>>& data, const QVector<uint32_t> &allBodyIndexs, QVector<QVector3D> depthPointClounds) {
    allOpenGLWidget->updateBodyFrame(data, allBodyIndexs);
    kinectOpenGLWidget->updateBodyFrame(data, allBodyIndexs, depthPointClounds);
}

//rgb��ʾ
void GestureForRelease::updateColor(const QPixmap& image) {
    qDebug() << ui.label->size().height() << ui.label->size().width() << image.size().width() << image.size().height();
    qint16 w = ui.openGLWidget->size().width() - 10;
    qint16 h = w * 1080 / 1920;
    QPixmap scaledPix = image.scaled(QSize(w, h), Qt::KeepAspectRatio);
    qDebug() << scaledPix.height() << scaledPix.width();
    ui.label->setPixmap(scaledPix);
}

//3Dscatter�״������ʾ��
void GestureForRelease::addRadarData(const QQueue<QString> pointCloudBuffer) {

    QtDataVisualization::QScatterDataArray* dataArray = new QtDataVisualization::QScatterDataArray();
    int count = 0;
    for (const QString& pointsFrame : pointCloudBuffer) {
        QStringList lines = pointsFrame.split("\n");
        int numPoints = lines.size();
        for (int i = 0; i < numPoints; ++i) {
            QStringList coords = lines[i].split(" ");
            if (coords.size() == 3) {
                //�״��xyz,��Ӧ����ռ��xzy�� ��openglwidget�Ŀռ�������ռ���ͬ���ʶ����״��xyz��z��y�Ե���Ͷ�䵽openglwidget��
                std::vector<std::vector<float>> matrix = { {-1.08930402f,-0.00126659f,-0.05982419f},
                    {0.04921241f,1.10069102f,-0.24182825f},
                    {0.06662979f,0.21614036f,0.95915324f} }; 
                std::vector<float> vector = { coords[0].toFloat(),coords[2].toFloat(),coords[1].toFloat() };//���״��z�ŵ�����ռ��y //���״��y�ŵ�����ռ��z
                std::vector<float> vector2 = { 0.00788764f,-0.11529322f,0.08330746f };
                std::vector<float>result(3, 0.0f);
                for (int i = 0; i < 3; i++)
                    for (int j = 0; j < 3; j++) {
                        result[i] += matrix[i][j] * vector[j];
                    }
                for (int i = 0; i < 3; ++i) {
                    result[i] = result[i] + vector2[i];
                }

                dataArray->append(QVector3D(-result[0], result[1], result[2]));
                dataArray->append(QVector3D(-result[0], -2, result[2]));
                /*points[i][0] = result[0];
                points[i][1] = result[1];
                points[i][2] = result[2];*/

                //��У׼
                /* points[i][0] = coords[0].toFloat();
                 points[i][1] = coords[2].toFloat();
                 points[i][2] = coords[1].toFloat();*/
            }

        }
    }
    
    series->dataProxy()->resetArray(dataArray);//����ɢ��

}





void GestureForRelease::onComBoBoxSelectionSelection(int index) {
    QString selectedPortName = comboBox->itemText(index);
    radarSerialThread->setPortName(selectedPortName);
}

void GestureForRelease::setTimerThreshold(int ms) {
    //qDebug() << ms;
    radarSerialThread->onTimerSetThreshold(ms);
    //radarSerialThread->setFrameThreshold(100000);//���ۼ�֡����÷ǳ����൱�ڹر��ۻ�֡������
}

void GestureForRelease::openFileDialog() {
    QString folderPath = QFileDialog::getOpenFileName(this, "Select .txt File", QDir::homePath(), "Text Files (*.txt)");
    if (!folderPath.isEmpty()) {
        qDebug() << "Selected .txt File path:" << folderPath;
    }

    emit replay(folderPath);
    qDebug() << "�ļ��򿪽���";
}


void GestureForRelease::sendDataToClient(){
    QString text = ui.lineEdit->text();
    qDebug() << text;
    QByteArray bytedata = QByteArray::fromHex(text.toLatin1()).toUpper();
    qDebug() << socket->state();
    if (socket->state() == QTcpSocket::ConnectedState&&socket){ socket->write(bytedata);
    qDebug() << "���ͳɹ�";
    }
    else { qDebug() << "δ���ӣ���������ʧ��"; }
}

//��ʾ���Կͻ��˵�����
void GestureForRelease::showClientData(){
    QByteArray data = socket->readAll();
    data = data.toHex();
    qDebug() << data;
    ui.textEdit->setText(data);
    ui.textEdit->show();
    kinectOpenGLWidget->updateBodyFrameQstring(data);
} 

void GestureForRelease::onNewConnection(){
    socket = server->nextPendingConnection();
    connect(socket, &QTcpSocket::readyRead, this, &GestureForRelease::showClientData);
    connect(socket, &QTcpSocket::disconnected, socket, &QTcpSocket::deleteLater);
    connect(ui.pushButton_11, &QPushButton::clicked, this, &GestureForRelease::sendDataToClient);
}

void GestureForRelease::setMinDepth(QString s) {
    float min = s.toFloat();
    qDebug() << "min depth:" << min;
    kinectOpenGLWidget->setMinDepth(min);
    kinectThread->setMinDepth(min);
}

void GestureForRelease::setMaxDepth(QString s) {
    float max = s.toFloat();
    qDebug() << "max depth:" << max;
    kinectOpenGLWidget->setMaxDepth(max);
    kinectThread->setMaxDepth(max);
}

void GestureForRelease::setDepthBool(int state){
    if (state == Qt::Checked) {
        kinectThread->saveDepth = true;
    }
    else if (state == Qt::Unchecked) {
        kinectThread->saveDepth = false;
    }
}


void GestureForRelease::setMinX(QString s) {
    float min = s.toFloat();
    qDebug() << "min X:" << min;
    kinectThread->setMinX(min);
}

void GestureForRelease::setMaxX(QString s) {
    float max = s.toFloat();
    qDebug() << "max X:" << max;
    kinectThread->setMaxX(max);
}

void GestureForRelease::setMinY(QString s) {
    float min = s.toFloat();
    qDebug() << "min Y:" << min;
    kinectThread->setMinY(min);
}

void GestureForRelease::setMaxY(QString s) {
    float max = s.toFloat();
    qDebug() << "max Y:" << max;
    kinectThread->setMaxY(max);
}

void GestureForRelease::setFilePath(){
    QString folderPath = QFileDialog::getExistingDirectory(this, "ѡ���ļ�");
    if (!folderPath.isEmpty()) {
        //QMessageBox::information(this, "ѡ���·��", "��ѡ���·����:" + folderPath);
        ui.textEdit->setText(folderPath);
        kinectThread->folderPath = folderPath;
        radarSerialThread->folderPath = folderPath;
    }
}