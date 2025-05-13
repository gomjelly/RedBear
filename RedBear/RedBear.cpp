#include "RedBear.h"

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <QThread>
#include <QImage>
#include <QPainter>
#include <QMediaDevices>
#include <QCameraDevice>
#include <QCamera>
#include <QVideoWidget>

RedBear::RedBear(QWidget* parent)
    : QMainWindow(parent)
{
    // Qt 위젯 생성
    //m_scene = new QGraphicsScene();
    //m_view = new QGraphicsView(m_scene);
    //m_view->setWindowTitle("Camera View");
    //m_view->show();
    
    QList<QCameraDevice> cameras = QMediaDevices::videoInputs();

    // QComboBox에 카메라 이름 표시
    QComboBox* comboBox = new QComboBox;
    comboBox->clear();
    for (const QCameraDevice& camera : cameras) {
        comboBox->addItem(camera.description(), camera.id());
    }

    QGridLayout* layout = new QGridLayout;
    layout->addWidget(comboBox, 0, 0);

    QWidget* centralWidget = new QWidget;
    centralWidget->setLayout(layout);

    this->setCentralWidget(centralWidget);
    //this->startRender();

    this->setFixedSize(800, 600);
}

RedBear::~RedBear()
{
}

void RedBear::startRender()
{
    // 카메라를 별도의 스레드에서 열기 위한 준비
    QThread* thread = new QThread;
    CameraWorker* worker = new CameraWorker();
    worker->moveToThread(thread);

    connect(thread, &QThread::started, worker, &CameraWorker::process);
    connect(worker, &CameraWorker::finished, thread, &QThread::quit);
    connect(worker, &CameraWorker::finished, worker, &CameraWorker::deleteLater);
    connect(thread, &QThread::finished, thread, &QThread::deleteLater);

    // 카메라 프레임을 받아 업데이트하는 시그널
    connect(worker, &CameraWorker::frameReady, this, [=](const QImage& img) {
        QPixmap pixmap = QPixmap::fromImage(img);
        //m_scene->clear();
        if (m_user)
            m_scene->removeItem(m_user);
        m_user = m_scene->addPixmap(pixmap);

        });

    thread->start();
}

void drawPose(cv::Mat& frame, const cv::Mat& points) {
    int nPairs = 20; // COCO dataset에서의 관절 쌍 개수
    int posePairs[20][2] = {
        {1, 2}, {1, 5}, {2, 3}, {3, 4}, {5, 6}, {6, 7},  // 팔
        {1, 8}, {8, 9}, {9, 10}, {1, 11}, {11, 12}, {12, 13}, // 몸통
        {1, 0}, {0, 14}, {14, 16}, {0, 15}, {15, 17}  // 얼굴과 다리
    };

    for (int i = 0; i < nPairs; i++) {
        // 관절 쌍의 좌표를 가져옵니다.
        cv::Point partA = cv::Point((int)points.at<float>(0, posePairs[i][0]), (int)points.at<float>(1, posePairs[i][0]));
        cv::Point partB = cv::Point((int)points.at<float>(0, posePairs[i][1]), (int)points.at<float>(1, posePairs[i][1]));

        if (partA.x <= 0 || partA.y <= 0 || partB.x <= 0 || partB.y <= 0)
            continue;

        line(frame, partA, partB, cv::Scalar(0, 255, 255), 8);
        circle(frame, partA, 8, cv::Scalar(0, 0, 255), -1);
        circle(frame, partB, 8, cv::Scalar(0, 0, 255), -1);
    }
}

// CameraWorker 클래스 내부
void CameraWorker::process() {
    cv::VideoCapture cap(0);

    if (!cap.isOpened()) {
        emit finished();
        return;
    }

    // 모델 파일 및 구성 파일 경로
    std::string modelFile = "C:\\dev\\RedBear\\pose_iter_440000.caffemodel";
    std::string configFile = "C:\\dev\\RedBear\\pose_deploy_linevec.prototxt";

    cv::Mat frame;
    cv::Ptr<cv::dnn::Net> net = cv::makePtr<cv::dnn::Net>(cv::dnn::readNetFromCaffe(configFile, modelFile));

    while (true) {
        cv::Mat frame;
        cap.read(frame);

        if (frame.empty()) break;

        // 이미지 크기를 반으로 줄임
        cv::resize(frame, frame, cv::Size(frame.cols / 2, frame.rows / 2));

        cv::flip(frame, frame, 1); // frame을 좌우로 반전

        //
        cv::Mat inputBlob = cv::dnn::blobFromImage(frame, 1.0 / 255, cv::Size(128, 128), cv::Scalar(0, 0, 0), false, false);
        net->setInput(inputBlob);
        cv::Mat output = net->forward();

        // 출력 매트릭스의 차원을 얻습니다.
        int H = output.size[2];
        int W = output.size[3];

        // 각 관절의 위치를 저장할 매트릭스를 생성합니다.
        cv::Mat points(2, 18, CV_32F, cv::Scalar(0));

        for (int n = 0; n < 18; n++) {
            // 해당 신체 부위에 대한 히트맵에서 최대값을 찾습니다.
            cv::Mat probMap(H, W, CV_32F, output.ptr(0, n));
            cv::Point maxLoc;
            double prob;
            cv::minMaxLoc(probMap, 0, &prob, 0, &maxLoc);

            if (prob > 0.1) {
                points.at<float>(0, n) = (float)(maxLoc.x) * frame.cols / W;
                points.at<float>(1, n) = (float)(maxLoc.y) * frame.rows / H;
            }
        }

        drawPose(frame, points);

        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB); // BGR에서 RGB로 변환
        QImage img(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
        img.bits(); // QImage가 데이터를 복사하도록 강제

        emit frameReady(img);
        QThread::msleep(33); // 약 30프레임 속도
    }

    emit finished();
}


#if 0 // color version
void CameraWorker::process() {
    cv::VideoCapture cap(0);

    if (!cap.isOpened()) {
        emit finished();
        return;
    }

    // 모델 파일 및 구성 파일 경로
    std::string modelFile = "D:\\MyProject\\pose_iter_440000.caffemodel";
    std::string configFile = "D:\\MyProject\\pose_deploy_linevec.prototxt";

    cv::Mat frame;
    cv::Ptr<cv::dnn::Net> net = cv::makePtr<cv::dnn::Net>(cv::dnn::readNetFromCaffe(configFile, modelFile));

    while (true) {
        cv::Mat frame;
        cap.read(frame);

        if (frame.empty()) break;
        cv::flip(frame, frame, 1); // frame을 좌우로 반전

        //
        cv::Mat inputBlob = cv::dnn::blobFromImage(frame, 1.0 / 255, cv::Size(368, 368), cv::Scalar(0, 0, 0), false, false);
        net->setInput(inputBlob);
        cv::Mat output = net->forward();

        // 출력 매트릭스의 차원을 얻습니다.
        int H = output.size[2];
        int W = output.size[3];

        // 각 관절의 위치를 저장할 매트릭스를 생성합니다.
        cv::Mat points(2, 18, CV_32F, cv::Scalar(0));

        for (int n = 0; n < 18; n++) {
            // 해당 신체 부위에 대한 히트맵에서 최대값을 찾습니다.
            cv::Mat probMap(H, W, CV_32F, output.ptr(0, n));
            cv::Point maxLoc;
            double prob;
            minMaxLoc(probMap, 0, &prob, 0, &maxLoc);

            if (prob > 0.1) {
                points.at<float>(0, n) = (float)(maxLoc.x) * frame.cols / W;
                points.at<float>(1, n) = (float)(maxLoc.y) * frame.rows / H;
            }
        }

        drawPose(frame, points);

        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB); // BGR에서 RGB로 변환
        QImage img(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
        img.bits(); // QImage가 데이터를 복사하도록 강제

        emit frameReady(img);
        QThread::msleep(33); // 약 30프레임 속도
    }

    emit finished();
}
#endif