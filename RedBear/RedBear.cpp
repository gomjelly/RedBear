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
    // Source 그룹박스
    QGroupBox* sourceGroup = new QGroupBox(tr("Source"), this);
    QVBoxLayout* sourceMainLayout = new QVBoxLayout;



    // 카메라 선택 영역
    QHBoxLayout* cameraLayout = new QHBoxLayout;
    QRadioButton* camRadio = new QRadioButton(tr("카메라"), this);
    camRadio->setFixedWidth(70);
    QComboBox* cameraComboBox = new QComboBox;
    cameraComboBox->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    QList<QCameraDevice> cameras = QMediaDevices::videoInputs();
    for (const QCameraDevice& camera : cameras) {
        cameraComboBox->addItem(camera.description(), camera.id());
    }
    cameraLayout->addWidget(camRadio);
    cameraLayout->addWidget(cameraComboBox);



    // 파일 선택 영역
    QHBoxLayout* fileLayout = new QHBoxLayout;
    QRadioButton* fileRadio = new QRadioButton(tr("파일"), this);
    fileRadio->setFixedWidth(70);
    QLineEdit* filePath = new QLineEdit();
    filePath->setReadOnly(true);  // 직접 입력 방지
    QPushButton* fileButton = new QPushButton(tr("..."), this);
    connect(fileButton, &QPushButton::clicked, this, [this]() {
        QString filename = QFileDialog::getOpenFileName(this, tr("영상 파일 선택"), "",
            tr("Video Files (*.mp4 *.avi)"));
        if (!filename.isEmpty()) {
            // 파일 선택 시 처리
        }
        });



    fileLayout->addWidget(fileRadio);
    fileLayout->addWidget(filePath);
    fileLayout->addWidget(fileButton);



    // 라디오 버튼 그룹화
    QButtonGroup* sourceGroupBtn = new QButtonGroup(this);
    sourceGroupBtn->addButton(camRadio);
    sourceGroupBtn->addButton(fileRadio);
    camRadio->setChecked(true);  // 기본 선택



    sourceMainLayout->addLayout(cameraLayout);
    sourceMainLayout->addLayout(fileLayout);
    sourceGroup->setLayout(sourceMainLayout);



    QHBoxLayout* topLayout = new QHBoxLayout;



    QPushButton* startButton = new QPushButton(tr("시작"));
    QPushButton* stopButton = new QPushButton(tr("정지"));
    QPushButton* captureButton = new QPushButton(tr("녹화"));



    topLayout->addWidget(sourceGroup);
    topLayout->addWidget(startButton);
    topLayout->addWidget(stopButton);
    topLayout->addWidget(captureButton);



    // 3. 영상 출력 QLabel (좌: 원본, 우: AR 적용)
    originalVideoLabel = new AspectRatioLabel();
    arVideoLabel = new AspectRatioLabel();



    sourceGroup->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);



    // 전체 레이아웃
    QGridLayout* mainLayout = new QGridLayout;
    mainLayout->addLayout(topLayout, 0, 0, 1, 2);
    mainLayout->addWidget(originalVideoLabel, 1, 0);
    mainLayout->addWidget(arVideoLabel, 1, 1);



    QWidget* mainWidget = new QWidget;
    mainWidget->setLayout(mainLayout);



    this->setCentralWidget(mainWidget);



    setWindowTitle(tr("hello world"));



    // startButton 클릭 이벤트 연결
    connect(startButton, &QPushButton::clicked, this, &RedBear::startRender);
}



RedBear::~RedBear()
{
    if (cameraThread) {
        cameraWorker->stop(); // 작업자에게 종료 요청 (필요 시 구현)
        cameraThread->quit(); // 스레드 종료 요청
        cameraThread->wait(); // 스레드가 종료될 때까지 대기
    }
}



// startRender 메서드 수정
void RedBear::startRender()
{
    if (cameraThread) return; // 이미 실행 중이면 무시



    // 카메라 스레드 및 작업자 초기화
    cameraThread = new QThread;
    cameraWorker = new CameraWorker();
    cameraWorker->moveToThread(cameraThread);



    connect(cameraThread, &QThread::started, cameraWorker, &CameraWorker::process);
    connect(cameraWorker, &CameraWorker::frame1Ready, this, &RedBear::updateOriginalVideo);
    connect(cameraWorker, &CameraWorker::frame2Ready, this, &RedBear::updateARVideo);
    connect(cameraWorker, &CameraWorker::finished, cameraThread, &QThread::quit);
    connect(cameraWorker, &CameraWorker::finished, cameraWorker, &CameraWorker::deleteLater);
    connect(cameraThread, &QThread::finished, cameraThread, &QThread::deleteLater);



    cameraThread->start();
}



// originalVideoLabel 업데이트
void RedBear::updateOriginalVideo(const QImage& img)
{
    QPixmap pixmap = QPixmap::fromImage(img);
    originalVideoLabel->setPixmap(pixmap);
}



void RedBear::updateARVideo(const QImage& img)
{
    QPixmap pixmap = QPixmap::fromImage(img);
    arVideoLabel->setPixmap(pixmap);
}



// closeEvent 구현
void RedBear::closeEvent(QCloseEvent* event)
{
    if (cameraThread) {
        cameraWorker->stop(); // 작업자에게 종료 요청 (필요 시 구현)
        cameraThread->quit(); // 스레드 종료 요청
        cameraThread->wait(); // 스레드가 종료될 때까지 대기
    }
    event->accept(); // 창 닫기 허용
}



void drawPose(cv::Mat& frame, const std::vector<cv::Point>& points) {
    int posePairs[17][2] = {
        {1, 2}, {1, 5}, {2, 3}, {3, 4}, {5, 6}, {6, 7},  // 팔
        {1, 8}, {8, 9}, {9, 10}, {1, 11}, {11, 12}, {12, 13}, // 몸통
        {8, 11}, // 골반 연결
        {0, 14}, {14, 16}, {0, 15}, {15, 17}  // 얼굴과 다리
    };



    for (int i = 0; i < 17; i++) {
        int partAIdx = posePairs[i][0];
        int partBIdx = posePairs[i][1];



        if (partAIdx >= points.size() || partBIdx >= points.size()) continue;



        cv::Point partA = points[partAIdx];
        cv::Point partB = points[partBIdx];



        if (partA.x <= 0 || partA.y <= 0 || partB.x <= 0 || partB.y <= 0) continue;



        // 선 그리기
        cv::line(frame, partA, partB, cv::Scalar(255, 0, 0), 3);



        // 관절 점 그리기
        cv::circle(frame, partA, 2, cv::Scalar(0, 0, 255), -1);
    cv::circle(frame, partB, 2, cv::Scalar(0, 0, 255), -1);
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
    std::string modelFile = "D:\\01_Workspace\\RedBear\\pose_iter_440000.caffemodel";
    std::string configFile = "D:\\01_Workspace\\RedBear\\pose_deploy_linevec.prototxt";
 
    cv::Mat frame;
    cv::Ptr<cv::dnn::Net> net = cv::makePtr<cv::dnn::Net>(cv::dnn::readNetFromCaffe(configFile, modelFile));
 
    while (true) {
        {
            QMutexLocker locker(&mutex); // 뮤텍스 잠금
            if (!running) break;         // 실행 중단 플래그 확인
        }
        cv::Mat frame;
        cap.read(frame);
 
        if (frame.empty()) break;
 
        // 이미지 크기를 반으로 줄임
        cv::resize(frame, frame, cv::Size(frame.cols / 4, frame.rows / 4));
        cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB); // BGR에서 RGB로 변환
        cv::flip(frame, frame, 1); // frame을 좌우로 반전
 
        //
        cv::Mat inputBlob = cv::dnn::blobFromImage(frame, 1.0 / 255, cv::Size(128, 128), cv::Scalar(0, 0, 0), false, false);
 
        QImage img1(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
        img1.bits();
        emit frame1Ready(img1);
 
        net->setInput(inputBlob);
        cv::Mat output = net->forward();
 
        // 출력 매트릭스의 차원을 얻습니다.
        int H = output.size[2];
        int W = output.size[3];
 
        if (H <= 0 || W <= 0) {
            qDebug() << "출력 매트릭스 크기가 유효하지 않습니다.";
            break;
        }
 
        // 여러 사람의 관절 위치를 저장할 벡터
        std::vector<std::vector<cv::Point>> allPeoplePoints;
 
        for (int n = 0; n < 18; n++) {
            cv::Mat probMap(H, W, CV_32F, output.ptr(0, n));
 
            if (probMap.empty()) continue;
 
            cv::Mat resizedProbMap;
            cv::resize(probMap, resizedProbMap, cv::Size(frame.cols, frame.rows));
 
            cv::Mat binaryMap;
            cv::threshold(resizedProbMap, binaryMap, 0.1, 1.0, cv::THRESH_BINARY);
 
            if (binaryMap.type() != CV_8U) {
                binaryMap.convertTo(binaryMap, CV_8U);
            }
 
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(binaryMap, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
 
            for (const auto& contour : contours) {
                if (contour.empty()) continue;
 
                cv::Moments mu = cv::moments(contour);
                if (mu.m00 > 0) {
                    int x = static_cast<int>(mu.m10 / mu.m00);
                    int y = static_cast<int>(mu.m01 / mu.m00);
 
                    if (x >= 0 && x < frame.cols && y >= 0 && y < frame.rows) {
                        cv::Point point(x, y);
 
                        // 가장 가까운 사람 찾기
                        int closestPersonIdx = -1;
                        double minDistance = 10.0; // 거리 임계값 (조정 가능)
                        for (int i = 0; i < allPeoplePoints.size(); i++) {
                            if (allPeoplePoints[i][n].x > 0 && allPeoplePoints[i][n].y > 0) {
                                double distance = cv::norm(allPeoplePoints[i][n] - point);
                                if (distance < minDistance) {
                                    closestPersonIdx = i;
                                    minDistance = distance;
                                }
                            }
                        }
 
                        // 기존 사람에 추가하거나 새로운 사람 생성
                        if (closestPersonIdx != -1) {
                            allPeoplePoints[closestPersonIdx][n] = point;
                        }
                        else {
                            std::vector<cv::Point> newPerson(18, cv::Point(-1, -1));
                            newPerson[n] = point;
                            allPeoplePoints.push_back(newPerson);
                        }
                    }
                }
            }
        }
 
        for (const auto& personPoints : allPeoplePoints) {
            drawPose(frame, personPoints);
        }
 
        //std::vector<cv::Point> personPoints(18, cv::Point(-1, -1)); // 한 사람의 관절 좌표 저장
 
        //for (int n = 0; n < 18; n++) {
        //    cv::Mat probMap(H, W, CV_32F, output.ptr(0, n));
 
        //    if (probMap.empty()) continue;
 
        //    cv::Mat resizedProbMap;
        //    cv::resize(probMap, resizedProbMap, cv::Size(frame.cols, frame.rows));
 
        //    cv::Mat binaryMap;
        //    cv::threshold(resizedProbMap, binaryMap, 0.1, 1.0, cv::THRESH_BINARY);
 
        //    if (binaryMap.type() != CV_8U) {
        //        binaryMap.convertTo(binaryMap, CV_8U);
        //    }
 
        //    std::vector<std::vector<cv::Point>> contours;
        //    cv::findContours(binaryMap, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
 
        //    for (const auto& contour : contours) {
        //        if (contour.empty()) continue;
 
        //        cv::Moments mu = cv::moments(contour);
        //        if (mu.m00 > 0) {
        //            int x = static_cast<int>(mu.m10 / mu.m00);
        //            int y = static_cast<int>(mu.m01 / mu.m00);
 
        //            if (x >= 0 && x < frame.cols && y >= 0 && y < frame.rows) {
        //                personPoints[n] = cv::Point(x, y); // 관절 좌표 저장
        //                break; // 한 관절에 대해 첫 번째 중심점만 사용
        //            }
        //        }
        //    }
        //}
 
        //drawPose(frame, personPoints);
        QImage img2(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
        img2.bits(); // QImage가 데이터를 복사하도록 강제
 
        emit frame2Ready(img2);
        QThread::msleep(100);
    }
 
    emit finished();
}


// #include "RedBear.h"

// #include <opencv2/opencv.hpp>
// #include <opencv2/dnn.hpp>
// #include <QThread>
// #include <QImage>
// #include <QPainter>
// #include <QMediaDevices>
// #include <QCameraDevice>
// #include <QCamera>
// #include <QVideoWidget>

// RedBear::RedBear(QWidget* parent)
//     : QMainWindow(parent)
// {
//     // Qt 위젯 생성
//     //m_scene = new QGraphicsScene();
//     //m_view = new QGraphicsView(m_scene);
//     //m_view->setWindowTitle("Camera View");
//     //m_view->show();
    
//     QList<QCameraDevice> cameras = QMediaDevices::videoInputs();

//     // QComboBox에 카메라 이름 표시
//     QComboBox* comboBox = new QComboBox;
//     comboBox->clear();
//     for (const QCameraDevice& camera : cameras) {
//         comboBox->addItem(camera.description(), camera.id());
//     }

//     QGridLayout* layout = new QGridLayout;
//     layout->addWidget(comboBox, 0, 0);

//     QWidget* centralWidget = new QWidget;
//     centralWidget->setLayout(layout);

//     this->setCentralWidget(centralWidget);
//     //this->startRender();

//     this->setFixedSize(800, 600);
// }

// RedBear::~RedBear()
// {
// }

// void RedBear::startRender()
// {
//     // 카메라를 별도의 스레드에서 열기 위한 준비
//     QThread* thread = new QThread;
//     CameraWorker* worker = new CameraWorker();
//     worker->moveToThread(thread);

//     connect(thread, &QThread::started, worker, &CameraWorker::process);
//     connect(worker, &CameraWorker::finished, thread, &QThread::quit);
//     connect(worker, &CameraWorker::finished, worker, &CameraWorker::deleteLater);
//     connect(thread, &QThread::finished, thread, &QThread::deleteLater);

//     // 카메라 프레임을 받아 업데이트하는 시그널
//     connect(worker, &CameraWorker::frameReady, this, [=](const QImage& img) {
//         QPixmap pixmap = QPixmap::fromImage(img);
//         //m_scene->clear();
//         if (m_user)
//             m_scene->removeItem(m_user);
//         m_user = m_scene->addPixmap(pixmap);

//         });

//     thread->start();
// }

// void drawPose(cv::Mat& frame, const cv::Mat& points) {
//     int nPairs = 20; // COCO dataset에서의 관절 쌍 개수
//     int posePairs[20][2] = {
//         {1, 2}, {1, 5}, {2, 3}, {3, 4}, {5, 6}, {6, 7},  // 팔
//         {1, 8}, {8, 9}, {9, 10}, {1, 11}, {11, 12}, {12, 13}, // 몸통
//         {1, 0}, {0, 14}, {14, 16}, {0, 15}, {15, 17}  // 얼굴과 다리
//     };

//     for (int i = 0; i < nPairs; i++) {
//         // 관절 쌍의 좌표를 가져옵니다.
//         cv::Point partA = cv::Point((int)points.at<float>(0, posePairs[i][0]), (int)points.at<float>(1, posePairs[i][0]));
//         cv::Point partB = cv::Point((int)points.at<float>(0, posePairs[i][1]), (int)points.at<float>(1, posePairs[i][1]));

//         if (partA.x <= 0 || partA.y <= 0 || partB.x <= 0 || partB.y <= 0)
//             continue;

//         line(frame, partA, partB, cv::Scalar(0, 255, 255), 8);
//         circle(frame, partA, 8, cv::Scalar(0, 0, 255), -1);
//         circle(frame, partB, 8, cv::Scalar(0, 0, 255), -1);
//     }
// }

// // CameraWorker 클래스 내부
// void CameraWorker::process() {
//     cv::VideoCapture cap(0);

//     if (!cap.isOpened()) {
//         emit finished();
//         return;
//     }

//     // 모델 파일 및 구성 파일 경로
//     std::string modelFile = "C:\\dev\\RedBear\\pose_iter_440000.caffemodel";
//     std::string configFile = "C:\\dev\\RedBear\\pose_deploy_linevec.prototxt";

//     cv::Mat frame;
//     cv::Ptr<cv::dnn::Net> net = cv::makePtr<cv::dnn::Net>(cv::dnn::readNetFromCaffe(configFile, modelFile));

//     while (true) {
//         cv::Mat frame;
//         cap.read(frame);

//         if (frame.empty()) break;

//         // 이미지 크기를 반으로 줄임
//         cv::resize(frame, frame, cv::Size(frame.cols / 2, frame.rows / 2));

//         cv::flip(frame, frame, 1); // frame을 좌우로 반전

//         //
//         cv::Mat inputBlob = cv::dnn::blobFromImage(frame, 1.0 / 255, cv::Size(128, 128), cv::Scalar(0, 0, 0), false, false);
//         net->setInput(inputBlob);
//         cv::Mat output = net->forward();

//         // 출력 매트릭스의 차원을 얻습니다.
//         int H = output.size[2];
//         int W = output.size[3];

//         // 각 관절의 위치를 저장할 매트릭스를 생성합니다.
//         cv::Mat points(2, 18, CV_32F, cv::Scalar(0));

//         for (int n = 0; n < 18; n++) {
//             // 해당 신체 부위에 대한 히트맵에서 최대값을 찾습니다.
//             cv::Mat probMap(H, W, CV_32F, output.ptr(0, n));
//             cv::Point maxLoc;
//             double prob;
//             cv::minMaxLoc(probMap, 0, &prob, 0, &maxLoc);

//             if (prob > 0.1) {
//                 points.at<float>(0, n) = (float)(maxLoc.x) * frame.cols / W;
//                 points.at<float>(1, n) = (float)(maxLoc.y) * frame.rows / H;
//             }
//         }

//         drawPose(frame, points);

//         cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB); // BGR에서 RGB로 변환
//         QImage img(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
//         img.bits(); // QImage가 데이터를 복사하도록 강제

//         emit frameReady(img);
//         QThread::msleep(33); // 약 30프레임 속도
//     }

//     emit finished();
// }

