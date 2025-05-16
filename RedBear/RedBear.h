#pragma once



#include <QtWidgets>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QTimer>
#include <QImage>
#include <QMutex>
#include <QMutexLocker>
#include <QVideoSink>
#include <QVideoFrame>
#include <QMediaCaptureSession>
#include <QCamera>



class CameraWorker : public QObject
{
    Q_OBJECT
public:
    CameraWorker() : running(true) {}
    ~CameraWorker() {};



    void stop() {
        QMutexLocker locker(&mutex); // 뮤텍스 잠금
        running = false;             // 실행 중단 요청
    }
private:
    bool running;       // 실행 상태 플래그
    QMutex mutex;       // 플래그 보호를 위한 뮤텍스



public slots:
    void process();



signals:
    void finished();
    void frame1Ready(const QImage& img);
    void frame2Ready(const QImage& img);
};



class AspectRatioLabel : public QLabel
{
    Q_OBJECT
public:
    explicit AspectRatioLabel(QWidget* parent = nullptr)
        : QLabel(parent)
    {
        setMinimumSize(320, 240);
        setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        setAlignment(Qt::AlignCenter);
        setStyleSheet("background-color: #222; color: #fff; border: 1px solid #aaa;");



        setAttribute(Qt::WA_OpaquePaintEvent);
    }



    void setPixmap(const QPixmap& pix) {
        QPixmap scaled = pix.scaled(this->size(), Qt::KeepAspectRatio);
        QLabel::setPixmap(scaled);
    }



protected:
    // 가로세로 비율(4:3) 계산
    int heightForWidth(int w) const override {
        return static_cast<int>(w * 3.0 / 4.0);
    }



    // 크기 힌트 설정
    QSize sizeHint() const override {
        return QSize(320, 240);
    }



    // 리사이즈 이벤트 처리
    void resizeEvent(QResizeEvent* event) override {
        QLabel::resizeEvent(event);
        if (event->oldSize().width() > 0) {
            int new_height = heightForWidth(this->width());
            this->setFixedHeight(new_height);
        }
    }
};



class RedBear : public QMainWindow
{
    Q_OBJECT



public:
    RedBear(QWidget *parent = nullptr);
    ~RedBear();



private slots:
    void startRender(); // 영상 출력 시작
    void updateOriginalVideo(const QImage& img); // originalVideoLabel 업데이트
    void updateARVideo(const QImage& img); // arVideoLabel 업데이트



protected:
    void closeEvent(QCloseEvent* event) override; // 창 닫기 이벤트 처리
private:
    AspectRatioLabel* originalVideoLabel; // 원본 영상 출력 라벨
    AspectRatioLabel* arVideoLabel;
    QThread* cameraThread;               // 카메라 스레드
    CameraWorker* cameraWorker;          // 카메라 작업자



    //QGraphicsScene* m_scene = nullptr;
    //QGraphicsView* m_view = nullptr;
    //QGraphicsPixmapItem* m_user = nullptr;
};
 

