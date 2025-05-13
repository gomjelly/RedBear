#pragma once

#include <QtWidgets/QMainWindow>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QTimer>
#include <QImage>


class CameraWorker : public QObject
{
    Q_OBJECT
public:
    CameraWorker() {};
    ~CameraWorker() {};

public slots:
    void process();

signals:
    void finished();
    void frameReady(const QImage& img);

};

class RedBear : public QMainWindow
{
    Q_OBJECT

public:
    RedBear(QWidget *parent = nullptr);
    ~RedBear();

    void startRender();

private:
    QGraphicsScene* m_scene = nullptr;
    QGraphicsView* m_view = nullptr;
    QGraphicsPixmapItem* m_user = nullptr;
};
