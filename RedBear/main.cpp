#include "RedBear.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    RedBear w;
    w.show();
    return a.exec();
}
