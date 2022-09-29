#include "library.h"

#include <iostream>
#include <QApplication>
#include <QDebug>

void hello() {
    std::cout << "Hello, World!" << std::endl;
    qDebug() << "Qt version :" << QT_VERSION_STR;
}
