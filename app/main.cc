#include "main_window.h"
#include <QApplication>

int main(int argc, char *argv[]) {
    LOG_ON(INFO);
    cl::Logger::GetInstance()->set_log_file("log.txt");

    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
