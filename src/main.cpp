#include <QApplication>
#include "gui/MainWindow.h"

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);

    rbf::MainWindow window;
    window.show();

    return app.exec();
}
