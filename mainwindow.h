#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <opencv.hpp>
#include <opencv/cv.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    QImage mat2qimage(cv::Mat& mat);
    ~MainWindow();

public Q_SLOTS:
    void updateView(int i);
    void toggleFilter(bool filter);

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
