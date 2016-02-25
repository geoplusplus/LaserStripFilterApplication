#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QObject::connect(ui->blueSliderMin, SIGNAL(valueChanged(int)), this, SLOT(updateView(int)));
    QObject::connect(ui->blueSliderMax, SIGNAL(valueChanged(int)), this, SLOT(updateView(int)));
    QObject::connect(ui->greenSliderMin, SIGNAL(valueChanged(int)), this, SLOT(updateView(int)));
    QObject::connect(ui->greenSliderMax, SIGNAL(valueChanged(int)), this, SLOT(updateView(int)));
    QObject::connect(ui->redSliderMin, SIGNAL(valueChanged(int)), this, SLOT(updateView(int)));
    QObject::connect(ui->redSliderMax, SIGNAL(valueChanged(int)), this, SLOT(updateView(int)));
    QObject::connect(ui->checkBox, SIGNAL(toggled(bool)), this, SLOT(toggleFilter(bool)));
}

QImage MainWindow::mat2qimage(cv::Mat& mat) {
    switch ( mat.type() ) {
    // 8-bit, 4 channel
    case CV_8UC4: {
        QImage image( mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB32 );
        return image;
    }

        // 8-bit, 3 channel
    case CV_8UC3: {
        QImage image( mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888 );
        return image.rgbSwapped();
    }

        // 8-bit, 1 channel
    case CV_8UC1: {
        static QVector<QRgb>  sColorTable;

        // only create our color table once
        if ( sColorTable.isEmpty() ) {
            for ( int i = 0; i < 256; ++i )
                sColorTable.push_back( qRgb( i, i, i ) );
        }

        QImage image( mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Indexed8 );

        image.setColorTable( sColorTable );

        return image;
    }

    default:
        std::cout << "ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" << mat.type();
        break;
    }

    return QImage();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::toggleFilter(bool filter)
{
    updateView(0);
}

void MainWindow::updateView(int i)
{
    cv::Mat before = cv::imread("/home/minions/pictureBefore_1.png");
    cv::Mat after = cv::imread("/home/minions/pictureAfter_1.png");
    cv::Mat diff;
    cv::Mat redOnly;
    QImage qImg;

    cv::absdiff(before, after, diff);

    if (ui->checkBox->isChecked()) {
        cv::inRange(diff, cv::Scalar(ui->blueSliderMin->value(), ui->greenSliderMin->value(), ui->redSliderMin->value()),
                         cv::Scalar(ui->blueSliderMax->value(), ui->greenSliderMax->value(), ui->redSliderMax->value()), redOnly);
        qImg = mat2qimage(redOnly);

    } else {
        qImg = mat2qimage(diff);
    }

    QPixmap pixMap = QPixmap::fromImage(qImg);
    ui->imageLabel->setPixmap(pixMap);
}
