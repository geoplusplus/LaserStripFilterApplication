#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <math.h>
#include <list>
#include <iostream>
#include <fstream>

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
    QObject::connect(ui->medianSlider, SIGNAL(valueChanged(int)), this, SLOT(updateView(int)));
    QObject::connect(ui->erosionSlider, SIGNAL(valueChanged(int)), this, SLOT(updateView(int)));
    QObject::connect(ui->checkBox, SIGNAL(toggled(bool)), this, SLOT(toggleFilter(bool)));
    QObject::connect(ui->diffCheckBox, SIGNAL(toggled(bool)), this, SLOT(toggleFilter(bool)));
    QObject::connect(ui->diffReferenceCheckBox, SIGNAL(toggled(bool)), this, SLOT(toggleFilter(bool)));
    QObject::connect(ui->medianCheckBox, SIGNAL(toggled(bool)), this, SLOT(toggleFilter(bool)));
    QObject::connect(ui->erosionCheckBox, SIGNAL(toggled(bool)), this, SLOT(toggleFilter(bool)));
    QObject::connect(ui->mainCheckBox, SIGNAL(toggled(bool)), this, SLOT(toggleFilter(bool)));

    ui->greenSliderMax->setValue(130);
    ui->blueSliderMax->setValue(130);
    ui->redSliderMax->setValue(255);
    ui->redSliderMin->setValue(30);

    ui->medianSlider->setValue(5);
    ui->erosionSlider->setValue(1);

    ui->diffCheckBox->toggle();
    ui->checkBox->toggle();
    ui->diffReferenceCheckBox->toggle();
    ui->erosionCheckBox->toggle();
    ui->medianCheckBox->toggle();

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
    QString path = QString("/home/minions/pikk.ply");


    std::ofstream myfile;
    myfile.open (path.toUtf8(), std::ofstream::out);
    myfile << "ply" << std::endl;
    myfile << "format ascii 1.0" << std::endl;
    myfile << "comment Made with spinscan!" << std::endl;
    myfile << "element vertex " << std::endl;
    myfile << "property double x" << std::endl;
    myfile << "property double y" << std::endl;
    myfile << "property double z" << std::endl;
    myfile << "property float nx" << std::endl;
    myfile << "property float ny" << std::endl;
    myfile << "property float nz" << std::endl;
//    myfile << "property uchar red" << std::endl;
//    myfile << "property uchar green" << std::endl;
//    myfile << "property uchar blue" << std::endl;
    myfile << "end_header" << std::endl;
    myfile.close();
    std::cout << "Wrote to ply file" << std::endl;

    processImage(this->filtered, path);
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

    cv::Mat referenceBefore = cv::imread("/home/minions/pictureReferenceBefore_1.png");
    cv::Mat referenceAfter = cv::imread("/home/minions/pictureReferenceAfter_1.png");

    cv::Mat final = after;
    cv::Mat temp;

    if (ui->diffCheckBox->isChecked() && ui->mainCheckBox->isChecked()) {
        cv::absdiff(before, after, temp);
        final = temp;
    }

    if (ui->diffReferenceCheckBox->isChecked() && ui->mainCheckBox->isChecked()) {
        cv::Mat referenceDiff;
        cv::absdiff(referenceBefore, referenceAfter, referenceDiff);
        cv::absdiff(final, referenceDiff, temp);
        final = temp;
    }

    cv::Mat temp2;

    if (ui->sharpeningCheckBox->isChecked() && ui->mainCheckBox->isChecked()) {
        cv::GaussianBlur(final, temp2, cv::Size(0, 0), 5);
        cv::addWeighted(final, 3, temp2, -1, 0, temp);
        final = temp;
    }

    if (ui->medianCheckBox->isChecked() && ui->mainCheckBox->isChecked()) {
        // Post-processing to remove noise
        int kernelSize = ui->medianSlider->value();

        if (kernelSize % 2 != 1)
            kernelSize = kernelSize - 1;

        if (kernelSize < 1)
            kernelSize = 1;

        cv::medianBlur(final, temp, kernelSize);
        final = temp;
    }

    if (ui->erosionCheckBox->isChecked() && ui->mainCheckBox->isChecked()) {
        int erosion_size = ui->erosionSlider->value();
        cv::Mat element = getStructuringElement(cv::MORPH_RECT,
                                                cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                                cv::Point( erosion_size, erosion_size ) );
        // Apply the erosion operation
        erode(final, temp, element );
        final = temp;
    }

    if (ui->checkBox->isChecked() && ui->mainCheckBox->isChecked()) {
        cv::inRange(final, cv::Scalar(ui->blueSliderMin->value(), ui->greenSliderMin->value(), ui->redSliderMin->value()),
                         cv::Scalar(ui->blueSliderMax->value(), ui->greenSliderMax->value(), ui->redSliderMax->value()), temp);
        final = temp;
    }

    QImage qImg = mat2qimage(final);
    QPixmap pixMap = QPixmap::fromImage(qImg);
    ui->imageLabel->setPixmap(pixMap);
    this->filtered = final;
}
/*
void processImages()
{
    double angle_step = 1.0;

    int steps = round(180/angle_step);

    for (int i = 0; i < steps; i++) {

        processImage();
    }

}
*/

void MainWindow::processImage(cv::Mat img, QString output_file)
{
    int cam_res_vert = img.rows;
    int cam_res_hor = img.cols;

    double radiansToDegrees = 180.0 / 3.14159265359;
    double degreesToRadians = 3.14159265359 / 180.0;

    // FIXME! Set the correct angle here
    double laser_angle = 30.0;

    // These values are not perfect, but the variance between cameras is good enough for now
    // Assumes hd resolution is used
    double cam_focal_distance_pixels = 1081.37;
//    double cam_focal_distance_mm = 3.291;
//    double cam_cx = 959.5;
//    double cam_cy = 539.5;

//    double cam_fov_hor = 84.1; // Degrees
//    double cam_fov_vert = 53.8;
//    double cam_tilt_angle = 0.0; // Degrees
    double cam_center_dist = 200.0; // mm
    double laser_cam_dist = cam_center_dist*tan(laser_angle*degreesToRadians);
    double laser_center_dist = cam_center_dist / cos(laser_angle*degreesToRadians);

    /* This assmumes you have brightness information
     * currently we only have a binary image, and so this does not work.
    for (int y = 0; y < cam_res_vert; y++) {
      // find the brightest pixel
      brightestValue = 0;
      brightestX = -1;

      for (int x = 0; x < cam_res_hor; x++) {
        int pixelValue = laserImage.pixels[index];
        float pixelBrightness = pixelValue >> 16 & 0xFF;

        if (pixelBrightness > brightestValue && pixelBrightness > threshold) {
          brightestValue = pixelBrightness;
          brightestX = x;
        }

        index++;
      }
    */
    for (int y = 0; y < cam_res_vert; y++) {

        bool found_first_x = false;
        int first_x = -1;
        int last_x = -1;

        for (int x = 0; x < cam_res_hor; x++) {
            cv::Scalar intensity = img.at<uchar>(y, x);
            if (intensity.val[0] > 200) {
                if (!found_first_x) {
                    found_first_x = true;
                    std::cout << "Found a point" << std::endl;
                    std::cout << "    X-location is " << x << std::endl;
                    first_x = x;
                } else {
                    last_x = x;
                }
            } else if (found_first_x) {
                // End of continuous segment of the line. Just break out of the loop and calculate the point.
                break;
            }
        }

        if (found_first_x) {


            double average_x = double (first_x + last_x)/2;

            double pixel_angle_x = atan2(average_x - (cam_res_hor/2), cam_focal_distance_pixels)*radiansToDegrees;

            double laser_hor_angle = (180 - 90 - laser_angle);
            double laser_cam_angle = 180 - (90 - pixel_angle_x) - laser_hor_angle;

            double laser_cam_plane_dist = laser_cam_dist/sin(laser_cam_angle*degreesToRadians)*sin((90 - pixel_angle_x)*degreesToRadians);

            double radius = laser_center_dist - laser_cam_plane_dist;

            double x_pos = sin(laser_angle*degreesToRadians)*radius;
            double y_pos = cos(laser_angle*degreesToRadians)*radius;

            double cam_cam_plane_dist = cam_center_dist - y_pos;

            double z_pos = y / cam_focal_distance_pixels * cam_cam_plane_dist;

            std::ofstream myfile;
            myfile.open(output_file.toUtf8(), std::ofstream::out | std::ofstream::app);
            myfile << x_pos << " ";
            myfile << y_pos << " ";
            myfile << z_pos << " ";
            myfile << "0.0 0.0 0.0" << std::endl;
            myfile.close();

//            float pointZ = -atan((camVFOV * degreesToRadians / 2.0)) * 2.0 * camDistance * float(y) / float(videoHeight);

            // println("line: " + y + " point: " + pointX + "," + pointY + "," + pointZ);
            // println("brightestX: " + brightestX + " camAngle: " + camAngle + " radius: " + radius);

//            thisPoint[0] = pointX;
//            thisPoint[1] = pointY;
//            thisPoint[2] = pointZ;
            // println(thisPoint);
//            pointList.add(thisPoint);
//            framePointList.add(thisPoint);

            // FIXME: these normals are bad
            // assume normals are all pointing outwards from 0,0,z = pointX,pointY,0 (should be point to camera...)
            // normalize it
            // float normalLength = sqrt((pointX * pointX) + (pointY * pointY) + (0.0 * 0.0));
            // thisNormal[0] = pointX/normalLength;
            // thisNormal[1] = pointY/normalLength;
//            thisNormal[0] = pointX;
//            thisNormal[1] = pointY;
//            thisNormal[2] = 0.0;
//            normalList.add(thisNormal);
        }
     }

}


    /*


  // code based on http://www.sjbaker.org/wiki/index.php?title=A_Simple_3D_Scanner

  laserOffset = Float.parseFloat(laserOffsetField.getText());

  // all the points in this frame ie. this spline
  ArrayList framePointList = new ArrayList();

//  println("Processing frame: " + frame + "/" + laserMovie.getFrameCount());
  laserMovie.gotoFrameNumber(frame);
  laserMovie.read();
  laserImage = laserMovie.get();

  textureMovie.gotoFrameNumber(frame);
  textureMovie.read();
  textureImage = textureMovie.get();

  int brightestX = 0;
  float brightestValue = 0;

  laserImage.loadPixels();
  textureImage.loadPixels();

  int index = 0;

  float frameAngle = float(frame) * (360.0 / float(laserMovie.getFrameCount()));

  for (int y = 0; y < videoHeight; y++) {
    // find the brightest pixel
    brightestValue = 0;
    brightestX = -1;

    for (int x = 0; x < videoWidth; x++) {
      int pixelValue = laserImage.pixels[index];
      float pixelBrightness = pixelValue >> 16 & 0xFF;

      if (pixelBrightness > brightestValue && pixelBrightness > threshold) {
        brightestValue = pixelBrightness;
        brightestX = x;
      }

      index++;
    }

    int[] thisColor = new int[3];
    float[] thisPoint = new float[3];
    float[] thisNormal = new float[3];

    if (brightestX > 0) {
      laserImage.pixels[y*videoWidth+brightestX] = color(0, 255, 0);
      float r = red(textureImage.pixels[y*videoWidth+brightestX]);
      float g = green(textureImage.pixels[y*videoWidth+brightestX]);
      float b = blue(textureImage.pixels[y*videoWidth+brightestX]);
      thisColor[0] = int(r);
      thisColor[1] = int(g);
      thisColor[2] = int(b);
      colorList.add(thisColor);

      float radius;
      float camAngle = camHFOV * (0.5 - float(brightestX) / float(videoWidth));

      float pointAngle = 180.0 - camAngle + laserOffset;
      radius = camDistance * sin(camAngle * degreesToRadians) / sin(pointAngle * degreesToRadians);

      float pointX = radius * sin(frameAngle * degreesToRadians);
      float pointY = radius * cos(frameAngle * degreesToRadians);
      float pointZ = -atan((camVFOV * degreesToRadians / 2.0)) * 2.0 * camDistance * float(y) / float(videoHeight);

      // println("line: " + y + " point: " + pointX + "," + pointY + "," + pointZ);
      // println("brightestX: " + brightestX + " camAngle: " + camAngle + " radius: " + radius);

      thisPoint[0] = pointX;
      thisPoint[1] = pointY;
      thisPoint[2] = pointZ;
      // println(thisPoint);
      pointList.add(thisPoint);
      framePointList.add(thisPoint);

      // FIXME: these normals are bad
      // assume normals are all pointing outwards from 0,0,z = pointX,pointY,0 (should be point to camera...)
      // normalize it
      // float normalLength = sqrt((pointX * pointX) + (pointY * pointY) + (0.0 * 0.0));
      // thisNormal[0] = pointX/normalLength;
      // thisNormal[1] = pointY/normalLength;
      thisNormal[0] = pointX;
      thisNormal[1] = pointY;
      thisNormal[2] = 0.0;
      normalList.add(thisNormal);

      p.addPoint(thisPoint[0], -thisPoint[2], -thisPoint[1], r/255.0, g/255.0, b/255.0, 1);
    }
  }

  splineList.add(framePointList);

  laserImage.updatePixels();
}
*/
