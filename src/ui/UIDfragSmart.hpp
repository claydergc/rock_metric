#ifndef UIDFRAGSMART_H
#define UIDFRAGSMART_H

#include "UIMain.hpp"
#include <QTimer>
#include <QStandardItemModel>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include "QVTKWidget.h"

//OpenCV
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <ros/ros.h>

using namespace cv;

QT_BEGIN_NAMESPACE

class UIDfragSmart : public QMainWindow
{

	//Q_OBJECT
  //public slots:    // A slot or function is defined which will be intiated by timer
  
    //public Q_SLOTS:
  /*void displayImage();
  void btnImagenPressed();
  void btnPointcloudPressed();
  void btnFotoPressed();*/
    
public:

  QPushButton *btnFoto;
  QImage imDisplay;
  QTimer* timer;
  QStandardItemModel *model;
  QVTKWidget *qvtkWidget;  

  unsigned int thumbnailCounter = 0;
  const unsigned int MAX_THUMBAILS = 10;
  std::vector<QImage> imVector;

  bool isImageReady = false;
  bool isLblImReady = false;
  bool isPointcloudReady = false;
  bool isQvtkCloudReady = false;
  
  Mat *img;
  pcl::visualization::PCLVisualizer::Ptr viewer;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr zedCloudRaw;

  Ui::mainWindow ui;

  UIDfragSmart(Mat* img, pcl::PointCloud<pcl::PointXYZRGB>::Ptr zedCloudRaw)
  {
    ui.setupUi(this);

    this->img = img;
    this->zedCloudRaw = zedCloudRaw;

    btnFoto = new QPushButton(ui.centralwidget);
    btnFoto->setObjectName(QStringLiteral("btnFoto"));
    btnFoto->setText(QApplication::translate("mainWindow", "Shoot", Q_NULLPTR));
    ui.gridLayout->addWidget(btnFoto, 0, 0, 1, 1, Qt::AlignLeft|Qt::AlignBottom);

    qvtkWidget = new QVTKWidget();      
    qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));
    QSizePolicy sizePolicy2(QSizePolicy::Expanding, QSizePolicy::Expanding);
    sizePolicy2.setHorizontalStretch(0);
    sizePolicy2.setVerticalStretch(0);
    sizePolicy2.setHeightForWidth(qvtkWidget->sizePolicy().hasHeightForWidth());
    qvtkWidget->setSizePolicy(sizePolicy2);
    qvtkWidget->setMinimumSize(QSize(1280, 720));
    qvtkWidget->setMaximumSize(QSize(1280, 720));
    qvtkWidget->setLayoutDirection(Qt::LeftToRight);
    
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (qvtkWidget->GetInteractor (), qvtkWidget->GetRenderWindow ());
    viewer->addPointCloud (zedCloudRaw, "zedCloudRaw");
    viewer->resetCamera ();  

    model = new QStandardItemModel(this);
    model->insertColumn(0);
    model->insertRows(0,MAX_THUMBAILS);
    imVector = std::vector<QImage>(MAX_THUMBAILS);

    this->setWindowState(Qt::WindowMaximized);
    connect (ui.btnImagen,  SIGNAL (clicked ()), this, SLOT (btnImagenPressed ()));
    connect (ui.btnPointcloud,  SIGNAL (clicked ()), this, SLOT (btnPointcloudPressed ()));
    connect (btnFoto,  SIGNAL (clicked ()), this, SLOT (btnFotoPressed ()));
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(displayImage()) );
    timer->start(200);

    isLblImReady = true;
    isQvtkCloudReady = true;
  }

  ~UIDfragSmart()
  {
    delete qvtkWidget;
    delete timer;
    delete btnFoto;
    delete model;
    delete img;
    delete ui.centralwidget;
    delete ui.horizontalLayout;
    delete ui.verticalLayout_3;
    delete ui.lstProjects;
    delete ui.horizontalLayout_2;
    delete ui.pushButton_3;
    delete ui.pushButton_4;
    delete ui.pushButton_5;
    delete ui.gridLayout;
    delete ui.lblIm;
    delete ui.lstIm;
    delete ui.verticalLayout_2;
    delete ui.btnImagen;
    delete ui.btnPointcloud;    
  }

  Q_OBJECT
  public Q_SLOTS:
  void displayImage()
  {
    ros::spinOnce(); 
    if(isImageReady && isLblImReady)
    {
      cv::cvtColor(*img, *img, cv::COLOR_BGR2RGB);
      imDisplay = QImage((const unsigned char*)img->data, // uchar* data
            img->cols, img->rows, img->step, QImage::Format_RGB888); //Format conversion
          
      ui.lblIm->setPixmap(QPixmap::fromImage(imDisplay));
    }
    isImageReady = false; 
  }

  void btnImagenPressed()
  {
    isLblImReady = true;
    isQvtkCloudReady = false;
    ui.lblIm->setParent(ui.centralwidget);
    ui.gridLayout->addWidget(ui.lblIm, 0, 0, 1, 1);
    btnFoto->setParent(ui.centralwidget);
    ui.gridLayout->addWidget(btnFoto, 0, 0, 1, 1, Qt::AlignLeft|Qt::AlignBottom);
    ui.gridLayout->update();
    ui.gridLayout->removeWidget(qvtkWidget);
    qvtkWidget->setParent(NULL);
  }

  void btnPointcloudPressed()
  { 
    isLblImReady = false;
    isQvtkCloudReady = true;   
    qvtkWidget->setParent(ui.centralwidget);
    ui.gridLayout->addWidget(qvtkWidget, 0, 0, 1, 1);
    ui.gridLayout->removeWidget(ui.lblIm);
    ui.lblIm->setParent(NULL);
    ui.gridLayout->removeWidget(btnFoto);
    btnFoto->setParent(NULL);    
    ui.gridLayout->update();    
    
    viewer->updatePointCloud (zedCloudRaw, "zedCloudRaw");
    qvtkWidget->update ();
  }

  void btnFotoPressed()
  {
    QImage thumbnail = imDisplay;
    thumbnail = thumbnail.scaled(200, 100, Qt::IgnoreAspectRatio, Qt::FastTransformation);
    
    imVector[thumbnailCounter] = thumbnail;

    for(int i=0; i<MAX_THUMBAILS; ++i)
      model->setData(model->index(i,0),QPixmap::fromImage(imVector[i]),Qt::DecorationRole); 
    
    ui.lstIm->setModel(model);
    thumbnailCounter = (thumbnailCounter==MAX_THUMBAILS-1)?(MAX_THUMBAILS-1):(thumbnailCounter+1);
  }

};

QT_END_NAMESPACE

#endif // UIDFRAGSMART_H
