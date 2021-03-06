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
#include "std_msgs/String.h"
#include <sstream>

#include <chrono>

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

  QPushButton *btnTomarFoto;
  QPushButton *btnGuardarFoto;
  //QPushButton *btnRegresarAFoto;
  QImage imDisplay;
  QTimer* timer;
  QStandardItemModel *model;
  QVTKWidget *qvtkWidget;  

  unsigned int thumbnailCounter = 0;
  const unsigned int MAX_THUMBAILS = 10;
  const unsigned char VIEWER_IMAGE = 0;
  const unsigned char VIEWER_POINTCLOUD = 1;
  const unsigned char VIEWER_PHOTO = 2;
  std::vector<QImage> imVector;

  bool isImageReady = false;
  bool isPhotoTaken = false;
  //bool isPointcloudReady = false;
  //bool isQvtkCloudReady = false;
  unsigned char viewerState = VIEWER_IMAGE;
  
  Mat *img;
  pcl::visualization::PCLVisualizer::Ptr viewer;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr zedCloudRaw;
  pcl::PointCloud<pcl::PointXYZRGB> zedCloudRawPhoto;

  Ui::mainWindow ui;

  ros::Publisher* pubGainExposure;

  UIDfragSmart(ros::Publisher* pubGainExposure, Mat* img, pcl::PointCloud<pcl::PointXYZRGB>::Ptr zedCloudRaw)
  {
    ui.setupUi(this);

    this->pubGainExposure = pubGainExposure;
    this->img = img;
    this->zedCloudRaw = zedCloudRaw;

    ui.btnPointcloud->setEnabled(false);

    btnTomarFoto = new QPushButton(ui.centralwidget);
    btnTomarFoto->setObjectName(QStringLiteral("btnTomarFoto"));
    btnTomarFoto->setText(QApplication::translate("mainWindow", "Shoot", Q_NULLPTR));
    ui.gridLayout->addWidget(btnTomarFoto, 0, 0, 1, 1, Qt::AlignLeft|Qt::AlignBottom);

    btnGuardarFoto = new QPushButton(ui.centralwidget);
    btnGuardarFoto->setObjectName(QStringLiteral("btnGuardarFoto"));
    btnGuardarFoto->setText(QApplication::translate("mainWindow", "Guardar Foto", Q_NULLPTR));
    ui.gridLayout->addWidget(btnGuardarFoto, 0, 0, 1, 1, Qt::AlignRight|Qt::AlignBottom);
    btnGuardarFoto->setVisible(false);

    //btnRegresarAFoto = new QPushButton(ui.centralwidget);
    //btnRegresarAFoto->setObjectName(QStringLiteral("btnRegresarAFoto"));
    //btnRegresarAFoto->setText(QApplication::translate("mainWindow", "Regresar a Foto", Q_NULLPTR));
    //ui.gridLayout->addWidget(btnRegresarAFoto, 0, 0, 1, 1, Qt::AlignLeft|Qt::AlignTop);
    //btnRegresarAFoto->setVisible(false);

    ui.btnRegresarAFoto->setEnabled(false);

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
    connect (btnTomarFoto,  SIGNAL (clicked ()), this, SLOT (btnTomarFotoPressed ()));
    connect (btnGuardarFoto,  SIGNAL (clicked ()), this, SLOT (btnGuardarFotoPressed ()));
    connect (ui.btnRegresarAFoto,  SIGNAL (clicked ()), this, SLOT (btnRegresarAFotoPressed ()));
    connect(ui.sldGain, SIGNAL(valueChanged(int)), this, SLOT(onGainSliderChange(int)));
    connect(ui.sldExposure, SIGNAL(valueChanged(int)), this, SLOT(onExposureSliderChange(int)));
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(displayImage()) );
    //timer->start(200);
    timer->start(199);

    //isLblImReady = true;
    //isQvtkCloudReady = true;
  }

  ~UIDfragSmart()
  {
    delete qvtkWidget;
    delete timer;
    delete btnTomarFoto;
    delete model;
    delete img;
    delete pubGainExposure;
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
    delete ui.sldGain;
    delete ui.sldExposure;
    delete ui.btnRegresarAFoto;
  }

  Q_OBJECT
  public Q_SLOTS:
  void displayImage()
  {
    ros::spinOnce(); 
    //if(isImageReady && isLblImReady)
    if(isImageReady)
    {
      auto start = std::chrono::steady_clock::now();
      cv::cvtColor(*img, *img, cv::COLOR_BGR2RGB);
      imDisplay = QImage((const unsigned char*)img->data, // uchar* data
            img->cols, img->rows, img->step, QImage::Format_RGB888); //Format conversion
          
      ui.lblIm->setPixmap(QPixmap::fromImage(imDisplay));
      auto end = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
      //std::cout<<elapsed.count()<<std::endl;
      timer->setInterval(200-elapsed.count());

    }
    isImageReady = false; 
  }

  void btnImagenPressed()
  {
    //if(viewerState==VIEWER_POINTCLOUD)
    if(isPhotoTaken)
    {
      //isLblImReady = true;
      //isQvtkCloudReady = false;
      isPhotoTaken = false;
      ui.lblIm->setParent(ui.centralwidget);
      ui.gridLayout->addWidget(ui.lblIm, 0, 0, 1, 1);
      
      btnTomarFoto->setParent(ui.centralwidget);
      ui.gridLayout->addWidget(btnTomarFoto, 0, 0, 1, 1, Qt::AlignLeft|Qt::AlignBottom);
      btnTomarFoto->setVisible(true);

      btnGuardarFoto->setParent(ui.centralwidget);
      ui.gridLayout->addWidget(btnGuardarFoto, 0, 0, 1, 1, Qt::AlignRight|Qt::AlignBottom);
      btnGuardarFoto->setVisible(false);

      ui.gridLayout->removeWidget(qvtkWidget);
      qvtkWidget->setParent(NULL);

      ui.btnPointcloud->setEnabled(false);
      ui.btnRegresarAFoto->setEnabled(false);
      ui.gridLayout->update();
      viewerState=VIEWER_IMAGE;
    }
  }

  void btnPointcloudPressed()
  { 
    //if(viewerState==VIEWER_IMAGE && !isImageReady && !isLblImReady)// Si se tomo foto
    //if(viewerState==VIEWER_IMAGE || && isPhotoTaken)// Si se tomo foto
    if(isPhotoTaken)// Si se tomo foto
    {
      //isLblImReady = false;
      //isQvtkCloudReady = true;
      qvtkWidget->setParent(ui.centralwidget);
      ui.gridLayout->addWidget(qvtkWidget, 0, 0, 1, 1);
      ui.gridLayout->removeWidget(ui.lblIm);
      ui.lblIm->setParent(NULL);
      
      ui.gridLayout->removeWidget(btnTomarFoto);
      btnTomarFoto->setParent(NULL);

      ui.gridLayout->removeWidget(btnGuardarFoto);
      btnGuardarFoto->setParent(NULL);

      //ui.gridLayout->addWidget(btnRegresarAFoto, 0, 0, 1, 1, Qt::AlignLeft|Qt::AlignTop);
      //btnRegresarAFoto->setVisible(true);
      ui.btnRegresarAFoto->setEnabled(true);
      ui.gridLayout->update();    
      
      //viewer->updatePointCloud (zedCloudRaw, "zedCloudRaw");
      //pcl::PointCloud<pcl::PointXYZRGB>::Ptr zedCloudRawPhotoAux(new pcl::PointCloud<pcl::PointXYZRGB>);;
      //*zedCloudRawPhotoAux = zedCloudRawPhoto;
      //viewer->updatePointCloud (zedCloudRawPhotoAux, "zedCloudRaw");
      viewer->updatePointCloud (zedCloudRaw, "zedCloudRaw");
      qvtkWidget->update ();
      viewerState=VIEWER_POINTCLOUD;
    }
  }

  void btnTomarFotoPressed()
  {
    isImageReady = false;
    //isLblImReady = false;
    isPhotoTaken = true;
    btnTomarFoto->setVisible(false);
    btnGuardarFoto->setVisible(true);
    ui.btnPointcloud->setEnabled(true);
    //zedCloudRawPhoto = *zedCloudRaw;   
    /*QImage thumbnail = imDisplay;
    thumbnail = thumbnail.scaled(200, 100, Qt::IgnoreAspectRatio, Qt::FastTransformation);
    
    imVector[thumbnailCounter] = thumbnail;

    for(int i=0; i<MAX_THUMBAILS; ++i)
      model->setData(model->index(i,0),QPixmap::fromImage(imVector[i]),Qt::DecorationRole); 
    
    ui.lstIm->setModel(model);
    thumbnailCounter = (thumbnailCounter==MAX_THUMBAILS-1)?(MAX_THUMBAILS-1):(thumbnailCounter+1);*/
  }

  void btnRegresarAFotoPressed()
  {
    if(viewerState==VIEWER_POINTCLOUD && isPhotoTaken)
    {      
      //isPhotoTaken = false; //Activa el recibimiento de imagenes
      ui.lblIm->setParent(ui.centralwidget);
      ui.gridLayout->addWidget(ui.lblIm, 0, 0, 1, 1);
      
      btnTomarFoto->setParent(ui.centralwidget);
      ui.gridLayout->addWidget(btnTomarFoto, 0, 0, 1, 1, Qt::AlignLeft|Qt::AlignBottom);
      btnTomarFoto->setVisible(false);

      btnGuardarFoto->setParent(ui.centralwidget);
      ui.gridLayout->addWidget(btnGuardarFoto, 0, 0, 1, 1, Qt::AlignRight|Qt::AlignBottom);
      btnGuardarFoto->setVisible(true);

      ui.gridLayout->removeWidget(qvtkWidget);
      qvtkWidget->setParent(NULL);

      ui.btnPointcloud->setEnabled(true);
      ui.btnRegresarAFoto->setEnabled(false);
      ui.gridLayout->update();
      viewerState=VIEWER_PHOTO;
    }
  }

  void btnGuardarFotoPressed()
  { 
    //isQvtkCloudReady = true;
    QImage thumbnail = imDisplay;
    thumbnail = thumbnail.scaled(200, 100, Qt::IgnoreAspectRatio, Qt::FastTransformation);
    
    imVector[thumbnailCounter] = thumbnail;

    for(int i=0; i<MAX_THUMBAILS; ++i)
      model->setData(model->index(i,0),QPixmap::fromImage(imVector[i]),Qt::DecorationRole); 
    
    ui.lstIm->setModel(model);
    thumbnailCounter = (thumbnailCounter==MAX_THUMBAILS-1)?(MAX_THUMBAILS-1):(thumbnailCounter+1);

    //ui.btnPointcloud->setEnabled(true);
    //zedCloudRawPhoto = *zedCloudRaw;
    //isQvtkCloudReady = false;
  }

  void onGainSliderChange(int position) 
  {
    // Calculate float position of slider
    //float positionF = position / float(STEPS);
    //m_textEdit->setText( QString::number(positionF, 'f', 2) );
    
    //std::cout<<position<<std::endl;
    std_msgs::String msg;
    std::stringstream ss;
    ss << "G" << position;
    msg.data = ss.str();
    pubGainExposure->publish(msg);
  }

  void onExposureSliderChange(int position) 
  {
    // Calculate float position of slider
    //float positionF = position / float(STEPS);
    //m_textEdit->setText( QString::number(positionF, 'f', 2) );
    
    //std::cout<<position<<std::endl;
    std_msgs::String msg;
    std::stringstream ss;
    ss << "E"<<position;
    msg.data = ss.str();
    pubGainExposure->publish(msg);
  }

};

QT_END_NAMESPACE

#endif // UIDFRAGSMART_H
