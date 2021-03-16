#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "ui/UIDfragSmart.hpp"

#include <iostream>
#include <cstdlib>

using namespace cv;

//Mat img, imgAux;
Mat img;
//bool isImageReady = false;
//bool isLblImReady = false;
//bool isPointcloudReady = false;
//pcl::visualization::PCLVisualizer::Ptr viewer;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr zedCloudRaw (new pcl::PointCloud<pcl::PointXYZRGB>);
UIDfragSmart* window;
//unsigned int thumbnailCounter = 0;
//std::vector<QImage> imVector;
//const unsigned int MAX_THUMBAILS = 10;

/*UIDfragSmart::UIDfragSmart()
{

		  ui.setupUi(this);

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
      //ui.verticalLayout->addWidget(qvtkWidget);

      viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
      qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
      viewer->setupInteractor (qvtkWidget->GetInteractor (), qvtkWidget->GetRenderWindow ());
      viewer->addPointCloud (zedCloudRaw, "zedCloudRaw");
      viewer->resetCamera ();
      
      isLblImReady = true;

      model = new QStandardItemModel(this);
      model->insertColumn(0);
      model->insertRows(0,MAX_THUMBAILS);
      imVector = std::vector<QImage>(MAX_THUMBAILS);
  
      this->setWindowState(Qt::WindowMaximized);
		  connect (ui.btnImagen,  SIGNAL (clicked ()), this, SLOT (btnImagenPressed ()));
      connect (ui.btnPointcloud,  SIGNAL (clicked ()), this, SLOT (btnPointcloudPressed ()));
      connect (ui.btnFoto,  SIGNAL (clicked ()), this, SLOT (btnFotoPressed ()));
      timer = new QTimer(this);
	   	connect(timer, SIGNAL(timeout()), this, SLOT(displayImage()) );
		  timer->start(200);
}

UIDfragSmart::~UIDfragSmart()
{
		  delete timer;
      //delete lblIm;
      delete qvtkWidget;
}*/

/*void UIDfragSmart::displayImage()
{ 
  ros::spinOnce(); 
  if(isImageReady && isLblImReady)
  {
    cv::cvtColor(img, imgAux, cv::COLOR_BGR2RGB);
    imDisplay = QImage((const unsigned char*)imgAux.data, // uchar* data
          imgAux.cols, imgAux.rows, imgAux.step, QImage::Format_RGB888); //Format conversion
        
    ui.lblIm->setPixmap(QPixmap::fromImage(imDisplay));      
  }  
}*/

/*void UIDfragSmart::btnImagenPressed()
{
  ui.gridLayout->removeWidget(qvtkWidget);
  qvtkWidget->setParent(NULL);
  isLblImReady = true;
  
  ui.lblIm->setParent(ui.centralwidget);
  ui.gridLayout->addWidget(ui.lblIm, 0, 0, 1, 1);
  ui.gridLayout->update();
}*/

/*void UIDfragSmart::btnPointcloudPressed()
{
  //ui.verticalLayout->removeWidget(ui.lstIm);
  //ui.lstIm->setParent(NULL);
  ui.gridLayout->removeWidget(ui.lblIm);
  ui.lblIm->setParent(NULL);  
  isLblImReady = false;
  
  //ui.lstIm->setParent(ui.centralwidget);
  //ui.verticalLayout->addWidget(ui.lstIm);
  qvtkWidget->setParent(ui.centralwidget);
  ui.gridLayout->addWidget(qvtkWidget, 0, 0, 1, 1);
  ui.gridLayout->update();

  //viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  //qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  //viewer->setupInteractor (qvtkWidget->GetInteractor (), qvtkWidget->GetRenderWindow ());
  viewer->updatePointCloud (zedCloudRaw, "zedCloudRaw");
  qvtkWidget->update ();  
}



void UIDfragSmart::btnFotoPressed()
{
  QImage thumbnail = imDisplay;
  thumbnail = thumbnail.scaled(200, 100, Qt::IgnoreAspectRatio, Qt::FastTransformation);
  
  imVector[thumbnailCounter] = thumbnail;

  //for(int i=0;i<numRows ;++i)
  //QPixmap pixmap = QPixmap::fromImage(thumbnail);  
  for(int i=0; i<MAX_THUMBAILS; ++i)
    model->setData(model->index(i,0),QPixmap::fromImage(imVector[i]),Qt::DecorationRole); 
  
  //model->setData(model->index(0,0),QPixmap::fromImage(imVector[thumbnailCounter]),Qt::DecorationRole); 

  //std::cout<<imVector.size()<<std::endl;
  ui.lstIm->setModel(model);
  thumbnailCounter = (thumbnailCounter==MAX_THUMBAILS-1)?(MAX_THUMBAILS-1):(thumbnailCounter+1);
}*/

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  img = cv_bridge::toCvShare(msg, "bgr8")->image;
  window->isImageReady = true;
}

void zedCloudHandler(const sensor_msgs::PointCloud2ConstPtr& zedCloudMsg)
{  
	//double timeScanCur = zedCloudMsg->header.stamp.toSec();
	pcl::fromROSMsg(*zedCloudMsg, *zedCloudRaw);
  window->isPointcloudReady = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rock_metric");
  ros::NodeHandle nh;
  
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/zed/zed_node/rgb/image_rect_color", 1, imageCallback);

  ros::Subscriber subZedCloud = nh.subscribe<sensor_msgs::PointCloud2>
		                        ("/zed/zed_node/point_cloud/cloud_registered", 2, zedCloudHandler);

  QApplication a (argc, argv);
  window = new UIDfragSmart(&img, zedCloudRaw);
	window->show();
  int r = a.exec();

  delete window;
 
  return r;
}
