///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2010, Jason Mora Saragih, all rights reserved.
//
// This file is part of FaceTracker.
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//
//     * The software is provided under the terms of this licence stricly for
//       academic, non-commercial, not-for-profit purposes.
//     * Redistributions of source code must retain the above copyright notice, 
//       this list of conditions (licence) and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright 
//       notice, this list of conditions (licence) and the following disclaimer 
//       in the documentation and/or other materials provided with the 
//       distribution.
//     * The name of the author may not be used to endorse or promote products 
//       derived from this software without specific prior written permission.
//     * As this software depends on other libraries, the user must adhere to 
//       and keep in place any licencing terms of those libraries.
//     * Any publications arising from the use of this software, including but
//       not limited to academic journal and conference publications, technical
//       reports and manuals, must cite the following work:
//
//       J. M. Saragih, S. Lucey, and J. F. Cohn. Face Alignment through 
//       Subspace Constrained Mean-Shifts. International Conference of Computer 
//       Vision (ICCV), September, 2009.
//
// THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED 
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO 
// EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF 
// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///////////////////////////////////////////////////////////////////////////////
#include <FaceTracker/Tracker.h>
#include <opencv/highgui.h>
#include <opencv2/imgcodecs.hpp>
#include <iostream>
//=============================================================================
void Draw(cv::Mat &image,cv::Mat &shape,cv::Mat &con,cv::Mat &tri,cv::Mat &visi)
{
  int i,n = shape.rows/2; cv::Point p1,p2; cv::Scalar c;

  //draw triangulation
  c = CV_RGB(0,0,0);
  for(i = 0; i < tri.rows; i++){
    if(visi.at<int>(tri.at<int>(i,0),0) == 0 ||
       visi.at<int>(tri.at<int>(i,1),0) == 0 ||
       visi.at<int>(tri.at<int>(i,2),0) == 0)continue;
    p1 = cv::Point(shape.at<double>(tri.at<int>(i,0),0),
		   shape.at<double>(tri.at<int>(i,0)+n,0));
    p2 = cv::Point(shape.at<double>(tri.at<int>(i,1),0),
		   shape.at<double>(tri.at<int>(i,1)+n,0));
    cv::line(image,p1,p2,c);
    p1 = cv::Point(shape.at<double>(tri.at<int>(i,0),0),
		   shape.at<double>(tri.at<int>(i,0)+n,0));
    p2 = cv::Point(shape.at<double>(tri.at<int>(i,2),0),
		   shape.at<double>(tri.at<int>(i,2)+n,0));
    cv::line(image,p1,p2,c);
    p1 = cv::Point(shape.at<double>(tri.at<int>(i,2),0),
		   shape.at<double>(tri.at<int>(i,2)+n,0));
    p2 = cv::Point(shape.at<double>(tri.at<int>(i,1),0),
		   shape.at<double>(tri.at<int>(i,1)+n,0));
    cv::line(image,p1,p2,c);
  }
  //draw connections
  c = CV_RGB(0,0,255);
  for(i = 0; i < con.cols; i++){
    if(visi.at<int>(con.at<int>(0,i),0) == 0 ||
       visi.at<int>(con.at<int>(1,i),0) == 0)continue;
    p1 = cv::Point(shape.at<double>(con.at<int>(0,i),0),
		   shape.at<double>(con.at<int>(0,i)+n,0));
    p2 = cv::Point(shape.at<double>(con.at<int>(1,i),0),
		   shape.at<double>(con.at<int>(1,i)+n,0));
    cv::line(image,p1,p2,c,1);
  }
  //draw points
  for(i = 0; i < n; i++){    
    if(visi.at<int>(i,0) == 0)continue;
    p1 = cv::Point(shape.at<double>(i,0),shape.at<double>(i+n,0));
    c = CV_RGB(255,0,0); cv::circle(image,p1,2,c);
  }return;
}
//=============================================================================
//http://jepsonsblog.blogspot.com/2012/10/overlay-transparent-image-in-opencv.html
void overlayImage(const cv::Mat &background, const cv::Mat &frgnd,
                  //                  cv::Mat &output,
                  cv::Point2i location, double scale)
{
    //    background.copyTo(output);
    cv::Mat foreground;
    cv::resize(frgnd, foreground, cv::Size(scale*frgnd.cols,scale*frgnd.rows));
    
    // start at the row indicated by location, or at row 0 if location.y is negative.
    for(int y = std::max(location.y , 0); y < background.rows; ++y)
    {
        int fY = y - location.y; // because of the translation
        
        // we are done of we have processed all rows of the foreground image.
        if(fY >= foreground.rows)
            break;
        
        // start at the column indicated by location,
        
        // or at column 0 if location.x is negative.
        for(int x = std::max(location.x, 0); x < background.cols; ++x)
        {
            int fX = x - location.x; // because of the translation.
            
            // we are done with this row if the column is outside of the foreground image.
            if(fX >= foreground.cols)
                break;
            
            // determine the opacity of the foregrond pixel, using its fourth (alpha) channel.
            double opacity =
            ((double)foreground.data[fY * foreground.step + fX * foreground.channels() + 3])
            
            / 255.;
            
            
            // and now combine the background and foreground pixel, using the opacity,
            
            // but only if opacity > 0.
            for(int c = 0; opacity > 0 && c < background.channels(); ++c)
            {
                unsigned char foregroundPx =
                foreground.data[fY * foreground.step + fX * foreground.channels() + c];
                unsigned char backgroundPx =
                background.data[y * background.step + x * background.channels() + c];
                background.data[y*background.step + background.channels()*x + c] =
                backgroundPx * (1.-opacity) + foregroundPx * opacity;
            }
        }
    }
}
//=============================================================================
void Draw(cv::Mat &image,cv::Mat &shape,cv::Mat &visi,cv::Mat &overlay)
{
  int i,n = shape.rows/2; cv::Point p1,p2; cv::Scalar c;
  char sss[256]; std::string text;
    double x0,x16; double scale;
//0
//1
//15
//16
  if(visi.at<int>(0,0) != 0){
//    p2=cv::Point(shape.at<double>(16,0),shape.at<double>(16 + n,0));
    x16 = shape.at<double>(16,0);
    x0 = shape.at<double>(0,0);
    scale = (x16 - x0)/752.0;
    p2=cv::Point(x0,shape.at<double>(36+n,0)-(207*scale));
    overlayImage(image, overlay, p2, scale);
  }
  //draw numbers
//  for(i = 0; i < n; i++){
//    if(visi.at<int>(i,0) == 0)continue;
//    p1 = cv::Point(shape.at<double>(i,0),shape.at<double>(i+n,0));
//    if (i==0 || i==16){
//        sprintf(sss,"%d(%d)",(int) i, p1.x);
//    }else{
//        sprintf(sss,"%d",(int) i);
//    }
//    text = sss;
//    c = CV_RGB(255,0,0); //cv::circle(image,p1,2,c);
//    cv::putText(image,text,p1, CV_FONT_HERSHEY_SIMPLEX,0.5,c);
//  }
  return;
}
//Background removing
//https://docs.opencv.org/3.3.0/db/d5c/tutorial_py_bg_subtraction.html
//=============================================================================
//https://stackoverflow.com/questions/20957433/how-to-draw-a-transparent-image-over-live-camera-feed-in-opencv
//cv::Mat display_img( src.size(), src.type() ){
//    for (int y = 0; y < src.rows; y++)
//    {
//        const cv::Vec3b* src_pixel = src.ptr<cv::Vec3b>(y);
//        const cv::Vec4b* ovl_pixel = overlay.ptr<cv::Vec4b>(y);
//        cv::Vec3b* dst_pixel = display_img.ptr<cv::Vec3b>(y);
//        for (int x = 0; x < src.cols; x++, ++src_pixel, ++ovl_pixel, ++dst_pixel)
//        {
//            double alpha = (*ovl_pixel).val[3] / 255.0;
//            for (int c = 0; c < 3; c++)
//            {
//                (*dst_pixel).val[c] = (uchar) ((*ovl_pixel).val[c] * alpha + (*src_pixel).val[c] * (1.0 -alpha));
//            }
//        }
//    }
//}
//=============================================================================
int parse_cmd(int argc, const char** argv,
	      char* ftFile,char* conFile,char* triFile,
	      bool &fcheck,double &scale,int &fpd)
{
  int i; fcheck = false; scale = 1; fpd = -1;
  for(i = 1; i < argc; i++){
    if((std::strcmp(argv[i],"-?") == 0) ||
       (std::strcmp(argv[i],"--help") == 0)){
      std::cout << "track_face:- Written by Jason Saragih 2010" << std::endl
	   << "Performs automatic face tracking" << std::endl << std::endl
	   << "#" << std::endl 
	   << "# usage: ./face_tracker [options]" << std::endl
	   << "#" << std::endl << std::endl
	   << "Arguments:" << std::endl
	   << "-m <string> -> Tracker model (default: model/face2.tracker)"
	   << std::endl
	   << "-c <string> -> Connectivity (default: model/face.con)"
	   << std::endl
	   << "-t <string> -> Triangulation (default: model/face.tri)"
	   << std::endl
	   << "-s <double> -> Image scaling (default: 1)" << std::endl
	   << "-d <int>    -> Frames/detections (default: -1)" << std::endl
	   << "--check     -> Check for failure" << std::endl;
      return -1;
    }
  }
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"--check") == 0){fcheck = true; break;}
  }
  if(i >= argc)fcheck = false;
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"-s") == 0){
      if(argc > i+1)scale = std::atof(argv[i+1]); else scale = 1;
      break;
    }
  }
  if(i >= argc)scale = 1;
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"-d") == 0){
      if(argc > i+1)fpd = std::atoi(argv[i+1]); else fpd = -1;
      break;
    }
  }
  if(i >= argc)fpd = -1;
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"-m") == 0){
      if(argc > i+1)std::strcpy(ftFile,argv[i+1]);
      else strcpy(ftFile,"model/face2.tracker");
      break;
    }
  }
  if(i >= argc)std::strcpy(ftFile,"model/face2.tracker");
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"-c") == 0){
      if(argc > i+1)std::strcpy(conFile,argv[i+1]);
      else strcpy(conFile,"model/face.con");
      break;
    }
  }
  if(i >= argc)std::strcpy(conFile,"model/face.con");
  for(i = 1; i < argc; i++){
    if(std::strcmp(argv[i],"-t") == 0){
      if(argc > i+1)std::strcpy(triFile,argv[i+1]);
      else strcpy(triFile,"model/face.tri");
      break;
    }
  }
  if(i >= argc)std::strcpy(triFile,"model/face.tri");
  return 0;
}
//=============================================================================
int main(int argc, const char** argv)
{
  //parse command line arguments
  char ftFile[256],conFile[256],triFile[256];
  bool fcheck = false; double scale = 1; int fpd = -1; bool show = true;
  if(parse_cmd(argc,argv,ftFile,conFile,triFile,fcheck,scale,fpd)<0)return 0;

  //set other tracking parameters
  std::vector<int> wSize1(1); wSize1[0] = 7;
  std::vector<int> wSize2(3); wSize2[0] = 11; wSize2[1] = 9; wSize2[2] = 7;
  int nIter = 5; double clamp=3,fTol=0.01; 
  FACETRACKER::Tracker model(ftFile);
  cv::Mat tri=FACETRACKER::IO::LoadTri(triFile);
  cv::Mat con=FACETRACKER::IO::LoadCon(conFile);
  
  //initialize camera and display window
  cv::Mat frame,gray,im; double fps=0; char sss[256]; std::string text;

  //https://stackoverflow.com/questions/34497099/opencv-undefined-reference-to-imread
  model._googles = cv::imread("res/eyes.png" , cv::IMREAD_UNCHANGED);

  cv::VideoCapture camera(CV_CAP_ANY); if(!camera.isOpened()) return -1;
  int64 t1,t0 = cvGetTickCount(); int fnum=0;
  cvNamedWindow("Face Tracker",1);
  std::cout << "Hot keys: "        << std::endl
	    << "\t ESC - quit"     << std::endl
	    << "\t d   - Redetect" << std::endl;

  //loop until quit (i.e user presses ESC)
  bool failed = true;
  while(1){ 
    //grab image, resize and flip
    camera.read(frame);
    if(scale == 1)im = frame; 
    else cv::resize(frame,im,cv::Size(scale*frame.cols,scale*frame.rows));
    cv::flip(im,im,1); cv::cvtColor(im,gray,CV_BGR2GRAY);

    //track this image
    std::vector<int> wSize; if(failed)wSize = wSize2; else wSize = wSize1; 
    if(model.Track(gray,wSize,fpd,nIter,clamp,fTol,fcheck) == 0){
      int idx = model._clm.GetViewIdx(); failed = false;
      Draw(im,model._shape,model._clm._visi[idx], model._googles);
    }else{
      if(show){cv::Mat R(im,cvRect(0,0,150,50)); R = cv::Scalar(0,0,255);}
      model.FrameReset(); failed = true;
    }     
    //draw framerate on display image 
    if(fnum >= 9){      
      t1 = cvGetTickCount();
      fps = 10.0/((double(t1-t0)/cvGetTickFrequency())/1e+6); 
      t0 = t1; fnum = 0;
    }else fnum += 1;
    if(show){
      sprintf(sss,"%d frames/sec",(int)round(fps)); text = sss;
      cv::putText(im,text,cv::Point(10,20),
		  CV_FONT_HERSHEY_SIMPLEX,0.5,CV_RGB(255,255,255));
    }
    //show image and check for user input
    imshow("Face Tracker",im); 
    int c = cvWaitKey(5);
    if(c == 27)break; else if(char(c) == 'd')model.FrameReset();
  }return 0;
}
//=============================================================================
