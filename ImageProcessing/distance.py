//This code:                                                                              //
// Given a list of chessboard images, the number of corners (nx, ny)                      //
// on the chessboards, và a flag called useCalibrated (0 for Hartley                      //
// hay 1 for Bouguet stereo methods). Calibrate the cameras và display the                //
// rectified các kết quả along with the computed disparity images.                        //
////////////////////////////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include "cv.h"
#include "cxmisc.h"
#include "highgui.h"
#include "cvaux.h"
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <ctype.h>
#include <iostream>
#include <core\core.hpp>
using namespace cv;
using namespace std;
CvCapture*capture0;
CvCapture*capture1;
 
// helper function:
// find angle between vectors from pt0->pt1 and from pt0->pt2
static double angle( CvPoint *pt1 ,CvPoint *pt2, CvPoint *pt0 )
{
    double dx1 = pt1->x - pt0->x;
    double dy1 = pt1->y - pt0->y;
    double dx2 = pt2->x - pt0->x;
    double dy2 = pt2->y - pt0->y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}
// helper function:
// find rectangle on picture--> using Threshold,Canny-->FindContours-->Check-Filter (4 edges, Area>900,Cosin of all angles <0.15)
// return the coordinate of rectangle's center on image
// draw the rectangle on image
static CvPoint RecCenter(CvMat* image)
{
    //Finding Rectangels on Image
    IplImage* imageTh = cvCreateImage(cvGetSize(image), 8, 1);
    cvThreshold(image,imageTh,128,255,CV_THRESH_BINARY);
    cvCanny(imageTh,imageTh,0,70,5);
    CvSeq * contours;//hold the pointer to a contour in the memory block
    CvSeq* result;//hold sequence of points of a contour
    CvMemStorage *storage = cvCreateMemStorage(0); //storage area for all contours
    //finding all contours in the image
    cvFindContours(imageTh, storage,&contours,sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
    CvPoint center;
    while(contours)
    {
        //obtain a sequence of points of contour, pointed by the variable 'contour'
        result = cvApproxPoly(contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.02, 0);
        //if there are 3  vertices  in the contour(It should be a triangle)
        if((result->total==4)&&(fabs(cvContourArea(result))>900) )
        {
            //iterating through each point
            CvPoint *pt0[4];
            for(int i=0;i<4;i++)
            {
            pt0[i] = (CvPoint*)cvGetSeqElem(result, i);
            }
             
            double  maxCos0=0;
            for( int j = 2; j < 5; j++ )
            {
             double cosine0 = fabs(angle(pt0[j%4], pt0[j-2], pt0[j-1]));
             if( cosine0 > maxCos0)
             {
                maxCos0 =cosine0;
             }
            }
            // if cosines of all angles are small
            // (all angles are ~90 degree) then write quandrange
            // vertices to resultant sequence
            if( maxCos0 < 0.15 )  
            {
            //caculating the x center of the rectangle
            center.x = (pt0[0]->x+pt0[1]->x+pt0[2]->x+pt0[3]->x)/4;
            center.y =  (pt0[0]->y+pt0[1]->y+pt0[2]->y+pt0[3]->y)/4;
            //drawing lines around the quadrilateral
            cvLine(image, *pt0[0], *pt0[1], cvScalar(0,0,255),4);
            cvLine(image, *pt0[1], *pt0[2], cvScalar(0,0,255),4);
            cvLine(image, *pt0[2], *pt0[3], cvScalar(20,0,255),4);
            cvLine(image, *pt0[3], *pt0[0], cvScalar(0,0,255),4);
            }
        }
        //iterating through each point
        contours = contours->h_next;     
    }
    return center;
}
 
static void StereoCalib(const char* imageList, int nx, int ny, int useUncalibrated)
{
int displayCorners = 0;
int showUndistorted = 1;
bool isVerticalStereo = false;//OpenCV có thể handle left -right
//or up-down camera arrangements
const int maxScale = 1;
const float squareSize = 2.5f; //Set this to your actual square size
FILE* f = fopen(imageList, "rt");
int i, j, lr, nframes, n = nx*ny, N = 0;
vector<string> imageNames[2];
vector<CvPoint3D32f> objectPoints;
vector<CvPoint2D32f> points[2];
vector<int> npoints;
vector<uchar> active[2];
vector<CvPoint2D32f> temp(n);
CvSize imageSize = {0,0};
// ARRAY và VECTOR STORAGEimageNames
double M1[3][3], M2[3][3], D1[5], D2[5];
double R[3][3], T[3], E[3][3], F[3][3];
CvMat _M1 = cvMat(3, 3, CV_64F, M1 );
CvMat _M2 = cvMat(3, 3, CV_64F, M2 );
CvMat _D1 = cvMat(1, 5, CV_64F, D1 );
CvMat _D2 = cvMat(1, 5, CV_64F, D2 );
CvMat _R = cvMat(3, 3, CV_64F, R );
CvMat _T = cvMat(3, 1, CV_64F, T );
CvMat _E = cvMat(3, 3, CV_64F, E );
CvMat _F = cvMat(3, 3, CV_64F, F );
if( displayCorners )
cvNamedWindow( "corners", 1 );
// READ IN THE LIST OF CHESSBOARDS:
if( !f )
{
fprintf(stderr, "can not open file %s\n", imageList );
return;
}
for(i=0;;i++)
{
char buf[1024];
int count = 0, result=0;
lr = i % 2;
vector<CvPoint2D32f>& pts = points[lr];
if( !fgets( buf, sizeof(buf)-3, f ))
break;
size_t len = strlen(buf);
while( len > 0 && isspace(buf[len-1]))
buf[--len] = '\0';
if( buf[0] == '#')
continue;
IplImage* img = cvLoadImage( buf, 0 );
if( !img )
break;
imageSize = cvGetSize(img);
imageNames[lr].push_back(buf);
//FIND CHESSBOARDS và CORNERS THEREIN:
for( int s = 1; s <= maxScale; s++ )
{
IplImage* timg = img;
if( s > 1 )
{
timg = cvCreateImage(cvSize(img->width*s,img->height*s),
img->depth , img->nChannels );
cvResize( img, timg, CV_INTER_CUBIC );
}
result = cvFindChessboardCorners( timg, cvSize(nx, ny),
&temp[0], &count,
CV_CALIB_CB_ADAPTIVE_THRESH |
CV_CALIB_CB_NORMALIZE_IMAGE);
if( timg != img )
cvReleaseImage( &timg );
if( result || s == maxScale )
for( j = 0; j < count; j++ )
{
temp[j].x /= s;
temp[j].y /= s;
}
if( result )
break;
}
if( displayCorners )
{
printf("%s\n", buf);
IplImage* cimg = cvCreateImage( imageSize, 8, 3 );
cvCvtColor( img, cimg, CV_GRAY2BGR );
cvDrawChessboardCorners( cimg, cvSize(nx, ny), &temp[0],
count, result );
cvShowImage( "corners", cimg );
cvReleaseImage( &cimg );
if( cvWaitKey(0) == 27 ) //Allow ESC to quit
exit(-1);
}
else
putchar('.');
N = pts.size();
pts.resize(N + n, cvPoint2D32f(0,0));
active[lr].push_back((uchar)result);
//assert( result != 0 );
if( result )
{
//Calibration sẽ suffer with out subpixel interpolation
cvFindCornerSubPix( img, &temp[0], count,
cvSize(11, 11), cvSize(-1,-1),
cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
30, 0.01) );
copy( temp.begin(), temp.end(), pts.begin() + N );
}
cvReleaseImage( &img );
}
fclose(f);
printf("\n");
// HARVEST CHESSBOARD 3D OBJECT POINT LIST:
nframes = active[0].size();//Number of good chessboads found
objectPoints.resize(nframes*n);
for( i = 0; i < ny; i++ )
for( j = 0; j < nx; j++ )
objectPoints[i*nx + j] =
cvPoint3D32f(i*squareSize, j*squareSize, 0);
for( i = 1; i < nframes; i++ )
copy( objectPoints.begin(), objectPoints.begin() + n,
objectPoints.begin() + i*n );
npoints.resize(nframes,n);
N = nframes*n;
CvMat _objectPoints = cvMat(1, N, CV_32FC3, &objectPoints[0] );
CvMat _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
CvMat _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0] );
cvSetIdentity(&_M1);
cvSetIdentity(&_M2);
cvZero(&_D1);
cvZero(&_D2);
// CALIBRATE THE STEREO CAMERAS
printf("Running stereo calibration ...");
fflush(stdout);
cvStereoCalibrate( &_objectPoints, &_imagePoints1,
&_imagePoints2, &_npoints,
&_M1, &_D1, &_M2, &_D2,
imageSize, &_R, &_T, &_E, &_F,
cvTermCriteria(CV_TERMCRIT_ITER+
CV_TERMCRIT_EPS, 100, 1e-5),
CV_CALIB_FIX_ASPECT_RATIO +
CV_CALIB_ZERO_TANGENT_DIST +
CV_CALIB_SAME_FOCAL_LENGTH);
printf(" done\n");
// CALIBRATION QUALITY CHECK
// vì the output fundamental matrix implicitly
// bao gồm tất cả the output thông tin,
// ta có thể check the chất lượng của calibration dùng the
// epipolar geometry constraint: m2^t*F*m1=0
vector<CvPoint3D32f> lines[2];
points[0].resize(N);
points[1].resize(N);
_imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
_imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
lines[0].resize(N);
lines[1].resize(N);
CvMat _L1 = cvMat(1, N, CV_32FC3, &lines[0][0]);
CvMat _L2 = cvMat(1, N, CV_32FC3, &lines[1][0]);
//Always work in undistorted space
cvUndistortPoints( &_imagePoints1, &_imagePoints1,
&_M1, &_D1, 0, &_M1 );
cvUndistortPoints( &_imagePoints2, &_imagePoints2,
&_M2, &_D2, 0, &_M2 );
cvComputeCorrespondEpilines( &_imagePoints1, 1, &_F, &_L1 );
cvComputeCorrespondEpilines( &_imagePoints2, 2, &_F, &_L2 );
double avgErr = 0;
for( i = 0; i < N; i++ )
{
double err = fabs(points[0][i].x*lines[1][i].x +
points[0][i].y*lines[1][i].y + lines[1][i].z)
+ fabs(points[1][i].x*lines[0][i].x +
points[1][i].y*lines[0][i].y + lines[0][i].z);
avgErr += err;
}
printf( "avg err = %g\n", avgErr/(nframes*n) );
//COMPUTE và DISPLAY RECTIFICATION
if( showUndistorted )
{
CvMat* mx1 = cvCreateMat( imageSize.height,
imageSize.width, CV_32F );
CvMat* my1 = cvCreateMat( imageSize.height,
imageSize.width, CV_32F );
CvMat* mx2 = cvCreateMat( imageSize.height,
imageSize.width, CV_32F );
CvMat* my2 = cvCreateMat( imageSize.height,
imageSize.width, CV_32F );
CvMat* img0r = cvCreateMat( imageSize.height,
imageSize.width, CV_8U );
CvMat* img1r = cvCreateMat( imageSize.height,
imageSize.width, CV_8U );
CvMat* disp = cvCreateMat( imageSize.height,
imageSize.width, CV_16S );
CvMat* vdisp = cvCreateMat( imageSize.height,
imageSize.width, CV_8U );
double R1[3][3], R2[3][3], P1[3][4], P2[3][4];
CvMat _R1 = cvMat(3, 3, CV_64F, R1);
CvMat _R2 = cvMat(3, 3, CV_64F, R2);
// IF BY CALIBRATED (BOUGUET'S METHOD)
if( useUncalibrated == 0 )
{
CvMat _P1 = cvMat(3, 4, CV_64F, P1);
CvMat _P2 = cvMat(3, 4, CV_64F, P2);
cvStereoRectify( &_M1, &_M2, &_D1, &_D2, imageSize,
&_R, &_T,
&_R1, &_R2, &_P1, &_P2, 0,
0/*CV_CALIB_ZERO_DISPARITY*/ );
isVerticalStereo = fabs(P2[1][3]) > fabs(P2[0][3]);
//Precompute maps for cvRemap()
cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_P1,mx1,my1);
cvInitUndistortRectifyMap(&_M2,&_D2,&_R2,&_P2,mx2,my2);
}
//OR ELSE HARTLEY'S METHOD
else if( useUncalibrated == 1 || useUncalibrated == 2 )
// dùng intrinsic parameters of each camera, but
// compute the rectification transformation directly
// from the fundamental matrix
{
double H1[3][3], H2[3][3], iM[3][3];
CvMat _H1 = cvMat(3, 3, CV_64F, H1);
CvMat _H2 = cvMat(3, 3, CV_64F, H2);
CvMat _iM = cvMat(3, 3, CV_64F, iM);
//Just to show bạn có thể have independently used F
if( useUncalibrated == 2 )
cvFindFundamentalMat( &_imagePoints1,
&_imagePoints2, &_F);
cvStereoRectifyUncalibrated( &_imagePoints1,
&_imagePoints2, &_F,
imageSize,
&_H1, &_H2, 3);
cvInvert(&_M1, &_iM);
cvMatMul(&_H1, &_M1, &_R1);
cvMatMul(&_iM, &_R1, &_R1);
cvInvert(&_M2, &_iM);
cvMatMul(&_H2, &_M2, &_R2);
cvMatMul(&_iM, &_R2, &_R2);
//Precompute map for cvRemap()
cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_M1,mx1,my1);
cvInitUndistortRectifyMap(&_M2,&_D1,&_R2,&_M2,mx2,my2);
}
else
assert(0);
//////Definition variables///////////////////////////////
IplImage*img0;
IplImage*img1;
IplImage* imgGrayScale0;
IplImage* imgGrayScale1;
double disparity=0,dist=0;
double ttx1=0,ttx0=0,tty0=0,tty1=0;
char chardist[10];
char chardisparity[10];
/////////Init Font to display test on images/////////////
CvFont font;
        double hScale=1.0;
        double vScale=1.0;
        int    lineWidth=2;
        cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,lineWidth);
/////////////////////////////////////////////////////////
while(true)
{
//Loading images from camera0 and camera1
    //camera0-->left
    img0 = cvQueryFrame(capture0);
    if(!img0) break;
    imgGrayScale0 = cvCreateImage(cvGetSize(img0), 8, 1);
    cvCvtColor(img0,imgGrayScale0,CV_BGR2GRAY);
    //camera1-->right
    img1 = cvQueryFrame(capture1);
    if(!img1) break;
    imgGrayScale1 = cvCreateImage(cvGetSize(img1), 8, 1);
    cvCvtColor(img1,imgGrayScale1,CV_BGR2GRAY);
//Rectify the Images
    cvRemap(imgGrayScale0,img0r,mx1,my1);
    cvRemap(imgGrayScale1,img1r,mx2,my2);
//Finding center of the rectangle on images and saving them in ttx, tty variables
    ttx0=RecCenter(img0r).x;
    ttx1=RecCenter(img1r).x;
    tty0=RecCenter(img0r).y;
    tty1=RecCenter(img1r).y;
    cout<<ttx0<<"  "<<ttx1<<"\n";
//caculating disparity and distance.Then, display them on images
    if ( (ttx0>0) && (ttx1>0))
        {   
        disparity=ttx0-ttx1+115;
        dist=(810*9)/disparity;
        _itoa(dist,chardist,10);
        _itoa(disparity,chardisparity,10);
        /////////Display text on image
        cvPutText (img1r,chardist,cvPoint(ttx1,tty1), &font, cvScalar(255,255,255));
        cvPutText (img1r,"cm",cvPoint(ttx1+40,tty1-20), &font, cvScalar(255,255,255));
        cvPutText (img0r,chardisparity,cvPoint(ttx0,tty0), &font, cvScalar(255,255,255));
        //////////// End display text
    }
    cvShowImage("Tracked Image0",img0r);
    cvShowImage("Tracked Image1",img1r);
     
    cvWaitKey(50);
}
//////////////////////////////
cvReleaseMat( &mx1 );
cvReleaseMat( &my1 );
cvReleaseMat( &mx2 );
cvReleaseMat( &my2 );
cvReleaseMat( &img0r );
cvReleaseMat( &img1r );
cvReleaseImage(&img0);
cvReleaseImage(&img1);
}
}
int main(void)
{
     
capture0 = cvCaptureFromCAM(0);
if(!capture0){
printf("Capture camera0 failure\n");
return -1;
}
capture1 = cvCaptureFromCAM(1);
if(!capture1){
printf("Capture camera1 failure\n");
return -1;
}
StereoCalib("list.txt", 9, 6, 1);
cvReleaseCapture(&capture0);
cvReleaseCapture(&capture1);
return 0;
}