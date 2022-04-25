#include <iostream>
#include <boost/iterator/iterator_concepts.hpp>

#include <pangolin/pangolin.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>


using namespace std;
using namespace pangolin;
using namespace cv;


void setImageData(unsigned char * imageArray, cv::Mat image) {
    int clos = image.cols;
    int rows = image.rows;
  
    int imageArray_count=0;

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < clos; j++) {
            imageArray[imageArray_count] = (unsigned char)image.at<Vec3b>(i,j)[0];
            imageArray_count++;
            imageArray[imageArray_count] = (unsigned char)image.at<Vec3b>(i,j)[1];
            imageArray_count++;
            imageArray[imageArray_count] = (unsigned char)image.at<Vec3b>(i,j)[2];
            imageArray_count++;
        }
    }
}

int main(int argc, char** argv) {
  
    int width =  image_initial.cols;
    int height = image_initial.rows;
  
    namedWindow("opencv", 1);
  
    pangolin::CreateWindowAndBind("Pangolin", width, height);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
    pangolin::View& d_image = pangolin::Display("image").SetBounds(1,0,1,0)
                                                        .SetLock(pangolin::LockRight, pangolin::LockBottom);

    unsigned char* imageArray = new unsigned char[3*width*height];
    pangolin::GlTexture imageTexture(width,height);
  
    while(!pangolin::ShouldQuit()) {

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        setImageData(imageArray, image_initial);
        imageTexture.Upload(imageArray,GL_RGB,GL_UNSIGNED_BYTE);
        d_image.Activate();
    
        glColor3f(1.0,1.0,1.0);
    
        imageTexture.RenderToViewportFlipY();

        pangolin::FinishFrame();
    
        imshow("opencv",image_initial);

        waitKey(1);
    }	  
    
    destroyWindow("opencv");
    delete[] imageArray;

    return 0;
}