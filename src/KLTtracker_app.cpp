#include <iostream>
#include <pangolin/pangolin.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace pangolin;

#define width 640 
#define height 480 
#define fps 30

unsigned int d435_exp  = 0;
unsigned int d435_gain = 0;

void setImageData(unsigned char * imageArray, cv::Mat image) {
    int clos = image.cols;
    int rows = image.rows;
  
    int imageArray_count=0;

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < clos; j++) {
            imageArray[imageArray_count] = (unsigned char)image.at<uchar>(i,j);
            imageArray_count++;
        }
    }
}

void d435_laser_off(void) {
    rs2::context ctx;
	auto list = ctx.query_devices(); 
    rs2::device dev = list.front();
    auto depth_sensor = dev.first<rs2::depth_sensor>();
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
}

void d435_laser_on(void) {
    rs2::context ctx;
	auto list = ctx.query_devices(); 
    rs2::device dev = list.front();
    auto depth_sensor = dev.first<rs2::depth_sensor>();
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f);
}

void d435_stereo_exp(void) {
    rs2::context ctx;
	auto list = ctx.query_devices(); 
    rs2::device dev = list.front();
    auto depth_stereo_sensor = dev.first<rs2::depth_stereo_sensor>();
    if (depth_stereo_sensor.supports(RS2_OPTION_EXPOSURE)) {
        depth_stereo_sensor.set_option(RS2_OPTION_EXPOSURE, d435_exp);
    }
}

void d435_stereo_gain(void) {
    rs2::context ctx;
	auto list = ctx.query_devices(); 
    rs2::device dev = list.front();
    auto depth_stereo_sensor = dev.first<rs2::depth_stereo_sensor>();
    if (depth_stereo_sensor.supports(RS2_OPTION_GAIN)) {
        depth_stereo_sensor.set_option(RS2_OPTION_GAIN, d435_gain);
    }
}

int main(int argc, char** argv) try {

	rs2::context ctx;
	auto list = ctx.query_devices(); 
	if (list.size() == 0) 
		throw std::runtime_error("No device detected. Is it plugged in?");

    rs2::device dev = list.front();
    auto depth_sensor = dev.first<rs2::depth_sensor>();
    depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);

    rs2::frameset frames;
    rs2::pipeline pipe;
	rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
	cfg.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);

    pipe.start(cfg);
    pangolin::CreateWindowAndBind("Pangolin", width * 2, height);
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  
    pangolin::View& d_image1 = pangolin::Display("image1").SetBounds(1,0,1,0)
                                                         .SetLock(pangolin::LockRight, pangolin::LockBottom);
    pangolin::View& d_image2 = pangolin::Display("image2").SetBounds(1,0,1,0)
                                                         .SetLock(pangolin::LockRight, pangolin::LockBottom);
    pangolin::Display("multi").SetBounds(0, 1, 0, 1)
                              .SetLayout(pangolin::LayoutEqualHorizontal)
                              .AddDisplay(d_image1)
                              .AddDisplay(d_image2); 

    unsigned char* imageArray1 = new unsigned char[width * height];
    unsigned char* imageArray2 = new unsigned char[width * height];

    pangolin::GlTexture imageTexture1(width, height);
    pangolin::GlTexture imageTexture2(width, height);

    pangolin::CreatePanel("set").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(160));//创建

    pangolin::Var<unsigned int> d435_exposure("set. ", 1, 10000, 165000);
    pangolin::Var<std::function<void(void)>> exposure("set.Exposure", d435_stereo_exp);

    pangolin::Var<unsigned int> d435_gain("set.  ", 16, 16, 248);
    pangolin::Var<std::function<void(void)>> gain("set.Gain", d435_stereo_gain);
    
    pangolin::Var<std::function<void(void)>> laser_off("set.Laser Off", d435_laser_off);
    pangolin::Var<std::function<void(void)>> laser_on("set.Laser ON", d435_laser_on);

    while(1) {
        frames = pipe.wait_for_frames();

        rs2::video_frame ir_frame_left = frames.get_infrared_frame(1);
        rs2::video_frame ir_frame_right = frames.get_infrared_frame(2);

        Mat pic_right(Size(width,height), CV_8UC1, (void*)ir_frame_right.get_data());
        Mat pic_left(Size(width,height), CV_8UC1, (void*)ir_frame_left.get_data());

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        Mat image_initial = pic_right;

        d_image1.Activate();
        setImageData(imageArray1, image_initial);
        imageTexture1.Upload(imageArray1, GL_LUMINANCE, GL_UNSIGNED_BYTE);
        imageTexture1.RenderToViewportFlipY();
        
        image_initial = pic_left;

        d_image2.Activate();
        setImageData(imageArray2, image_initial);
        imageTexture2.Upload(imageArray2, GL_LUMINANCE, GL_UNSIGNED_BYTE);
        imageTexture2.RenderToViewportFlipY();

        d435_exp  = d435_exposure;
        d435_gain = d435_gain;
        
        pangolin::FinishFrame();
    }

    return 0;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}



