#include <iostream>
#include <pangolin/pangolin.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <deque>
#include <mutex>
#include <unistd.h>
#include <condition_variable>

#define width 640 
#define height 480 
#define fps 30

unsigned int d435_exp  = 0;

static const std::string window_name = "KLTtracker";
std::deque<cv::Mat> right_imgs;
std::deque<cv::Mat> left_imgs;

std::mutex mut_;
std::condition_variable data_cond_;

rs2::frameset frames;
rs2::config cfg;
rs2::pipeline pipe_;

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

void Setup() {
    pangolin::CreateWindowAndBind(window_name, width * 2, height);

    glEnable(GL_DEPTH_TEST);

    pangolin::GetBoundWindow()->RemoveCurrent();
}

void Pangolin_Run() {
    pangolin::BindToContext(window_name);

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

    pangolin::GlTexture imageTexture1(width, height);
    pangolin::GlTexture imageTexture2(width, height);

    pangolin::CreatePanel("set").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(160));//创建

    pangolin::Var<unsigned int> d435_exposure("set. ", 1, 10000, 165000);
    pangolin::Var<std::function<void(void)>> exposure("set.Exposure", d435_stereo_exp);
    
    pangolin::Var<std::function<void(void)>> laser_off("set.Laser Off", d435_laser_off);
    pangolin::Var<std::function<void(void)>> laser_on("set.Laser ON", d435_laser_on);

    unsigned char* imageArray1 = new unsigned char[width * height];
    unsigned char* imageArray2 = new unsigned char[width * height];

    while(true) {

        std::unique_lock<std::mutex> lk(mut_);

        data_cond_.wait(lk, []{return !right_imgs.empty() && !left_imgs.empty();});

        cv::Mat image1_front = right_imgs.front();
        cv::Mat image2_front = left_imgs.front();

        right_imgs.pop_front();
        left_imgs.pop_front();

        lk.unlock();

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_image1.Activate();
        setImageData(imageArray1, image1_front);
        imageTexture1.Upload(imageArray1, GL_LUMINANCE, GL_UNSIGNED_BYTE);
        imageTexture1.RenderToViewportFlipY();

        d_image2.Activate();
        setImageData(imageArray2, image2_front);
        imageTexture2.Upload(imageArray2, GL_LUMINANCE, GL_UNSIGNED_BYTE);
        imageTexture2.RenderToViewportFlipY();

        d435_exp  = d435_exposure;
     
        pangolin::FinishFrame();
    }

    pangolin::GetBoundWindow()->RemoveCurrent();
}

void D435i_Run() {

    while(true) {

        frames = pipe_.wait_for_frames();

        rs2::video_frame ir_frame_left = frames.get_infrared_frame(1);
        rs2::video_frame ir_frame_right = frames.get_infrared_frame(2);

        cv::Mat pic_right(cv::Size(width, height), CV_8UC1, (void*)ir_frame_right.get_data());
        cv::Mat pic_left(cv::Size(width, height), CV_8UC1, (void*)ir_frame_left.get_data());

        std::lock_guard<std::mutex> lk(mut_);
        
        right_imgs.push_back(pic_right);
        left_imgs.push_back(pic_left);

        data_cond_.notify_one();
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
    
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
	cfg.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);

    pipe_.start(cfg);

    Setup();

    std::thread render_loop(Pangolin_Run);
    std::thread d435i_loop(D435i_Run);

    render_loop.join();
    d435i_loop.join();

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



