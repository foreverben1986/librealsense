// include the librealsense C++ header file
#include <librealsense2/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>
#include <sstream>
#include <ctime>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

float get_depth_scale(rs2::device dev);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
int outputIntrinsics(float depth_scale, const struct rs2_intrinsics* intrin, const string& path);

int main(int argc, char *argv[])
{
    // get fps and output path
    if (argc < 3) {
        throw std::runtime_error("please input fps and output path"); 
    }
    int num = 0;
    if (argc >= 4) {
        num = std::atoi(argv[3]);
    }
    int fps = std::atoi(argv[1]);
    string path = argv[2];
    if (fps > 30) {
        throw std::runtime_error("please input fps cannot be larger than 30"); 
    }

    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);


    // ben add start
    //Instruct pipeline to start streaming with the requested configuration
    rs2::pipeline_profile profile = pipe.start(cfg);

    // Each depth camera might have different units for depth pixels, so we get it here
    // Using the pipeline's profile, we can retrieve the device that the pipeline uses
    float depth_scale = get_depth_scale(profile.get_device());
    std::cout << "depth_scale: " << depth_scale << std::endl;

    // ben add end

    // ben modify start
    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;
    for(long i = 1; i < 60; i++)
    {
        int interval = 30 / fps;
        //Wait for all configured streams to produce a frame
        frames = pipe.wait_for_frames();
        if (i%interval != 0) {
            continue;
        }
        //Pipeline could choose a device that does not have a color stream
        //If there is no color stream, choose to align depth to another stream
        rs2_stream align_to = find_stream_to_align(profile.get_streams());

        // Create a rs2::align object.
        // rs2::align allows us to perform alignment of depth frames to others frames
        //The "align_to" is the stream type to which we plan to align depth frames.
        rs2::align align(align_to);

        //Get processed aligned frame
        auto processed = align.process(frames);

        // Trying to get both other and aligned depth frames
        rs2::video_frame other_frame = processed.first(align_to);
        rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();
        rs2::frame newFrame = color_map.process(aligned_depth_frame);

        rs2_intrinsics intr = aligned_depth_frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics();

        //If one of them is unavailable, continue iteration
        // std::cout << "aligned_depth_frame" << aligned_depth_frame << std::endl;
        // std::cout << "other_frame" << other_frame << std::endl;
        if (!aligned_depth_frame || !other_frame)
        {
            return 0;
        }

        // Creating OpenCV Matrix from a color image
        Mat color(Size(1280, 720), CV_8UC3, (void*)other_frame.get_data(), Mat::AUTO_STEP);
        Mat ir(Size(1280, 720), CV_8UC3, (void*)newFrame.get_data(), Mat::AUTO_STEP);
        Mat depth(Size(1280, 720), CV_16U, (void*)aligned_depth_frame.get_data(), Mat::AUTO_STEP);

        // resize(color, color, Size(1920,1080), INTER_CUBIC);
        // resize(ir, ir, Size(1920,1080), INTER_CUBIC);
        stringstream colorPng;
        stringstream irPng;
        stringstream depthTxt;
        stringstream depthPng;
        time_t now = time(0);
        colorPng << path << "color" << "_" << i << now << ".png";
        irPng << path << "ir" << "_" << i << now << ".png";
        depthTxt << path << "depth" << "_" << i << now << ".json";
        depthPng << path << "depth" << "_" << i << now << ".png";
        int depthResult = outputIntrinsics(depth_scale, &intr, depthTxt.str());
        if (!depthResult) {
            std::cout << depthTxt.str() << " output failed" << std::endl;
            continue;
        }
        // cv::medianBlur(depth, depth, 5);
        // cv::medianBlur(ir, ir, 5);
        // cv::GaussianBlur(ir, ir, cv::Size(9,9), 1.0, 1.0);
        // cv::GaussianBlur(depth, depth, cv::Size(9, 9), 1.0, 1.0);
        imwrite(colorPng.str(), color);
        imwrite(irPng.str(), ir);
        imwrite(depthPng.str(), depth);

        if (--num == 0) {
            break;
        }

    }


    return 0;
}

float get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

int outputIntrinsics(float depth_scale, const struct rs2_intrinsics* intrin, const string& path) 
{
    ofstream depthf(path);
    if (!depthf) return 0;
    depthf << "{" << std::endl;
    depthf << "\"scale\":" << depth_scale << ",";
    depthf << "\"fx\":" << intrin->fx << ",";
    depthf << "\"fy\":" << intrin->fy << ",";
    depthf << "\"ppx\":" << intrin->ppx << ",";
    depthf << "\"ppy\":" << intrin->ppy << ",";
    depthf << "\"model\":\"" << intrin->model << "\",";
    depthf << "\"coeffs\": [" ;
    for (int i = 0; i < 5; i++) 
    {
        if (i == 4) {
            depthf << intrin->coeffs[i];
        } else {
            depthf << intrin->coeffs[i] <<",";
        }
    }
    depthf << "]" ;
    depthf << "}" << std::endl;
    return 1;
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    //Given a vector of streams, we try to find a depth stream and another stream to align depth with.
    //We prioritize color streams to make the view look better.
    //If color is not available, we take another stream that (other than depth)
    rs2_stream align_to = RS2_STREAM_ANY;
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)
    {
        rs2_stream profile_stream = sp.stream_type();
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found)         //Prefer color
                align_to = profile_stream;

            if (profile_stream == RS2_STREAM_COLOR)
            {
                color_stream_found = true;
            }
        }
        else
        {
            depth_stream_found = true;
        }
    }

    if(!depth_stream_found)
        throw std::runtime_error("No Depth stream available");

    if (align_to == RS2_STREAM_ANY)
        throw std::runtime_error("No stream found to align with Depth");

    return align_to;
}