// include the librealsense C++ header file
#include <librealsense2/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

float get_depth_scale(rs2::device dev);
void traverseDepth(const rs2::frame& frm, int w, int h, float depth_scale);
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);

int main()
{
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
    for(int i = 0; i < 30; i++)
    {
        //Wait for all configured streams to produce a frame
        frames = pipe.wait_for_frames();
    }

    frames = pipe.wait_for_frames();

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

    //If one of them is unavailable, continue iteration
    std::cout << "aligned_depth_frame" << aligned_depth_frame << std::endl;
    std::cout << "other_frame" << other_frame << std::endl;
    if (!aligned_depth_frame || !other_frame)
    {
        return 0;
    }

    // Creating OpenCV Matrix from a color image
    Mat color(Size(1280, 720), CV_8UC3, (void*)other_frame.get_data(), Mat::AUTO_STEP);
    Mat ir(Size(1280, 720), CV_8UC3, (void*)newFrame.get_data(), Mat::AUTO_STEP);
    // Mat depth(Size(1280, 720), CV_16F, (void*)aligned_depth_frame.get_data(), Mat::AUTO_STEP);

    // resize(color, color, Size(1920,1080), INTER_CUBIC);
    // resize(ir, ir, Size(1920,1080), INTER_CUBIC);
    imwrite("./aa2.png", color);
    imwrite("./bb2.png", ir);
    // Display in a GUI1
    // namedWindow("Display Image", WINDOW_AUTOSIZE );
    // imshow("Display Image", color);


    //Get each frame
    // rs2::frame ir_frame = frames.first(RS2_STREAM_INFRARED);
    // rs2::frame depth_frame = frames.get_depth_frame();

    // Wait for the next set of frames from the camera. Now that autoexposure, etc.
    // has settled, we will write these to disk
    // for (auto&& frame : frames)
    // {
    //     // We can only save video frames as pngs, so we skip the rest
    //     if (auto vf = frame.as<rs2::video_frame>())
    //     {
    //         auto stream = frame.get_profile().stream_type();
    //         std::cout << "inininin1" << stream << std::endl;
    //         // Use the colorizer to get an rgb image for the depth stream
    //         if (vf.is<rs2::depth_frame>()) 
    //         {
    //             std::cout << "inininin2" << std::endl;
    //             // traverseDepth(frame, vf.get_width(), vf.get_height(), depth_scale);
    //         }

    //     }
    // }
    // ben modify end

    // ben delete start
    // // Creating OpenCV matrix from IR image
    // Mat ir(Size(640, 480), CV_8UC1, (void*)ir_frame.get_data(), Mat::AUTO_STEP);
    // // Mat ir(Size(640, 480), CV_8UC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

    // // Apply Histogram Equalization
    // equalizeHist( ir, ir );
    // applyColorMap(ir, ir, COLORMAP_JET);

    // // Display the image in GUI
    // namedWindow("Display Image", WINDOW_AUTOSIZE );
    // imshow("Display Image", ir);
    // ben delete end

    waitKey(0);

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

void traverseDepth(const rs2::frame& frm, int w, int h, float depth_scale)
{
    const uint16_t* p_depth_frame = reinterpret_cast<const uint16_t*>(frm.get_data());
    for (int y = 0; y < h; y++)
    {
        auto depth_pixel_index = y * w;
        for (int x = 0; x < w; x++, ++depth_pixel_index)
        {
            // Get the depth value of the current pixel
            auto pixels_distance = depth_scale * p_depth_frame[depth_pixel_index];
            std::cout << "x:" << x << ",y" << y << ",pixels_distance: " << pixels_distance << std::endl;
        }
    }
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