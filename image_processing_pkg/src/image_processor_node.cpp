extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavfilter/avfilter.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
#include <libavutil/opt.h>
}

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

class ImageProcessor
{
public:
    ImageProcessor(ros::NodeHandle &nh) : nh_(nh)
    {
        // load xmap and ymap files
        if (avformat_open_input(&xmap_format_ctx, xmap_filename, nullptr, nullptr) != 0)
        {
            // handle error
            std::cerr << "Error: Could not open xmap file" << std::endl;
        }

        if (avformat_open_input(&ymap_format_ctx, ymap_filename, nullptr, nullptr) != 0)
        {
            // handle error
            std::cerr << "Error: Could not open ymap file" << std::endl;
        }

        image_transport::ImageTransport it(nh_);
        image_sub_ = it.subscribe("/insta360/image_raw", 1, &ImageProcessor::imageCallback, this);
    }

    ~ImageProcessor()
    {
        // Release FFmpeg resources
        av_frame_free(&frame);
        avcodec_free_context(&codecContext);
        avformat_close_input(&formatContext);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // ffmpeg -i input.mp4 -i xmap.pgm -i ymap.pgm -q 0 -lavfi "format=pix_fmts=rgb24,remap" remapped.mp4
        // ffmpeg -r 30 -i ./demopics/insta360-still-001.png -i xmap.pgm -i ymap.pgm
        // -q 0 -lavfi "format=pix_fmts=rgb24,remap" -f image2  ./demopics/insta360_still-001_basic.jpg

        // Convert OpenCV Mat to AVFrame
        // AVFrame *avFrame = cvMatToAVFrame(cv_ptr->image);

        AVFormatContext *input_format_ctx = cvMatToAVFormatContext(cv_ptr->image);
        // Process AVFrame using FFmpeg
        // Example: You can encode the AVFrame or perform other operations
        // Create filter graph
        AVFilterGraph *filter_graph = avfilter_graph_alloc();
        if (!filter_graph)
        {
            // handle error
        }

        // Create input filter for input image 1504*3008
        AVFilterContext *input_filter_ctx = avfilter_graph_alloc_filter(filter_graph, avfilter_get_by_name("buffer"), "in");
        // Set input filter parameters (omitted for brevity)
        // av_opt_set_int_list(input_filter_ctx, "width", &width, 0, AV_OPT_SEARCH_CHILDREN);
        // av_opt_set_int_list(input_filter_ctx, "height", &height, 0, AV_OPT_SEARCH_CHILDREN);
        // av_opt_set_sample_fmt(input_filter_ctx, "pix_fmts", AV_PIX_FMT_RGB24, AV_OPT_SEARCH_CHILDREN);

        // Create input filter for xmap
        AVFilterContext *xmap_filter_ctx = avfilter_graph_alloc_filter(filter_graph, avfilter_get_by_name("buffer"), "xmap");
        // Set xmap filter parameters (omitted for brevity)

        // Create input filter for ymap
        AVFilterContext *ymap_filter_ctx = avfilter_graph_alloc_filter(filter_graph, avfilter_get_by_name("buffer"), "ymap");
        // Set ymap filter parameters (omitted for brevity)

        // Create output filter
        AVFilterContext *output_filter_ctx = avfilter_graph_alloc_filter(filter_graph, avfilter_get_by_name("buffersink"), "out");
        // Set output filter parameters (omitted for brevity)

        // Add filters to the filter graph
        // avfilter_link(input_filter_ctx, 0, xmap_filter_ctx, 0);
        // avfilter_link(xmap_filter_ctx, 0, ymap_filter_ctx, 0);
        // avfilter_link(ymap_filter_ctx, 0, output_filter_ctx, 0);

        // Configure filter graph
        AVFilterInOut *outputs = avfilter_inout_alloc();
        AVFilterInOut *inputs = avfilter_inout_alloc();
        outputs->name = av_strdup("in");
        outputs->filter_ctx = input_filter_ctx;
        outputs->pad_idx = 0;
        outputs->next = nullptr;
        inputs->name = av_strdup("out");
        inputs->filter_ctx = output_filter_ctx;
        inputs->pad_idx = 0;
        inputs->next = nullptr;

        if (avfilter_graph_parse_ptr(filter_graph, "format=pix_fmts=rgb24,remap", &inputs, &outputs, nullptr) < 0)
        {
            // handle error
        }

        if (avfilter_graph_config(filter_graph, nullptr) < 0)
        {
            // handle error
        }

        AVPacket packet;
        int ret;
        while (av_read_frame(input_format_ctx, &packet) >= 0)
        {
            if (packet.stream_index == 0)
            { // Assuming video stream index is 0
                // Decode the video packet
                AVFrame *frame = av_frame_alloc();
                if (!frame)
                {
                    // handle error
                }

                ret = avcodec_send_packet(input_filter_ctx->filter->inputs[0], &packet);
                if (ret < 0 && ret != AVERROR(EAGAIN) && ret != AVERROR_EOF)
                {
                    // handle error
                }

                while (ret >= 0)
                {
                    ret = avcodec_receive_frame(input_filter_ctx->filter->inputs[0], frame);
                    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                    {
                        break;
                    }
                    else if (ret < 0)
                    {
                        // handle error
                    }

                    // Convert AVFrame to cv::Mat
                    cv::Mat img = avframe_to_cvmat(frame);
                    cv::imshow("result", img);
                    cv::waitKey(0)

                    // Process the image using OpenCV functions (e.g., cv::imshow)
                    av_frame_unref(frame);

                }

                av_frame_free(&frame);
            }

            av_packet_unref(&packet);
        }

        // Cleanup
        avfilter_inout_free(&inputs);
        avfilter_inout_free(&outputs);
        avfilter_graph_free(&filter_graph);
        avformat_close_input(&input_format_ctx);
        avformat_close_input(&xmap_format_ctx);
        avformat_close_input(&ymap_format_ctx);

        // Release AVFrame
        av_frame_free(&avFrame);
    }

private:
    ros::NodeHandle nh_;
    image_transport::Subscriber image_sub_;

    AVFrame *frame = nullptr;
    AVCodecContext *codecContext = nullptr;
    AVFormatContext *formatContext = nullptr;

    const char *xmap_filename = "/data/insta360/Insta360-Air-remap/Insta360-Air-remap/xmap.pgm";
    const char *ymap_filename = "/data/insta360/Insta360-Air-remap/Insta360-Air-remap/ymap.pgm";

    // Open xmap file
    AVFormatContext *xmap_format_ctx = nullptr;
    // Open ymap file
    AVFormatContext *ymap_format_ctx = nullptr;

    AVFrame *cvMatToAVFrame(const cv::Mat &cvImage)
    {
        AVFrame *avFrame = av_frame_alloc();
        if (!avFrame)
        {
            ROS_ERROR("Failed to allocate AVFrame");
            return nullptr;
        }

        avFrame->format = AV_PIX_FMT_BGR24;
        avFrame->width = cvImage.cols;
        avFrame->height = cvImage.rows;

        int ret = av_frame_get_buffer(avFrame, 0);
        if (ret < 0)
        {
            ROS_ERROR("Failed to allocate AVFrame data");
            av_frame_free(&avFrame);
            return nullptr;
        }

        cv::Mat dst(cvImage.size(), CV_8UC3, avFrame->data[0], avFrame->linesize[0]);
        cvImage.copyTo(dst);

        return avFrame;
    }

    AVFormatContext *cvMatToAVFormatContext(const cv::Mat &cvImage)
    {
        AVFormatContext *input_format_ctx = nullptr;
        // Convert OpenCV image to AVFrame
        AVFrame *frame = av_frame_alloc();
        if (!frame)
        {
            ROS_ERROR("Failed to allocate AVFrame");
            return;
        }

        frame->format = AV_PIX_FMT_BGR24;
        frame->width = image.cols;
        frame->height = image.rows;

        int ret = av_frame_get_buffer(frame, 32);
        if (ret < 0)
        {
            ROS_ERROR("Failed to allocate frame data");
            av_frame_free(&frame);
            return;
        }

        // Copy image data to AVFrame
        memcpy(frame->data[0], image.data, image.total() * image.elemSize());

        // Open input format context if not opened yet
        if (!input_format_ctx)
        {
            avformat_alloc_output_context2(&input_format_ctx, nullptr, nullptr, nullptr);
            if (!input_format_ctx)
            {
                ROS_ERROR("Failed to allocate output context");
                av_frame_free(&frame);
                return;
            }
        }

        // Add frame to input format context
        AVStream *stream = avformat_new_stream(input_format_ctx, nullptr)

            if (!stream)
        {
            ROS_ERROR("Failed to create new stream");
            av_frame_free(&frame);
            return;
        }

        // Initialize codec parameters
        AVCodecParameters *codec_params = stream->codecpar;
        codec_params->codec_type = AVMEDIA_TYPE_VIDEO;
        codec_params->codec_id = AV_CODEC_ID_RAWVIDEO;
        codec_params->format = AV_PIX_FMT_BGR24;
        codec_params->width = frame->width;
        codec_params->height = frame->height;

        ret = avcodec_parameters_to_context(stream->codec, codec_params);
        if (ret < 0)
        {
            ROS_ERROR("Failed to initialize codec parameters");
            av_frame_free(&frame);
            return;
        }

        // Write the frame to the output context
        ret = av_write_frame(input_format_ctx, frame);
        if (ret < 0)
        {
            ROS_ERROR("Error writing frame to output context");
            av_frame_free(&frame);
            return;
        }
        return input_format_ctx;
    }
};

int main(int argc, char **argv)
{
    av_register_all();
    avcodec_register_all();
    avformat_network_init();

    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;

    ImageProcessor processor(nh);

    ros::spin();
    return 0;
}
