#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <fstream>
struct PGMImage
{
    int width{};
    int height{};
    std::vector<u_int16_t> data; // 存储图像像素数据
};

class ImageProcessor
{
public:
    ImageProcessor(ros::NodeHandle &nh) : nh_(nh)
    {
        image_transport::ImageTransport it(nh_);
        image_sub_ = it.subscribe("/insta360/image_raw", 1, &ImageProcessor::imageCallback, this);
        image_pub = it.advertise("/insta360/image_get", 1);
        xmap_in = loadPGM(xmap_filename);
        ymap_in = loadPGM(ymap_filename);
        _init_dstImg = false;
    }

    ~ImageProcessor()
    {
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
        auto start = std::chrono::high_resolution_clock::now();
        cv::Mat srcImg = cv_ptr->image;
        cv::resize(srcImg, srcImg, cv::Size(1024, 512));
        cv::Mat leftImg = srcImg(cv::Rect(0, 0, srcImg.cols / 2, srcImg.rows));
        cv::Mat rightImg = srcImg(cv::Rect(srcImg.cols / 2, 0, srcImg.cols / 2, srcImg.rows));

        cv::rotate(leftImg, leftImg, cv::ROTATE_90_CLOCKWISE);
        cv::rotate(rightImg, rightImg, cv::ROTATE_90_COUNTERCLOCKWISE);
        
        
        if (_init_dstImg == false)
        {
            dstImg = cv::Mat::zeros(srcImg.size(), srcImg.type());
            tmpImg = cv::Mat::zeros(srcImg.size(), srcImg.type());
            _init_dstImg = true;
            // 获取图像的宽度和高度
            width = srcImg.cols;
            height = srcImg.rows;

            dlinesize = width * 3;
            slinesize = width * 3;
            xlinesize = width;
            ylinesize = width;
            step = 3;
            _init_dstImg = true;
        }
        std::cout << "srcImg size: " << srcImg.size() << std::endl
                  << "start processing..." << std::endl;

        
        remap(srcImg, dstImg);
        cv::flip(dstImg, dstImg, 1);
        cv::flip(dstImg, dstImg, 0);
        auto end = std::chrono::high_resolution_clock::now();

        // 计算执行时间（以毫秒为单位）
        std::chrono::duration<double, std::milli> duration_ms = end - start;
        double cpu_time_used = duration_ms.count();

        std::cout << "Code execution time: " << cpu_time_used << " ms" << std::endl;
        cv::imshow("dst", dstImg);
        cv::waitKey(1);
        cv_image.image = dstImg;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        sensor_msgs::ImagePtr image_msg = cv_image.toImageMsg();
        image_msg->header = msg->header;
        image_pub.publish(image_msg);
    }

private:
    ros::NodeHandle nh_;
    image_transport::Subscriber image_sub_;
    // Publish dstImg on topic /insta360/image_get
    image_transport::Publisher image_pub;
    cv_bridge::CvImage cv_image;

    const char *xmap_filename = "/data/catkin_ws/src/image_processing_pkg/img/xmapfinal.pgm";
    const char *ymap_filename = "/data/catkin_ws/src/image_processing_pkg/img/ymapfinal.pgm";

   

    cv::Mat dstImg, tmpImg;
    int width, height, dlinesize, slinesize, xlinesize, ylinesize, step;
    bool _init_dstImg;
    PGMImage xmap_in, ymap_in;

    void remap(cv::Mat srcImg, cv::Mat &dstImg)
    {
        uint16_t *xmap;
        uint16_t *ymap;
        // 0:rotate,1:map

            xmap = xmap_in.data.data();
            ymap = ymap_in.data.data();

        uint8_t *dst = dstImg.data;
        uint8_t *src = srcImg.data;
        int c, x, y;
        for (y = 0; y < height; y++)
        {
            for (x = 0; x < width; x++)
            {
                for (c = 0; c < step; c++)
                {
                    if (ymap[x] < height && xmap[x] < width)
                    {
                        dst[x * step + c] = src[ymap[x] * slinesize + xmap[x] * step + c];
                    }
                    else
                    {
                        dst[x * step + c] = 0;
                    }
                }
            }
            dst += dlinesize;
            xmap += xlinesize;
            ymap += ylinesize;
        }
    };

    PGMImage loadPGM(const char *filename)
    {
        std::ifstream file(filename);
        if (!file.is_open())
        {
            throw std::runtime_error("Failed to open file");
        }

        std::string magic;
        std::string line;
        int width, height, maxVal;

        // 读取PGM文件头信息
        file >> magic;
        if (magic != "P2")
        {
            throw std::runtime_error("Invalid PGM file format");
        }

        // 跳过注释行
        std::getline(file, line); // Skip first line
        std::getline(file, line); // Skip comment line

        // 读取图像宽度和高度
        file >> width >> height;

        // 读取最大像素值
        file >> maxVal;

        // 读取像素数据
        PGMImage image;
        image.width = width;
        image.height = height;
        image.data.resize(width * height);

        for (int i = 0; i < width * height; ++i)
        {
            int pixelValue;
            file >> pixelValue;
            image.data[i] = static_cast<u_int16_t>(pixelValue);
        }

        return image;
    }
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;

    ImageProcessor processor(nh);

    ros::spin();
    return 0;
}
