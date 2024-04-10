#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <fstream>
// 结构体用于保存PGM图像的信息
struct PGMImage {
    int width{};
    int height{};
    std::vector<u_int16_t> data; // 存储图像像素数据
};

// 加载P2格式的PGM图像
PGMImage loadPGM(const char* filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file");
    }

    std::string magic;
    std::string line;
    int width, height, maxVal;

    // 读取PGM文件头信息
    file >> magic;
    if (magic != "P2") {
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

    for (int i = 0; i < width * height; ++i) {
        int pixelValue;
        file >> pixelValue;
        image.data[i] = static_cast<u_int16_t>(pixelValue);
    }

    return image;
}


int main()
{
    const char *xmap_filename = "../img/xmap.pgm";
    const char *ymap_filename = "../img/ymap.pgm";
    // 读取图像
    cv::Mat image = cv::imread("../img/insta360-still-001.png");

    // 检查图像是否成功加载
    if (image.empty())
    {
        std::cerr << "Failed to load image" << std::endl;
        return -1;
    }


    // 打印图像的宽度和高度
//    std::cout << "Image width: " << width << ", height: " << height << std::endl;

    cv::Mat dstImg = cv::Mat::zeros(image.size(), CV_8UC3);

    uint8_t *dst = dstImg.data;
    const uint8_t *src = image.data;
    // 获取图像的宽度和高度
    int width = image.cols;
    int height = image.rows;

    const int dlinesize = 5760;
    const int slinesize = 5760;
    const int xlinesize = 1920;
    const int ylinesize = 1920;

    PGMImage xin = loadPGM(xmap_filename);
    PGMImage yin = loadPGM(ymap_filename);
    const uint16_t *xmap = xin.data.data();
    const uint16_t *ymap = yin.data.data();
    const int step = 3;
    int c, x, y;
    auto start = std::chrono::high_resolution_clock::now();
    for (y = 0; y < 960; y++)
    {
        for (x = 0; x < 1920; x++)
        {
            for (c = 0; c < 3; c++)
            {
                if (ymap[x] < 960 && xmap[x] < 1920)
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
    auto end = std::chrono::high_resolution_clock::now();
    
    // 计算执行时间（以毫秒为单位）
    std::chrono::duration<double, std::milli> duration_ms = end - start;
    double cpu_time_used = duration_ms.count();

    std::cout << "Code execution time: " << cpu_time_used << " ms" << std::endl;

    cv::imshow("dst", dstImg);
    cv::waitKey(0);
    return 0;
}
