// cv_compat_411.cpp —— 头文件(4.10 接口) 与 库(4.11 接口) 的 cvtColor 签名适配层
//
// 背景：板子带的 OpenCV 头文件是 4.10 风格
//          void cvtColor(InputArray, OutputArray, int code, int dstCn = 0);
//       而 libopencv_imgproc.so 导出的却是 4.11 风格
//          void cvtColor(InputArray, OutputArray, int code, int dstCn, AlgorithmHint hint);
//       结果：用这套头文件编译，链接时会报
//          undefined reference to `cv::cvtColor(cv::_InputArray const&, cv::_OutputArray const&, int, int)'
//
// 本文件补上那个缺失的 4 参符号，内部转调库里真实存在的 5 参版本。
// 枚举名与枚举值必须与 OpenCV 4.11 完全一致，否则名字修饰对不上。

#include <opencv2/opencv.hpp>

#if !defined(CV_VERSION_MINOR) || CV_VERSION_MINOR < 11

namespace cv
{
// 与 4.11 的 cv::AlgorithmHint 保持同名同值（名字修饰只认名字和作用域）
enum class AlgorithmHint
{
    ALGO_HINT_DEFAULT = 0,
    ALGO_HINT_ACCURATE = 1,
    ALGO_HINT_APPROX = 2,
};

// 库里真实存在的 5 参版本
void cvtColor(const _InputArray& src, const _OutputArray& dst,
              int code, int dstCn, AlgorithmHint hint);

// 头文件里声明、但库里没有的 4 参版本：转发到 5 参
void cvtColor(const _InputArray& src, const _OutputArray& dst, int code, int dstCn)
{
    cvtColor(src, dst, code, dstCn, AlgorithmHint::ALGO_HINT_DEFAULT);
}

}  // namespace cv

#endif  // CV_VERSION_MINOR < 11
