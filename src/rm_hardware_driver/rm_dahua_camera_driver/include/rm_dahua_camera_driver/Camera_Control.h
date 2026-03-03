#ifndef RM_DAHUA_CAMERA_DRIVER_CAMERA_CONTROL_H_
#define RM_DAHUA_CAMERA_DRIVER_CAMERA_CONTROL_H_

#include <math.h>
#include "GenICam/System.h"
#include "GenICam/PixelType.h"
#include "Media/VideoRender.h"
#include "Media/ImageConvert.h"
#include <iostream>
#include "GenICam/Camera.h"
#include "GenICam/GigE/GigECamera.h"
#include "GenICam/GigE/GigEInterface.h"
#include "Infra/PrintLog.h"
#include "Memory/SharedPtr.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace Dahua::GenICam;
using namespace Dahua::Infra;
using namespace Dahua::Memory;
using namespace std;
using namespace cv;

namespace pka::dahua_camera
{
    class Camera_Control
    {
    public:
        Camera_Control();
        ~Camera_Control();

        // 枚举触发方式
        enum ETrigType
        {
            trigContinous = 0, // 连续拉流
            trigSoftware = 1,  // 软件触发
            trigLine = 2,      // 外部触发
        };
        /*==================相机搜索连接和断开操作==================*/
        bool videoCheck();        // 搜索相机
        bool Connect_Camera();    // 连接相机
        bool Disconnect_Camera(); // 断开相机连接

        /*==================相机参数设置==========================*/
        void CameraChangeTrig(ETrigType trigType = trigSoftware); // 设置触发模式，一般为软触发
        void ExecuteSoftTrig();                                   // 执行一次软触发

        bool SetExposeTime(double exp);                                       // 设置曝光
        bool SetAdjustPlus(double adj);                                       // 设置Gain增益
        bool setBrightness(int brightness);                                   // 调节亮度
        bool setROI(int64_t nX, int64_t nY, int64_t nWidth, int64_t nHeight); // 设置ROI
        bool autoBalance(int enableAutoWhiteBalance);                         // 是否开启自动白平衡
        bool setFrameRate(double rate = 210);                                 // 设置帧率

        /*==================相机拉流==========================*/
        bool videoStart();    // 创建流对象
        bool startGrabbing(); // 开始拉流
        bool stopGrabbing();  // 断开拉流

        /*==================相机取帧==========================*/
        bool getFrame(Mat &img); // 获取一帧图片					                //转换为opencv可以识别的格式


        bool loadSetting(int mode);

    private:
        TVector<ICameraPtr> m_vCameraPtrList; // 相机列表
        IStreamSourcePtr m_pStreamSource;     // 流文件
        ICameraPtr m_pCamera;                 // 相机对象
    };

} // namespace pka::dahua_camera
#endif // RM_DAHUA_CAMERA_DRIVER_CAMERA_CONTROL_H_
