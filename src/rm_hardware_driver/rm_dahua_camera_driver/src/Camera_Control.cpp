#include"rm_dahua_camera_driver/Camera_Control.h"
using namespace Dahua::GenICam;
using namespace Dahua::Infra;
using namespace Dahua::Memory;

namespace pka::dahua_camera{
Camera_Control::Camera_Control(){}
// 搜索是否有相机连接
Camera_Control::~Camera_Control()
{
    // 断开相机连接
    if(m_pCamera != nullptr)
    {
        if(m_pCamera->isConnected())
        {
            m_pCamera->disConnect();
        }
    }
}
bool Camera_Control::videoCheck()
{
    CSystem &systemObj = CSystem::getInstance();

    bool bRet = systemObj.discovery(m_vCameraPtrList);
    if( bRet == false )
    {
        std::cout<<"discovery fail!"<<std::endl;
        return false;
    }
    // 如果找到相机则打印相机基本信息（key, 制造商信息, 型号, 序列号）
    for (std::size_t i = 0; i < m_vCameraPtrList.size(); i++)
    {
        ICameraPtr cameraSptr;
        cameraSptr = m_vCameraPtrList[i];

        std::cout << "Camera[" << i << "] Info :" << std::endl;
        std::cout << "    key           = [" << cameraSptr->getKey() << "]" << std::endl;
        std::cout << "    vendor name   = [" << cameraSptr->getVendorName() << "]" << std::endl;
        std::cout << "    model         = [" << cameraSptr->getModelName() << "]" << std::endl;
        std::cout << "    serial number = [" << cameraSptr->getSerialNumber() << "]" << std::endl << std::endl;
    }
    if (m_vCameraPtrList.size() < 1)
    {
        std::cout<<"No camera,Device Disconnected!"<<std::endl;
        return false;
    }
    else
    {
        //默认设置列表中的第一个相机为当前相机，其他操作比如打开、关闭、修改曝光都是针对这个相机。
        m_pCamera = m_vCameraPtrList[0];
    }
    return true;
}

// 连接相机
bool Camera_Control::Connect_Camera()
{
    if(m_pCamera == nullptr)
    {
        // 当前没有检测到相机
        std::cout<<"No camera is currently detected!"<<std::endl;
        return false;
    }
    // 先检查相机是否连接成功
    if(m_pCamera->isConnected())
    {
        // 如果已连接先断开再连接
        if(m_pCamera->disConnect())
        {
            if(m_pCamera->connect())
            {
                std::cout<<"Successfully connected the camera!"<<std::endl;
                return true;
            }  
        }
    }
    else
    {
        if(m_pCamera->connect()) return true;
    }
    return false;

}

// 断开相机连接
bool Camera_Control::Disconnect_Camera()
{
    if(m_pCamera != nullptr)
    {
        if (m_pCamera->isConnected())
        {
            if(m_pCamera->disConnect())
            {
                std::cout<<"Successfully disconnected the camera!"<<std::endl;
                return true;
            }
        }
    }
    if(m_pCamera == nullptr)
    {
        std::cout<<"Is Already Successful disconnected the camera!"<<std::endl;
        return true;
    }
    return false;
}


/*================ setting camera =================*/
// 更改相机触发模式
void Camera_Control::CameraChangeTrig(ETrigType trigType)
{
    if(m_pCamera == nullptr)
    {
        std::cout<<"Failed to change the camera trigger mode, and the camera is not currently connected!"<<std::endl;
        return;
    }
    // 连续拉流
    if(trigType == trigContinous)
    {
        //设置触发模式
        CEnumNode nodeTriggerMode(m_pCamera, "TriggerMode");
        if(nodeTriggerMode.isValid() == false)
        {
            std::cout<<"get TriggerMode node fail."<<std::endl;
            return;
        }
        if(nodeTriggerMode.setValueBySymbol("Off") == false)
        {
            std::cout<<"set TriggerMode value = Off fail."<<std::endl;
            return;
        }
    }
    // 软件触发
    else if (trigType == trigSoftware)
    {
        //设置触发源为软触发
        CEnumNode nodeTriggerSource(m_pCamera, "TriggerSource");
        if (nodeTriggerSource.isValid() == false )
        {
            std::cout<<"get TriggerSource node fail."<<std::endl;
            return;
        }
        if (nodeTriggerSource.setValueBySymbol("Software") == false )
        {
            std::cout<<"set TriggerSource value = Software fail."<<std::endl;
            return;
        }
        //设置触发器
        CEnumNode nodeTriggerSelector(m_pCamera, "TriggerSelector");
        if (nodeTriggerSelector.isValid() == false )
        {
            std::cout<<"get TriggerSelector node fail."<<std::endl;
            return;
        }
        if (nodeTriggerSelector.setValueBySymbol("FrameStart") == false)
        {
            std::cout<<"set TriggerSelector value = FrameStart fail."<<std::endl;
            return;
        }

        //设置触发模式
        CEnumNode nodeTriggerMode(m_pCamera, "TriggerMode");
        if (nodeTriggerMode.isValid() == false)
        {
            std::cout<<"get TriggerMode node fail."<<std::endl;
            return;
        }
        if (nodeTriggerMode.setValueBySymbol("On") == false)
        {
            std::cout<<"set TriggerMode value = On fail."<<std::endl;
            return;
        }
    }
    // 外部触发
    else if (trigLine == trigType)
    {
        //设置触发源为Line1触发
        CEnumNode nodeTriggerSource(m_pCamera, "TriggerSource");
        if (nodeTriggerSource.isValid() == false )
        {
            std::cout<<"get TriggerSource node fail."<<std::endl;
            return;
        }
        if (nodeTriggerSource.setValueBySymbol("Line1") == false )
        {
            std::cout<<"set TriggerSource value = Line1 fail."<<std::endl;
            return;
        }

        //设置触发器
        CEnumNode nodeTriggerSelector(m_pCamera, "TriggerSelector");
        if (nodeTriggerSelector.isValid() == false )
        {
            std::cout<<"get TriggerSelector node fail."<<std::endl;
            return;
        }
        if (nodeTriggerSelector.setValueBySymbol("FrameStart") == false)
        {
            std::cout<<"set TriggerSelector value = FrameStart fail."<<std::endl;
            return;
        }

        //设置触发模式
        CEnumNode nodeTriggerMode(m_pCamera, "TriggerMode");
        if (nodeTriggerMode.isValid() == false)
        {
            std::cout<<"get TriggerMode node fail."<<std::endl;
            return;
        }
        if (nodeTriggerMode.setValueBySymbol("On") == false)
        {
            std::cout<<"set TriggerMode value = On fail."<<std::endl;
            return;
        }

        // 设置外触发为上升沿（下降沿为FallingEdge）
        CEnumNode nodeTriggerActivation(m_pCamera, "TriggerActivation");
        if (nodeTriggerActivation.isValid() == false )
        {
            std::cout<<"get TriggerActivation node fail."<<std::endl;
            return;
        }
        if (nodeTriggerActivation.setValueBySymbol("RisingEdge") == false )
        {
            std::cout<<"set TriggerActivation value = RisingEdge fail."<<std::endl;
            return;
        }
    }
}

// 执行一次软触发
void Camera_Control::ExecuteSoftTrig()
{
    if(m_pCamera == nullptr)
    {
        std::cout<<"No camera or camera is not connected."<<std::endl;
        return;
    }
    CCmdNode nodeTriggerSoftware(m_pCamera, "TriggerSoftware");
    if (nodeTriggerSoftware.isValid() == false )
    {
        std::cout<<"get TriggerSoftware node fail."<<std::endl;
        return;
    }
    if (nodeTriggerSoftware.execute() == false)
    {
        std::cout<<"set TriggerSoftware fail."<<std::endl;
        return;
    }
}

// 设置曝光时间
bool Camera_Control::SetExposeTime(double exp)
{
    IAcquisitionControlPtr sptrAcquisitionControl = CSystem::getInstance().createAcquisitionControl(m_pCamera);
    if (sptrAcquisitionControl == nullptr)
    {
        std::cout<<"Create a IAcquisitionControlPtr failed!"<<std::endl;
        return false;
    }
    // 获取自动曝光模式节点
    CEnumNode eNode = sptrAcquisitionControl->exposureAuto();
    // 用于存储自动曝光模式节点的值
    uint64 getValue;
    // 获取节点值
    if (!eNode.getValue(getValue))
    {
        std::cout<<"Get value of type is failed!"<<std::endl;
        return false;
    }
    //如果开启了自动曝光模式，则关闭
    if (getValue)
    {
        if (!eNode.setValueBySymbol("Off"))
        {
            std::cout<<"Close autoExposure failed!"<<std::endl;
            return false;
        }
    }
    CDoubleNode dNode = sptrAcquisitionControl->exposureTime();
    if (dNode.setValue(exp))
    {
        std::cout<<"Set exposureTime Successfully!"<<std::endl;
        return true;
    }
    else{
        std::cout<<"Set exposureTime failed!"<<std::endl;
        return false;
    }
}

// 设置Gain增益
bool Camera_Control::SetAdjustPlus(double adj)
{
    if(m_pCamera == nullptr)
    {
        std::cout<<"Set GainRaw fail. No camera or camera is not connected."<<std::endl;
        return false;
    }
    CDoubleNode nodeGainRaw(m_pCamera, "GainRaw");
    if (!nodeGainRaw.isValid())
    {
        std::cout<<"Get GainRaw node fail."<<std::endl;
        return false;
    }

    if (!nodeGainRaw.isAvailable())
    {
        std::cout<<"GainRaw is not available."<<std::endl;
        return false;
    }

    if (!nodeGainRaw.setValue(adj))
    {
        std::cout << "set GainRaw value = " << adj << " fail!" << std::endl;
        return false;
    }
     std::cout << "set GainRaw value = " << adj << " Successfully!" << std::endl;
    return true;
}

//调节亮度
bool Camera_Control::setBrightness(int brightness)
{
    if(brightness<0||brightness>100)
    {
        std::cout<<"Invalid value,failed to modify camera brightness!"<<std::endl;
        return false;
    }
    Dahua::GenICam::CIntNode node_Brightness(m_pCamera,"Brightness");//亮度
    int64_t val;
    node_Brightness.getValue(val);
    if(int(val)!=brightness)//如果相同就不需要修改
    {
        if(node_Brightness.setValue(brightness))
        {
            std::cout<<"The camera brightness is changed successfully."<<std::endl;
            return true;
        }
        else
        {
            std::cout<<"Failed to modify camera brightness!"<<std::endl;
            return false;
        }
    }
    std::cout<<"Attribute values are the same and do not need to be modified."<<std::endl;
    return true;
}

// 设置ROI
bool Camera_Control::setROI(int64_t nX, int64_t nY, int64_t nWidth, int64_t nHeight)
{
    CIntNode nodeWidth(m_pCamera, "Width");
    CIntNode nodeHeight(m_pCamera, "Height");
    CIntNode OffsetX(m_pCamera, "OffsetX");
    CIntNode OffsetY(m_pCamera, "OffsetY");
    if (nodeWidth.setValue(nWidth)&&
        nodeHeight.setValue(nHeight)&&
        OffsetX.setValue(nX)&&
        OffsetY.setValue(nY))
    {
        std::cout<<"Set ROI successfully!"<<std::endl;
        return true;
    }
    else{
        return false;
    }
}

// 是否开启自动白平衡
bool Camera_Control::autoBalance(int enableAutoWhiteBalance)
{
    IAnalogControlPtr sptrAnalogControl = CSystem::getInstance().createAnalogControl(m_pCamera);
    if (sptrAnalogControl == nullptr)
    {
        return false;
    }
    // 获取自动白平衡控制节点
    CEnumNode enumNode = sptrAnalogControl->balanceWhiteAuto();
    if (!enumNode.isReadable())
    {
        std::cout<<"BalanceWhiteAuto not supported."<<std::endl;
        return false;
    }

    if (enableAutoWhiteBalance) 
    {
        // 启用自动白平衡
        if (!enumNode.setValueBySymbol("Continuous"))
        {
            std::cout<<"Failed to set auto white balance to continuous."<<std::endl;
            return false;
        }
        std::cout<<"Auto white balance enabled."<<std::endl;
        return true;
    } 
    else 
    {
        // 禁用自动白平衡
        if (!enumNode.setValueBySymbol("Off"))
        {
            std::cout<<"Failed to turn off auto white balance."<<std::endl;
            return false;
        }
        std::cout<<"Auto white balance disabled."<<std::endl;
        return true;
    }
}

// 设置帧率
bool Camera_Control::setFrameRate(double rate)
{
    IAcquisitionControlPtr sptAcquisitionControl = CSystem::getInstance().createAcquisitionControl(m_pCamera);
    if (sptAcquisitionControl == nullptr)
    {
        return false;
    }

    CBoolNode booleanNode = sptAcquisitionControl->acquisitionFrameRateEnable();
    if (!booleanNode.setValue(true))
    {
         std::cout << "Set acquisitionFrameRateEnable fail." << std::endl;
        return false;
    }

    CDoubleNode doubleNode = sptAcquisitionControl->acquisitionFrameRate();
    if (!doubleNode.setValue(rate))
    {
        std::cout << "Set acquisitionFrameRate fail." << std::endl;
        return false;
    }
    std::cout<<"Set FrameRate Successfully!"<<std::endl;
    return true;
}

/*================ starting camera =================*/

// 启动相机的视频流
bool Camera_Control::videoStart()
{
    // 首先判断是否有相机连接
    if (m_pCamera == nullptr)
    {
        std::cout<<"Start camera fail.No camera connected!"<<std::endl;
        return false;
    }
    // 接着判断是否创建流对象
    if (m_pStreamSource != nullptr)
    {
        return true;
    }
    // 创建流的对象
    m_pStreamSource = CSystem::getInstance().createStreamSource(m_pCamera); 
    if (!m_pStreamSource)
    {
        std::cout<<"Create stream source failed."<<std::endl;
        return false;
    }
    return true;
}

// 开始拉流
bool Camera_Control::startGrabbing()
{
    if(m_pStreamSource)
    {
        if(m_pStreamSource->setBufferCount(15)&&m_pStreamSource->startGrabbing())
        {
            std::cout<<"startGrabbing successfully!"<<std::endl;
            return true;
        }
    }
    return false;
}

// 断开拉流
bool Camera_Control::stopGrabbing()
{
    if (m_pStreamSource)
    {
        if(m_pStreamSource->stopGrabbing())
        {
            std::cout<<"stopGrabbing successfully!"<<std::endl;
            return true;
        }
    }
    else if(!m_pStreamSource)
    {
        std::cout<<"stopGrabbing is already successfully!"<<std::endl;
        return true;

    }
    std::cout<<"stopGrabbing fail."<<std::endl;
    return false;
}

/*================ camera getframe =================*/
// 相机取帧
bool Camera_Control::getFrame(Mat &img )
{

    CFrame frame;
    /// \brief 获取一帧图像，该接口不支持多线程调用
	/// \param [out] frame 一帧图像
	/// \param [in]  timeoutMS 获取一帧图像的超时时长,单位MS,当值为INFINITE时表示无限等待
	/// \return 返回是否成功

    // 获取图片帧，等待300ms 
    if (!m_pStreamSource->getFrame(frame, 300))
    {   
        std::cout<<"getFrame fail."<<std::endl;
        return false;
    }
    // 判断帧的有效性
    if (!frame.valid())
    {
        std::cout<<"Frame is invalid!"<<std::endl;
        return false;
    }
    // 设置图像转换时的参数
    IMGCNV_SOpenParam openParam;
    openParam.width = frame.getImageWidth();
    openParam.height = frame.getImageHeight();
    openParam.paddingX = frame.getImagePadddingX();
    openParam.paddingY = frame.getImagePadddingY();
    openParam.dataSize = frame.getImageSize();
    // 相机读取的图像格式为Dahua::GenICam::gvspPixelBayRG8
    openParam.pixelForamt = frame.getImagePixelFormat();
    // std::cout<<"pixelFormat:"<<frame.getImagePixelFormat()<<std::endl;
    // std::cout<<"BGR:"<<Dahua::GenICam::gvspPixelBGR8<<std::endl;
    // std::cout<<"RGB:"<<Dahua::GenICam::gvspPixelRGB8<<std::endl;
    // std::cout<<"Mono8:"<<Dahua::GenICam::gvspPixelMono8<<std::endl;
    // std::cout<<"BayGB"<<Dahua::GenICam::gvspPixelBayRG8<<std::endl;


    uint8_t *pSrcData = new (std::nothrow) uint8_t[frame.getImageSize()];
    if (pSrcData)
    {
        memcpy(pSrcData, frame.getImage(), frame.getImageSize());
    }
    else
    {
        std::cout<<"new pSrcData failed!"<<std::endl;
        return false;
    }

    int dstDataSize = 0;
    uint8_t *Buffer_;
    if(frame.getImagePixelFormat() == Dahua::GenICam::gvspPixelMono8)
    {
        Buffer_ = new (std::nothrow) uint8_t[frame.getImageSize()];
        
    }
    else{
        Buffer_ = new (std::nothrow) uint8_t[frame.getImageWidth() * frame.getImageHeight() * 3];
    }
    if(Buffer_ == nullptr)
    {
        std::cout<<"new Buffer_ failed!"<<std::endl;
        delete[] pSrcData;
        return false;
    }
    IMGCNV_EErr status = IMGCNV_ConvertToRGB24(pSrcData, &openParam, Buffer_, &dstDataSize);
    if (status != IMGCNV_SUCCESS )
    {
        delete[] pSrcData;
        return false;
    }
    delete[] pSrcData;
    //将读进来的帧数据转化为opencv中的Mat格式操作
    img = Mat(frame.getImageHeight(),frame.getImageWidth(), CV_8UC3, Buffer_).clone();
    if (Buffer_ != nullptr)
    {
        delete[] Buffer_;
        Buffer_ = nullptr;
    }
    // 若相机反装先对图像进行flip,不建议对相机进行反装，否则需要重新计算相机对应的内参
    // flip(img,img,-1);                
    return true;
}




bool Camera_Control::loadSetting(int mode)
{
    CSystem &sysobj = CSystem::getInstance();
    IUserSetControlPtr iSetPtr;
    iSetPtr = sysobj.createUserSetControl(m_pCamera);
    CEnumNode nodeUserSelect(m_pCamera, "UserSetSelector");
    if (mode == 0)
    {
        if (!nodeUserSelect.setValueBySymbol("UserSet1")){
            cout << "set UserSetSelector failed!" << endl;
        }
    }
    else if (mode == 1){
        if (!nodeUserSelect.setValueBySymbol("UserSet2")){
            cout << "set UserSetSelector failed!" << endl;
        }
    }

    CCmdNode nodeUserSetLoad(m_pCamera, "UserSetLoad");
    if (!nodeUserSetLoad.execute()){
        cout << "set UserSetLoad failed!" << endl;
        return false;
    }
    return true;
}

} // namespace pka::dahua_camera

