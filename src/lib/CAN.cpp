#include"CAN.h"





CAN::CAN(const std::string interface_name, int bitrate, bool enable_canfd, int data_bitrate)      // 构造函数
{
    this->config.interface_name = interface_name;
    this->config.bitrate = bitrate;
    this->config.enable_canfd = enable_canfd;
    this->config.data_bitrate = data_bitrate;

    this->sock = -1;
    this->ACTIVATED = false;

    // 检查 root 权限
    if (geteuid() != 0) {
        std::cout << "[Warn]  root 权限缺失" << std::endl;
    }

}

CAN::CAN(const CAN_PARAMETERS& params)      // 构造函数
{
    this->config = params;
    this->sock = -1;
    this->ACTIVATED = false;

    // 检查 root 权限
    if (geteuid() != 0) {
        std::cout << "[Warn]  root 权限缺失" << std::endl;
    }
}



CAN::CAN()
{
    this->sock = -1;
    this->ACTIVATED = false;
}



CAN::~CAN()
{
    std::stringstream logger;
    int attempt = 0;
    while(!this->deactivate(logger))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        attempt++;

        if(attempt>=10)
        {
            logger << "[Error] Failed to deactivate CAN interface '" << this->config.interface_name << "' after 10 attempts." << std::endl;
            std::cout << logger.str() << std::endl;
            break;
        }
    }
}




bool CAN::activate(std::stringstream& logger)    // 激活 CAN 接口
{

    // 检查 root 权限
    if (geteuid() != 0) {
        logger << "[Error] Root privileges required !" << std::endl;
        return false;
    }

    // 先关闭 socket
    if(!this->sock<0)
    {
        close(this->sock);
        this->sock = -1;
    }





    // 配置 CAN ================================================================
    
    std::stringstream cmd;
    
    


    // 拉下接口
    cmd << "ip link set " << this->config.interface_name << " down";

    // 执行
    int ret = system(cmd.str().c_str());
    cmd.str("");
    // 检查
    if(ret!=0)
    {
        logger << "[Error] Failed to pull CAN interface '" << this->config.interface_name << "' down: " << ret << std::endl;
        close(this->sock);
        return false;
    }




    // 启用接口
    cmd << "ip link set " << this->config.interface_name << " up"
        << " type can bitrate " << this->config.bitrate ;

    if(this->config.enable_canfd)
    {
        cmd << " fd on dbitrate "<< this->config.data_bitrate;
    }
    else{
        cmd << " fd off";
    }

    // 执行
    ret = system(cmd.str().c_str());
    cmd.str("");
    // 检查
    if(ret!=0)
    {
        logger << "[Error] Failed to set interface '" << this->config.interface_name << "' up: " << ret << std::endl;
        close(this->sock);
        return false;
    }

    logger << "[Info]  Hardware config Done" << std::endl;




    // 配置并绑定socket ======================================================================

    // 初始化 socket 套接字
    this->sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    if (this->sock < 0){
        std::error_code ec{errno, std::system_category()};
        logger << "[Error] Failed to create socket: " << ec.message() << std::endl;
        return false;
    }
    logger << "[Info]  Socket created, socket descriptor: " << this->sock << std::endl;
    



    // 获取can接口索引
    ifreq ifr{};

    size_t len = this->config.interface_name.copy(ifr.ifr_name, IFNAMSIZ - 1);
    ifr.ifr_name[len] = '\0';

    if (ioctl(this->sock, SIOCGIFINDEX, &ifr) < 0) {
        std::error_code ec{errno, std::system_category()};
        logger << "[Error] Failed to get interface index for '" << this->config.interface_name << "' (" << ec.message() << ")" << std::endl;
        logger << "[Tips]  Please check if the interface is already up (ip link show type can)" << std::endl;
        close(this->sock);
        return false;
    }
    logger << "[Info]  Interface index: " << ifr.ifr_ifindex << std::endl;




    // 绑定 socket 到 can 接口
    sockaddr_can addr = {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(this->sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::error_code ec{errno, std::system_category()};
        logger << "[Error] Failed to bind socket to interface '" << this->config.interface_name << "' (" << ec.message() << ")" << std::endl;
        close(this->sock);
        return false;
    }
    logger << "[Info]  Socket bound to interface '" << this->config.interface_name << "'" << std::endl;



    
    int flags = fcntl(this->sock, F_GETFL, 0);
    fcntl(this->sock, F_SETFL, flags | O_NONBLOCK); // 开启非阻塞模式
    logger << "[Info]  Socket set to non-blocking mode" << std::endl;






    // 开启socket CAN FD模式
    if (this->config.enable_canfd) 
    {
        int fd_flags = 1;
        if (setsockopt(this->sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                    &fd_flags, sizeof(fd_flags)) < 0) {
            std::error_code ec{errno, std::system_category()};
            logger << "[Error] Failed to enable CAN_RAW_FD_FRAMES: " << ec.message() << std::endl;
            close(this->sock);
            return false;
        }
        logger << "[Info]  CAN FD frames enabled on socket" << std::endl;
    }











    // 激活成功
    logger << "[Info]  CAN Interface '" << this->config.interface_name << "' activated successfully !" << std::endl;

    logger << "[Info]  name:" << this->config.interface_name << " type:" << (this->config.enable_canfd ? "canfd":"can") << " bitrate:" << this->config.bitrate << "bps";

    if(this->config.enable_canfd)
    {
        logger << " dbitrate:" << this->config.data_bitrate << "bps" << std::endl;
    }
    else{
        logger << std::endl;
    }




    this->ACTIVATED = true;

    return true;
}






// 取消激活CAN接口
bool CAN::deactivate(std::stringstream& logger)
{
    if (this->sock >= 0) {
        
        close(this->sock);
    }
    this->sock = -1;

    std::stringstream cmd;
    // 拉下接口
    cmd << "ip link set " << this->config.interface_name << " down";

    // 执行
    int ret = system(cmd.str().c_str());
    cmd.str("");
    // 检查
    if(ret!=0)
    {
        logger << "[Error] Failed to pull CAN interface '" << this->config.interface_name << "' down: " << ret << std::endl;
        close(this->sock);
        return false;
    }



    logger << "[Info]  CAN Interface '" << this->config.interface_name << "' deactivated successfully !" << std::endl;



    this->ACTIVATED = false;
    
    return true;
}







// 设置CAN参数
bool CAN::set_parameters(const CAN_PARAMETERS& params, std::stringstream& logger){
    this->config = params;
    return this->activate(logger);
}







// 发送CAN帧
bool CAN::send(uint32_t id, uint8_t data_len, void* data, std::stringstream& logger)
{
    // 检查接口状态
    if (!this->ACTIVATED)
    {
        logger << "[Error] CAN Interface is not activated !" << std::endl;
        return false;
    }

    // 检查数据指针是否为空
    if(data == nullptr || data_len == 0)
    {
        logger << "[Error] Data pointer is null or data length is 0 !" << std::endl;
        return false;
    }

    // 检查数据长度是否超过最大长度
    if(data_len > (this->config.enable_canfd ? 64 : 8))
    {
        logger << "[Error] Data length is too long for current CAN mode !" << std::endl;
        return false;
    }


    if(!this->config.enable_canfd)
    {
        memset(&this->Frame, 0, sizeof(this->Frame));


        // 构建CAN帧(暂未完成FD模式)
        this->Frame.can_id = id;
        this->Frame.can_dlc = data_len;


        for(int i = 0; i < data_len; i++)
        {
            this->Frame.data[data_len - i - 1] = *((uint8_t*)data + i);
        }




        size_t ret = write(this->sock, &this->Frame, CAN_MTU);
        if(ret!= CAN_MTU)
        {
            std::error_code ec{errno, std::system_category()};
            logger << "[Error] Failed to send CAN frame: " << ret << std::endl;
            logger << "[Error] Error message: " << ec.message() << std::endl;

            return false;
        }
    }

    else
    {
        memset(&this->Frame_FD, 0, sizeof(this->Frame_FD));


        this->Frame_FD.flags = CANFD_BRS;
        this->Frame_FD.can_id = id;
        this->Frame_FD.len = data_len;



        for(int i = 0; i < data_len; i++)
        {
            this->Frame_FD.data[data_len - i - 1] = *((uint8_t*)data + i);
        }




        size_t ret = write(this->sock, &this->Frame_FD, CANFD_MTU);
        if(ret!= CANFD_MTU)
        {
            std::error_code ec{errno, std::system_category()};
            logger << "[Error] Failed to send CANFD frame: " << ret << std::endl;
            logger << "[Error] Error message: " << ec.message() << std::endl;

            return false;
        }
    }
    return true;
}



// 接收CAN帧
bool CAN::receive(can_frame& frame, std::stringstream& logger)
{
    // 检查接口状态
    if (!this->ACTIVATED) {
        logger << "[Error] CAN Interface is not activated !" << std::endl;
        return false;
    }
    if(this->config.enable_canfd)
    {
        logger << "[Error] CANFD mode not matched ! (Current: " << (this->config.enable_canfd ? "canfd":"can") << ")" << std::endl;
        return false;
    }

    ssize_t ret = read(this->sock, &frame, CAN_MTU);
    if (ret == -1) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            logger << "[Info]  no CAN frame received" << std::endl;
            // 无数据可用
            return false;
        }

        std::error_code ec{errno, std::system_category()};
        logger << "[Error] Failed to receive CAN frame: " << ec.message() << std::endl;

        return false;

    }
    if (ret != CAN_MTU) {

        logger << "[Error] Incomplete CAN frame received: " << ret << " bytes" << std::endl;
        return false;
    }


    return true;
}

// 接收CANFD帧
bool CAN::receive(canfd_frame& frame, std::stringstream& logger)
{
    // 检查接口状态
    if (!this->ACTIVATED) {
        logger << "[Error] CAN Interface is not activated !" << std::endl;
        return false;
    }

    if(!this->config.enable_canfd)
    {
        logger << "[Error] CANFD mode not matched ! (Current: " << (this->config.enable_canfd ? "canfd":"can") << ")" << std::endl;
        return false;
    }

    
    ssize_t ret = read(this->sock, &frame, CANFD_MTU);
    if (ret == -1) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            logger << "[Info]  no CAN frame received" << std::endl;
            // 无数据可用
            return false;
        }
        std::error_code ec{errno, std::system_category()};
        logger << "[Error] Failed to receive CANFD frame: " << ec.message() << std::endl;
        return false;
    }
    if (ret != CANFD_MTU) {
        std::error_code ec{errno, std::system_category()};
        logger << "[Error] Failed to receive CANFD frame: " << ret << std::endl;
        logger << "[Error] Error message: " << ec.message() << std::endl;

        return false;
    }


    return true;
}
