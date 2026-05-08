#include "Message_syncer.h"



Messenger::Messenger(CAN_PARAMETERS can_params, int frequency, int Rx_buffer_size):CAN(can_params)
{
    this->Period = 1.0 / frequency;
    this->running.store(false);
    this->msg_queue.clear();
    this->Rx_buffer_size = Rx_buffer_size;
}


Messenger::~Messenger()
{
    std::stringstream logger;
    int count = 0;

    while(!this->deactivate(logger))
    {
        count++;
        std::this_thread::sleep_for(std::chrono::milliseconds(int(this->Period * 1000 * 5)));
        if(count >= 10)
        {
            logger << "[Error] Failed to deactivate CAN interface" <<std::endl;
            std::cout << logger.str() << std::endl;
            break;
        }
    }
}





bool Messenger::set_control_state(uint16_t vx, uint16_t vy, uint16_t omega)
{
    static std::unique_lock<std::shared_mutex> lock(this->Tx_mutex, std::defer_lock);

    lock.lock();
    this->control_msg.vx = vx;
    this->control_msg.vy = vy;
    this->control_msg.omega = omega;
    lock.unlock();

    return true;
}






bool Messenger::activate(std::stringstream& logger)
{
    if(!CAN::activate(logger))
        return false;

    // 启动线程
    this->running.store(true);
    this->msg_update_thread = std::thread(&Messenger::keep_updating, this);
    this->msg_update_thread.detach();

    return true;
}


bool Messenger::deactivate(std::stringstream& logger)
{
    // 等待线程结束
    if(this->msg_update_thread.joinable())
    {
        this->running.store(false);
        std::this_thread::sleep_for(std::chrono::milliseconds(int(this->Period * 1000 * 5)));
        this->msg_update_thread.join();
    }

    // 关闭CAN连接
    if(!CAN::deactivate(logger))
        return false;

    
    return true;
}






void Messenger::keep_updating()
{
    
    std::stringstream logger;
    can_frame temp_frame;
    
    std::shared_lock<std::shared_mutex> Tx_lock(this->Tx_mutex, std::defer_lock);
    std::unique_lock<std::shared_mutex> Rx_lock(this->Rx_mutex, std::defer_lock);

    uint64_t packet;
    while(this->running.load())
    {
        std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

        packet = 0;

        Tx_lock.lock();
        packet |= static_cast<int64_t>(this->control_msg.vx) << 48;
        packet |= static_cast<int64_t>(this->control_msg.vy) << 32;
        packet |= static_cast<int64_t>(this->control_msg.omega) << 16;

        bool ret = CAN::send(this->control_msg.ID, 8, &packet, logger);
        Tx_lock.unlock();

        if(!ret)
        {
            std::cerr << logger.str() << std::endl;
        }

        if(CAN::receive(temp_frame, logger))
        {
            Rx_lock.lock();
            this->msg_queue.push_back(temp_frame);
            if(this->msg_queue.size() > this->Rx_buffer_size)
                this->msg_queue.pop_front();
            Rx_lock.unlock();
        }


        std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double> dt = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);

        if(dt.count() < this->Period * 1000)
            std::this_thread::sleep_for(std::chrono::milliseconds(int(this->Period * 1000 - dt.count())));

    }
        
}




bool Messenger::any_received(can_frame& frame)
{
    static std::shared_lock<std::shared_mutex> lock(this->Rx_mutex, std::defer_lock);

    if(this->msg_queue.empty())
        return false;

    lock.lock();
    frame = this->msg_queue.front();
    this->msg_queue.pop_front();
    lock.unlock();
    return true;
}
