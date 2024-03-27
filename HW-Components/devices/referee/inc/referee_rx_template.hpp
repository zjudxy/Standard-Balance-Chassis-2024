

/**
 * @file      referee_rx_template.hpp
 * @brief
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 * @date      2023-10-30
 *
 * @copyright Copyright (c) 2023 Hello World Team, Zhejiang University. All Rights Reserved.
 *
 * @attention
 *
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | {now_year} | ZhouShichan | description |
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @par last edit time  2023-10-30
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HWCOMPONETS_DEVICES_REFEREE_RX_TEMPLATE_H_
#define HWCOMPONETS_DEVICES_REFEREE_RX_TEMPLATE_H_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>
#include <cstring>

#include "referee_protocol.hpp"
#include "tick.hpp"

namespace hello_world
{
namespace devices
{
namespace referee
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class Referee;

class RxPackageInfo
{
    friend Referee;

   protected:
    bool is_handled_ = false;
    FrameCmdId command_id_ = kFrameCmdIdNone;
    uint32_t update_interval_ = 1000u;
    uint32_t last_update_tick_ = 0;
    virtual bool decode(const uint8_t *frame_cmd_data_ptr) = 0;

   public:
    RxPackageInfo(const FrameCmdId command_id, const uint32_t frequence)
    {
        this->command_id_ = command_id;

        if (frequence >= 1) {
            this->update_interval_ = 1000 / frequence;
        } else {
            this->update_interval_ = 1000;
        }

        this->last_update_tick_ = 0;
    };

    void setHandled()
    {
        is_handled_ = true;
    };

    bool isUpdated() const
    {
        uint32_t now_tick = dsp::tick::GetTickMs();
        return ((now_tick - last_update_tick_) <= update_interval_) && (is_handled_ == false);
    };
};

template <typename T>
class RxPackage : public RxPackageInfo
{
    bool decode(const uint8_t *frame_cmd_data_ptr);
    T data_;

   public:
    const T &data()
    {
        return data_;
    }
    RxPackage(const FrameCmdId command_id, const uint32_t frequence) : RxPackageInfo(command_id, frequence){};
};

template <typename T>
bool RxPackage<T>::decode(const uint8_t *frame_cmd_data_ptr)
{
    FrameCmdId cmd_id = (FrameCmdId)(frame_cmd_data_ptr[0] | frame_cmd_data_ptr[1] << 8);
    if (command_id_ != cmd_id) {
        return false;
    }
    memcpy(&data_, frame_cmd_data_ptr + 2, sizeof(T));
    last_update_tick_ = dsp::tick::GetTickMs();
    is_handled_ = false;
    return true;
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

}  // namespace referee

}  // namespace devices

}  // namespace hello_world

#endif /* HWCOMPONETS_DEVICES_REFEREE_RX_TEMPLATE_H_ */
