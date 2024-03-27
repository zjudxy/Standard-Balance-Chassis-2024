/** 
 * @file      rfr_decoder.cpp
 * @author    ZhouShichan (zsc19823382069@163.com)
 * @date      2024-02-19
 * @brief     
 * @par last editor  ZhouShichan (zsc19823382069@163.com)
 * @version   1.0.0
 * 
 * @copyright Copyright (c) 2024 Hello World Team, Zhejiang University. All Rights Reserved.
 * 
 * @attention 
 * 
 * @par history
 * | Version | Date | Author | Description |
 * | :---: | :---: | :---: | :---: |
 * | 1.0.0 | 2024-MM-DD | ZhouShichan | description |
 */
/* Define to prevent recursive inclusion -------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "rfr_decoder.hpp"

#include <cstring>

#include "rfr_crc.hpp"
/* Exported macro ------------------------------------------------------------*/
namespace hello_world
{
namespace referee
{
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions -----------------------------------------------*/

bool RfrDecoder::decodeFrame(const uint8_t *data_ptr, size_t length)
{
  bool process_result = false;
  for (size_t i = 0; i < length; i++) {
    process_result = processByte(data_ptr[i]);
    if (process_result) {
      break;
    }
  }
  return process_result;
};

bool RfrDecoder::processByte(uint8_t byte)
{
  bool process_result = false;
  switch (rx_status_) {
    case kRxStatusWaitingHeaderSof:
      if (byte == kRefereeFrameHeaderSof) {
        frame_.raw[data_index_++] = byte;
        rx_status_ = kRxStatusHeader;
      } else {
        restartDecodeFrame();
      }
      break;
    case kRxStatusHeader:
      frame_.raw[data_index_++] = byte;

      if (data_index_ == sizeof(FrameHeader)) {
        if (VerifyCrc8CheckSum(frame_.raw, sizeof(FrameHeader))) {
          expect_length_ = frame_.decoded.header.data_length + sizeof(FrameHeader) + sizeof(CmdId) + sizeof(FrameTail);
          rx_status_ = kRxStatusCmdDataTail;
        } else {
          restartDecodeFrame();
        }
      }
      break;
    case kRxStatusCmdDataTail:
      frame_.raw[data_index_++] = byte;
      if (data_index_ == expect_length_) {
        if (VerifyCrc16CheckSum(frame_.raw, expect_length_)) {
          process_result = decodeRxPackage(frame_.decoded.cmd_id, frame_.decoded.data);
        }
        restartDecodeFrame();
      }
      break;
    default:
      restartDecodeFrame();
      break;
  }
  return process_result;
};
void RfrDecoder::restartDecodeFrame()
{
  memset(&frame_, 0, sizeof(frame_));
  data_index_ = 0;
  expect_length_ = 0;
  rx_status_ = kRxStatusWaitingHeaderSof;
};

bool RfrDecoder::decodeRxPackage(const CmdId &cmd_id, const uint8_t *data_ptr)
{
  bool process_result = false;
  for (auto rx_package : rx_package_list_) {
    process_result |= rx_package->decode(cmd_id, data_ptr);
  }
  return process_result;
};

void RfrDecoder::appendRxPackage(ProtocolRxPackage *rx_package_ptr)
{
  bool is_existed = false;
  for (auto rx_package : rx_package_list_) {
    if (rx_package == rx_package_ptr) {
      is_existed = true;
      break;
    }
  }
  if (!is_existed) {
    rx_package_list_.push_back(rx_package_ptr);
  }
};
/* Private function definitions -----------------------------------------------*/

}  // namespace referee
}  // namespace hello_world