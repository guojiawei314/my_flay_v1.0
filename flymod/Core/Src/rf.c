#include "rf.h"

remote_t remote;

//*************************************************
//ң��������
//*************************************************
void Remote_Data(void)
{
  remote.lock = Usart2.Rx.Buff[13];
  
  if(!remote.lock)
  {
    remote.Thro  = ((((uint16_t)(Usart2.Rx.Buff[4]))<<8)|(Usart2.Rx.Buff[5])) + 1000;
    
    remote.Yaw   = (int16_t)(((uint16_t)Usart2.Rx.Buff[6]<<8) |Usart2.Rx.Buff[7])/50; //YAW�����
    
    remote.Roll  = (int16_t)(((uint16_t)Usart2.Rx.Buff[8]<<8) |Usart2.Rx.Buff[9])/25; //ROLL�����������20��
    remote.Pitch = (int16_t)(((uint16_t)Usart2.Rx.Buff[10]<<8)|Usart2.Rx.Buff[11])/25;//PITCH�����������20��
  }
  else
  {
    if(remote.Thro>1000) remote.Thro--;
    
    remote.Yaw=0;
    remote.Roll=0;
    remote.Pitch=0;
  }
}

