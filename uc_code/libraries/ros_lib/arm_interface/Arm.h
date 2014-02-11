#ifndef _ROS_arm_interface_Arm_h
#define _ROS_arm_interface_Arm_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace arm_interface
{

  class Arm : public ros::Msg
  {
    public:
      int16_t dof1;
      int16_t dof2;
      int16_t dof3;
      int16_t dof4;
      int16_t dof5;
      int16_t dof6;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_dof1;
      u_dof1.real = this->dof1;
      *(outbuffer + offset + 0) = (u_dof1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dof1.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->dof1);
      union {
        int16_t real;
        uint16_t base;
      } u_dof2;
      u_dof2.real = this->dof2;
      *(outbuffer + offset + 0) = (u_dof2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dof2.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->dof2);
      union {
        int16_t real;
        uint16_t base;
      } u_dof3;
      u_dof3.real = this->dof3;
      *(outbuffer + offset + 0) = (u_dof3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dof3.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->dof3);
      union {
        int16_t real;
        uint16_t base;
      } u_dof4;
      u_dof4.real = this->dof4;
      *(outbuffer + offset + 0) = (u_dof4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dof4.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->dof4);
      union {
        int16_t real;
        uint16_t base;
      } u_dof5;
      u_dof5.real = this->dof5;
      *(outbuffer + offset + 0) = (u_dof5.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dof5.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->dof5);
      union {
        int16_t real;
        uint16_t base;
      } u_dof6;
      u_dof6.real = this->dof6;
      *(outbuffer + offset + 0) = (u_dof6.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dof6.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->dof6);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_dof1;
      u_dof1.base = 0;
      u_dof1.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dof1.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->dof1 = u_dof1.real;
      offset += sizeof(this->dof1);
      union {
        int16_t real;
        uint16_t base;
      } u_dof2;
      u_dof2.base = 0;
      u_dof2.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dof2.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->dof2 = u_dof2.real;
      offset += sizeof(this->dof2);
      union {
        int16_t real;
        uint16_t base;
      } u_dof3;
      u_dof3.base = 0;
      u_dof3.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dof3.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->dof3 = u_dof3.real;
      offset += sizeof(this->dof3);
      union {
        int16_t real;
        uint16_t base;
      } u_dof4;
      u_dof4.base = 0;
      u_dof4.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dof4.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->dof4 = u_dof4.real;
      offset += sizeof(this->dof4);
      union {
        int16_t real;
        uint16_t base;
      } u_dof5;
      u_dof5.base = 0;
      u_dof5.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dof5.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->dof5 = u_dof5.real;
      offset += sizeof(this->dof5);
      union {
        int16_t real;
        uint16_t base;
      } u_dof6;
      u_dof6.base = 0;
      u_dof6.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dof6.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->dof6 = u_dof6.real;
      offset += sizeof(this->dof6);
     return offset;
    }

    const char * getType(){ return "arm_interface/Arm"; };
    const char * getMD5(){ return "9fcaa9dbd66a1045f049732870f734a2"; };

  };

}
#endif