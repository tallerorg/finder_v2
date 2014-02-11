#ifndef _ROS_finder_FourArmsInt16_h
#define _ROS_finder_FourArmsInt16_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace finder
{

  class FourArmsInt16 : public ros::Msg
  {
    public:
      int16_t arm1;
      int16_t arm2;
      int16_t arm3;
      int16_t arm4;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_arm1;
      u_arm1.real = this->arm1;
      *(outbuffer + offset + 0) = (u_arm1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_arm1.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->arm1);
      union {
        int16_t real;
        uint16_t base;
      } u_arm2;
      u_arm2.real = this->arm2;
      *(outbuffer + offset + 0) = (u_arm2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_arm2.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->arm2);
      union {
        int16_t real;
        uint16_t base;
      } u_arm3;
      u_arm3.real = this->arm3;
      *(outbuffer + offset + 0) = (u_arm3.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_arm3.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->arm3);
      union {
        int16_t real;
        uint16_t base;
      } u_arm4;
      u_arm4.real = this->arm4;
      *(outbuffer + offset + 0) = (u_arm4.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_arm4.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->arm4);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_arm1;
      u_arm1.base = 0;
      u_arm1.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_arm1.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->arm1 = u_arm1.real;
      offset += sizeof(this->arm1);
      union {
        int16_t real;
        uint16_t base;
      } u_arm2;
      u_arm2.base = 0;
      u_arm2.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_arm2.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->arm2 = u_arm2.real;
      offset += sizeof(this->arm2);
      union {
        int16_t real;
        uint16_t base;
      } u_arm3;
      u_arm3.base = 0;
      u_arm3.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_arm3.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->arm3 = u_arm3.real;
      offset += sizeof(this->arm3);
      union {
        int16_t real;
        uint16_t base;
      } u_arm4;
      u_arm4.base = 0;
      u_arm4.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_arm4.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->arm4 = u_arm4.real;
      offset += sizeof(this->arm4);
     return offset;
    }

    const char * getType(){ return "finder/FourArmsInt16"; };
    const char * getMD5(){ return "5226dcfcbb506c986d81f1b7743ff9a8"; };

  };

}
#endif