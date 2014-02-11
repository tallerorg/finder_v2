#ifndef _ROS_finder_TwoWheelInt16_h
#define _ROS_finder_TwoWheelInt16_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace finder
{

  class TwoWheelInt16 : public ros::Msg
  {
    public:
      int16_t wheel1;
      int16_t wheel2;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_wheel1;
      u_wheel1.real = this->wheel1;
      *(outbuffer + offset + 0) = (u_wheel1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel1.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->wheel1);
      union {
        int16_t real;
        uint16_t base;
      } u_wheel2;
      u_wheel2.real = this->wheel2;
      *(outbuffer + offset + 0) = (u_wheel2.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel2.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->wheel2);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_wheel1;
      u_wheel1.base = 0;
      u_wheel1.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel1.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->wheel1 = u_wheel1.real;
      offset += sizeof(this->wheel1);
      union {
        int16_t real;
        uint16_t base;
      } u_wheel2;
      u_wheel2.base = 0;
      u_wheel2.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel2.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->wheel2 = u_wheel2.real;
      offset += sizeof(this->wheel2);
     return offset;
    }

    const char * getType(){ return "finder/TwoWheelInt16"; };
    const char * getMD5(){ return "4b5429d7593bec59a254b0cfd2cbd095"; };

  };

}
#endif