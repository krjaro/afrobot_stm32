#ifndef _ROS_rtabmap_ros_CameraModels_h
#define _ROS_rtabmap_ros_CameraModels_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "rtabmap_ros/CameraModel.h"

namespace rtabmap_ros
{

  class CameraModels : public ros::Msg
  {
    public:
      uint32_t models_length;
      typedef rtabmap_ros::CameraModel _models_type;
      _models_type st_models;
      _models_type * models;

    CameraModels():
      models_length(0), models(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->models_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->models_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->models_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->models_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->models_length);
      for( uint32_t i = 0; i < models_length; i++){
      offset += this->models[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t models_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      models_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      models_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      models_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->models_length);
      if(models_lengthT > models_length)
        this->models = (rtabmap_ros::CameraModel*)realloc(this->models, models_lengthT * sizeof(rtabmap_ros::CameraModel));
      models_length = models_lengthT;
      for( uint32_t i = 0; i < models_length; i++){
      offset += this->st_models.deserialize(inbuffer + offset);
        memcpy( &(this->models[i]), &(this->st_models), sizeof(rtabmap_ros::CameraModel));
      }
     return offset;
    }

    const char * getType(){ return "rtabmap_ros/CameraModels"; };
    const char * getMD5(){ return "b9cc1f538c510301e1a2175d9d0ad481"; };

  };

}
#endif
