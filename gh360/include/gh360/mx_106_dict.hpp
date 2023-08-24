#ifndef MX_106_DICT_HPP_
#define MX_106_DICT_HPP_

#include "motor_dict.hpp"

namespace gh360 
{
    class MX_106_DICT: public MotorDictionary
    {
        public:
            MX_106_DICT(int protocol);
            virtual ~MX_106_DICT();
            
        private:


    };
}

// #include <pluginlib/class_list_macros.hpp>

// PLUGINLIB_EXPORT_CLASS(gh360::MX_106_DICT, gh360::MotorDictionary)

#endif // MX_106_DICT_HPP_