#ifndef MX_64_DICT_HPP_
#define MX_64_DICT_HPP_

#include "motor_dict.hpp"

namespace gh360 
{
    class MX_64_DICT: public MotorDictionary
    {
        public:
            MX_64_DICT(int protocol);
            virtual ~MX_64_DICT();

            
        private:


    };
}

// #include <pluginlib/class_list_macros.hpp>

// PLUGINLIB_EXPORT_CLASS(gh360::MX_64_DICT, gh360::MotorDictionary)

#endif // MX_64_DICT_HPP_