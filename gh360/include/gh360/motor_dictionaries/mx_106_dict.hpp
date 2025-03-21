#ifndef MX_106_DICT_HPP_
#define MX_106_DICT_HPP_

#include "motor_dict.hpp"

class MX_106_DICT: public MotorDictionary
{
    public:
        MX_106_DICT(int protocol);
        virtual ~MX_106_DICT();
};


#endif // MX_106_DICT_HPP_