#ifndef MX_64_DICT_HPP_
#define MX_64_DICT_HPP_

#include "motor_dict.hpp"

class MX_64_DICT: public MotorDictionary
{
    public:
        MX_64_DICT(int protocol);
        virtual ~MX_64_DICT();
};

#endif // MX_64_DICT_HPP_