#ifndef JOINT_HPP_
#define JOINT_HPP_

#include <iostream>
#include <cstdio>

class Joint
{
    public:
        virtual ~Joint(){}
        
        std::string joint_name;

        virtual std::string get_joint_name() = 0;
        virtual std::string get_joint_type() = 0;

    protected:
        Joint(){}

};

#endif // JOINT_HPP_