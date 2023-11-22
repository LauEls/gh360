#include "joint.hpp"


double Joint::positionIntToDouble(int data)
{
    double value;

    value = data * (2*M_PI/4096);

    return value;
}

double Joint::velocityIntToDouble(int data)
{
    double value;

    value = data * 0.229 * 0.10472;

    return value;
}

double Joint::currentIntToDouble(int data)
{
    double value;

    if (data > 0x7fff)
    {
        value = (data - 65536) * 3.36;
    }
    else
    {
        value = data * 3.36;
    }

    return value;
}

int Joint::positionDoubleToInt(double value)
{
    int data;

    data = value / (2*M_PI/4096);

    return data;
}

int Joint::velocityDoubleToInt(double value)
{
    int data;

    data = value / 0.229 / 0.10472;

    return data;
}

int Joint::currentDoubleToInt(double value)
{
    int data;

    if (value < 0)
    {
        data = value / 3.36 + 65536;
    }
    else{
        data = value / 3.36;
    }

    return data;
}

double Joint::calc_set_motor_goal_pos(double goal_pos_adjusted, double offset, int movement_direction)
{
    double goal_pos_raw = (goal_pos_adjusted + offset) * movement_direction;
    return goal_pos_raw;
}

double Joint::calc_get_motor_pos(double present_pos_raw, double offset, int movement_direction)
{
    double present_pos_adjusted = present_pos_raw / movement_direction - offset;
    return present_pos_adjusted;
}

