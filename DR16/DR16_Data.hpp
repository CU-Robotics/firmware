#pragma region DR16_Data Definition
#ifndef DR16_DATA_HPP
#define DR16_DATA_HPP

// Documentation for this radio receiver is like
// non existant. We need to make our own receiver @hardware peeps.
// This is the best I could find:
// http://dl.djicdn.com/downloads/dt7/en/DT7&DR16_RC_System_User_Manual_v2.00_en.pdf

const int DATA_SCALAR = 255;

class DR16_Data {
public:
    DR16_Data();

    void read_state();

    float get_r_stick_x();
    
    float get_r_stick_y();

    float get_l_stick_x();

    float get_l_stick_y();

    float get_wheel();

    float get_l_switch();

    float get_r_switch();

private:
    struct Data {
        float r_stick_x;
        float r_stick_y;
        float l_stick_x;
        float l_stick_y;
        float wheel;

        // These are both interpreted/read as int types but are casted to float
        float l_switch;
        float r_switch;
    }; Data data;

    float bounded_map(int value, int in_low, int in_high, int out_low, int out_high);
};

#endif // DR16_DATA_HPP

#pragma endregion