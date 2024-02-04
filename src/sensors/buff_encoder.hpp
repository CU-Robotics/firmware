#ifndef BUFF_ENCODER_H
#define BUFF_ENCODER_H
class BuffEncoder
{
    private:
        /// @brief the pin
        int _nCS;
        /// @brief current angle in radians
        float _radians;
    public:
        BuffEncoder(int nCS);//Construct a new BuffEncoder object
        void read();//Get the angle and get the radians
        float get_radians();//return angle in radians
};
#endif