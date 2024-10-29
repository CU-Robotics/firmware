#include <balance_can_helper.hpp>


MG8016_can::MG8016_can(){}

void MG8016_can::MG8016_init(){
    m_can3.begin();
    m_can3.setBaudRate(1000000);
    m_can3.enableFIFO(true);

    messages[0][0].id = 0x141;
    messages[0][1].id = 0x142;
    messages[0][2].id = 0x143;
    messages[0][3].id = 0x144;
}

uint8_t MG8016_can::MG8016_off(uint8_t id){
    
}

