#include <balance_can_helper.hpp>


MG8016_can::MG8016_can(){}

void MG8016_can::zero_all(){
    for(int i = 0; i < NUM_MESSAGE_IDS; i++){
        for(int j = 0; j < CAN_MESSAGE_SIZE; j++)
            messages[i].buf[j] = 0x00;
    }
}

void MG8016_can::MG8016_init(){
    m_can3.begin();
    m_can3.setBaudRate(1000000);
    m_can3.enableFIFO(true);

    messages[0].id = 0x141;
    messages[1].id = 0x142;
    messages[2].id = 0x143;
    messages[3].id = 0x144;
}

void MG8016_can::MG8016_off_all(){
    zero_all();
    for(int i = 0; i < NUM_MESSAGE_IDS; i++){
        messages[i].buf[0] = 0x80;
        m_can3.write(messages[i]);
    }
}

void MG8016_can::MG8016_on_all(){
    zero_all();
    for(int i = 0; i < NUM_MESSAGE_IDS; i++){
        messages[i].buf[0] = 0x88;
        m_can3.write(messages[i]);
    }
}

