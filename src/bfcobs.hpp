#include <iostream>

template <size_t BUFFER_SIZE>
class bfcobs
{
private:
    uint8_t buffer[BUFFER_SIZE];
    size_t write_index;
    size_t packet_num;
    void slide_buffer(size_t num){
        for (size_t i = 0; i < BUFFER_SIZE - num; i++)
        {
            this->buffer[i] = this->buffer[i+num];
        }
    }
public:
    bfcobs(){
        write_index = 0;
        packet_num = 0;
    }
    // if return == 0, data packet is not ready
    // if return > 1, data packet is ready and return value is data size
    // if return == -1, error (buffer overflow)
    int push(uint8_t data){
        if(data == 0){
            buffer[write_index] = 0;
            write_index++;
            packet_num++;
            return packet_num;
        }else if(write_index == BUFFER_SIZE){
            return -1;
        }else{
            buffer[write_index] = data;
            write_index++;
            return packet_num;
        }
    }
    // if return == 0, data packet is not ready
    // if return > 1, data packet is ready and return value is data size
    int ready(){
        return packet_num;
    }
    int read(uint8_t *data, size_t *size){
        if(packet_num == 0){
            return 0;
        }else{
            size_t next_zero = buffer[0];
            *size = 0;
            if(next_zero == 0){
                this->slide_buffer(1);
                write_index -= 1;
                packet_num -= 1;
            }
            for (size_t i = 1; i < BUFFER_SIZE; i++)
            {
                if(buffer[i] == 0){
                    break;
                }
                if(i == next_zero){
                    next_zero += buffer[i];
                    data[i-1] = 0;
                    *size += 1;
                }else{
                    data[i-1] = buffer[i];
                    *size += 1;
                }
                if(i == BUFFER_SIZE - 1){
                    return -1;
                }
            }
            this->slide_buffer(*size + 2);
            write_index -= *size + 2;
            packet_num--;
            return packet_num;
        }

    }
};
template <size_t N>
std::array<uint8_t, N+2> cobs_encode(std::array<uint8_t, N> input){
    std::array<uint8_t, N+2> encoded_data;
    encoded_data.fill(0x00);
    size_t index_last_zero = 0;
    for(size_t i=0; i < input.size(); i++){
        if(input[i] == 0x00){
            encoded_data[index_last_zero] = i + 1 - index_last_zero;
            index_last_zero = i + 1;
        }else{
            encoded_data[i+1] = input[i];
        }
    }
    encoded_data[index_last_zero] = input.size() + 1 - index_last_zero;
    return encoded_data;
}
