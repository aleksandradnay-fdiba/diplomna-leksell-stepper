/*
 * File:   uart_print.c
 * Author: sENEV
 *
 * Created on June 17, 2025, 2:19 PM
 */

#include "xc.h"
#include "uart_print.h"

const char charset[] = "0123456789ABCDEF";

void (*UART_Write)(uint8_t);

void send_uint8_as_hex_str_to_uart(uint8_t digit)
{
    uint8_t index;
        
    for(int j = 1; j >= 0; j--) {
        index = (digit >> 4*j)&0x0000000F;
        UART_Write(*(charset + index));
    }
}

void send_uint16_as_hex_str_to_uart(uint16_t digit)
{
    uint8_t index;
        
    for(int j = 3; j >= 0; j--) {
        index = (digit >> 4*j)&0x0000000F;
        UART_Write(*(charset + index));
    }
}

void send_uint32_as_hex_str_to_uart(uint32_t digit)
{
    uint8_t index;
        
    for(int j = 7; j >= 0; j--) {
        index = (digit >> 4*j)&0x0000000F;
        UART_Write(*(charset + index));
    }
}

void send_uint16_as_str_to_uart(int16_t number)
{
    uint8_t curr_char;
    uint8_t sending = 0;
    
    curr_char = (uint8_t)(number/10000);
    if(curr_char) {
        UART_Write(*(charset + curr_char));
        sending = 1;
    }
    number = number%10000;
    
    curr_char = (uint8_t)(number/1000);
    if(curr_char || sending) {
        UART_Write(*(charset + curr_char));
        sending = 1;
    }
    number = number%1000;
    
    curr_char = (uint8_t)(number/100);
    if(curr_char || sending) {
        UART_Write(*(charset + curr_char));
        sending = 1;
    }
    number = number%100;
    
    curr_char = (uint8_t)(number/10);
    if(curr_char || sending) {
        UART_Write(*(charset + curr_char));
        sending = 1;
    }
    number = (uint8_t)(number%10);

    curr_char = (uint8_t)(number%10);
    UART_Write(*(charset + curr_char));
}

void send_int32_as_str_to_uart(int32_t number)
{
    uint8_t negative = 0;
    uint32_t value;
    char buffer[11];
    int i = 0;

    if (number < 0) {
        negative = 1;
        value = (uint32_t)(-number);
    } else {
        value = (uint32_t)number;
    }

    if (negative) {
        UART_Write('-');
    }

    if (value == 0) {
        UART_Write('0');
        return;
    }

    while (value > 0) {
        buffer[i++] = (char)('0' + (value % 10));
        value /= 10;
    }

    while (i > 0) {
        UART_Write(buffer[--i]);
    }
}

void send_int16_as_str_to_uart(int16_t number)
{
    uint8_t negative_number = 0;
    
    if (number < 0) {
        negative_number = 1;
        UART_Write(0x2D);
        number *= -1;  
    }
    
    send_uint16_as_str_to_uart(number);
}

void send_string_to_uart(char *p, uint8_t length)
{
    for(int j = 0; j < length; j++) {
        UART_Write(*(p+j));
    }
}

void send_string(char *p) {
    while(*p != '\0') {
        UART_Write(*p);
        p++;
    }

}

void send_float_as_str_to_uart(float number, uint8_t encoding_custom)
{
    uint8_t dec_point_loc;
    uint8_t negative_number;
    uint8_t int_help;
    uint8_t bytes_sent = 0;
    
    negative_number = 0;
    if (number < 0) {
        negative_number = 1;
        if (!encoding_custom) {
            UART_Write(0x2D);
            bytes_sent++;
        }
        number *= -1;   
    }
    if (number > 10000000)   { dec_point_loc = 9;}
    if (number < 10000000)   { dec_point_loc = 8;}
    if (number < 1000000)    { dec_point_loc = 7;}
    if (number < 100000)     { dec_point_loc = 6;}
    if (number < 10000)      { dec_point_loc = 5;}
    if (number < 1000)       { dec_point_loc = 4;}  
    if (number < 100)        { dec_point_loc = 3;}
    if (number < 10)         { dec_point_loc = 2;}
           
    for (int j = 0; j < dec_point_loc - 2; j++) {
        number = number / 10;
    }
    
    if (encoding_custom) {
        dec_point_loc -= 2;
        if (negative_number) {
            dec_point_loc += 8;
        }

        UART_Write(*(charset + dec_point_loc));
        bytes_sent++;
        
        while (bytes_sent < 8) {
            int_help = (uint8_t)(number);
            UART_Write(*(charset + (int_help)));
            number = (number - int_help) * 10;
            bytes_sent++;
        }
        return;
    }
    while (bytes_sent < 8) {
        int_help = (uint8_t)(number);
        //while(!TXSTAbits.TRMT){};
        if(bytes_sent + 1 - negative_number == dec_point_loc) {
            UART_Write(0x2E);
        } else {
            UART_Write(*(charset + (int_help)));
            number = (number - int_help) * 10;
        }   
        bytes_sent++;
    }   
}