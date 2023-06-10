/*
 * Driver code for LED which includes initialization and writing to the 7 segment display
 *
 */

#include <i2c.h>
#include <led_driver.h>
#include <unistd.h>

#define LED_SLAVE_ADDR (uint8_t) 0xE0

//from ft16K33v110.pdf datasheet

// enable oscillator
#define LED_OSCIL_ON (uint8_t) 0x21

// enable display
#define LED_DISPLAY_ON (uint8_t) 0x81

// set led to full brightness
#define LED_FULL_BRIGHTNESS (uint8_t) 0xEF

//buffer size of RAM
#define BUF_SIZE 16

#define four_bit_mask 0xF

uint8_t hex_to_seven_segment(uint8_t hex);

/*
 * @brief - initializes the led driver
 * @param - None
 */

void led_driver_init(uint32_t addr){
  //edit this:
  (void) addr;
  uint8_t buf[BUF_SIZE] = {0};

  buf[0] = LED_OSCIL_ON;
  i2c_master_write(buf, 1, LED_SLAVE_ADDR);

  buf[0] = LED_DISPLAY_ON;
  i2c_master_write(buf, 1, LED_SLAVE_ADDR);

  buf[0] = LED_FULL_BRIGHTNESS;
  i2c_master_write(buf, 1, LED_SLAVE_ADDR);
  
  // clear RAM
  // first send 0, then send 0, then 2, then 0, etc... for rest of rows
  buf[0] = 0;
  buf[1] = 0;
  buf[2] = 2;
  buf[3] = 0;
  buf[4] = 6;
  buf[5] = 0;
  buf[6] = 8;
  buf[7] = 0;
  
  i2c_master_write(buf, BUF_SIZE, LED_SLAVE_ADDR);

  return;
}

/* 
 * @brief - sets the led value of led number led_num to input
 * @param[in] row_num which 7 segment display to write to
 * @param[in] disp_data led number to write input to must be 0, 2, 6, or 8
 */
void led_set_display(uint32_t input){
  
  uint8_t buf[BUF_SIZE];


//      led_set_display(light_val / 100, 6); // hundreds place
  //  led_set_display((light_val % 100) / 10, 2); // tens place
   // led_set_display(light_val % 10, 0); // ones place

  uint32_t zeroth_four_bits = four_bit_mask & input;
  uint32_t first_four_bits = ((four_bit_mask << 4) & input) >> 4;
  uint32_t second_four_bits = ((four_bit_mask << 8) & input) >> 8;
  uint32_t third_four_bits = ((four_bit_mask << 12) & input) >> 12;

  // send row number, then value
  buf[0] = 0;
  buf[1] = hex_to_seven_segment((uint8_t)zeroth_four_bits);
  
  buf[2] = 2;
  buf[3] = hex_to_seven_segment((uint8_t)first_four_bits);

  buf[4] = 6;
  buf[5] = hex_to_seven_segment((uint8_t)second_four_bits);

  buf[6] = 8;
  buf[7] = hex_to_seven_segment((uint8_t)third_four_bits);

  i2c_master_write(buf, BUF_SIZE, LED_SLAVE_ADDR);

  return;
}

/*
 * @brief - convert hex value to valid 7 segment display value
 * @param[in] hex value to convert to valid 7 segment display value
 * @return hex value converted to valid 7 segment display value
 */

uint8_t hex_to_seven_segment(uint8_t hex){
  uint8_t result;
  switch (hex){
    case 0x0:
      result = 0b00111111;
      break;
    case 0x1:
      result = 0b00000110;
      break;
    case 0x2:
      result = 0b01011011;
      break;
    case 0x3:
      result = 0b01001111;
      break;
    case 0x4:
      result = 0b01100110;
      break;
    case 0x5:
      result = 0b01101101;
      break;
    case 0x6:
      result = 0b01111101;
      break;
    case 0x7:
      result = 0b00000111;
      break;
    case 0x8:
      result = 0b01111111;
      break;
    case 0x9:
      result = 0b01101111;
      break;
    case 0xA:
      result = 0b01110111;
      break;
    case 0xB:
      result = 0b01111100;
      break;
    case 0xC:
      result = 0b00111001;
      break;
    case 0xD:
      result = 0b01011110;
      break;
    case 0xE:
      result = 0b01111001;
      break;
    case 0xF:
      result = 0b01110001;
      break;
    default:
      result = 0;
  }
  return result;
}
