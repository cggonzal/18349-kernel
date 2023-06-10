/**
 * @brief Driver code for i2c which includes initializing i2c bus 1 and writing data to
 * a given slave address.
 */

#include <gpio.h>
#include <i2c.h>
#include <unistd.h>
#include <rcc.h>

/** i2c register map see page 505 in reference manual */
struct i2c_reg_map{
    volatile uint32_t CR1;   /**< Control Register 1 >*/
    volatile uint32_t CR2;   /**< Control Register 2 >*/
    volatile uint32_t OAR1;  /**< Own Address Register 1 >*/
    volatile uint32_t OAR2;  /**< Own Address Register 2 >*/
    volatile uint32_t DR;    /**< Data Register >*/
    volatile uint32_t SR1;   /**< Status Register 1 >*/
    volatile uint32_t SR2;   /**< Status Register 2 >*/
    volatile uint32_t CCR;   /**< Clock Control Register >*/
    volatile uint32_t TRISE; /**< Rise Time Register >*/
    volatile uint32_t FLTR;  /**< FLTR Register >*/
};

/** corresponds to the address of i2c bus 1 */
#define I2C_BASE (struct i2c_reg_map*) 0x40005400 

/** peripheral clock frequency (16MHz) */
#define I2C_CLK_FRQ 0x10

/** enable i2c clock */
#define I2C_CLK_EN (1 << 21)

/** set TRISE register */
#define I2C_TRISE 0x11

/** enable i2c */
#define I2C_PE_EN (1 << 0)

/** enable START */
#define I2C_START_EN (1 << 8)

/** enable STOP */
#define I2C_STOP_EN (1 << 9)

/** enable ADDR */
#define I2C_ADDR_EN (1 << 1)

/** enable TxE */
#define I2C_TxE_EN (1 << 7)

/** enable BTF */
#define I2C_BTF_EN (1 << 2)

/** start bit */
#define I2C_SB (1 << 0)

/** acknowledge enable */
#define I2C_ACK_EN (1 << 10)

/**
 * @brief - initialize i2c protocol with the given clock
 * @param[in] clk clock value to be used to calculate value placed into CCR
 */

void i2c_master_init(uint16_t clk){
    
    // SDA pin B9 and SCL pin B8 based on images in doc and labels printed on board
    
    // init SCL pin B8
    gpio_init(GPIO_B, 8, MODE_ALT, OUTPUT_OPEN_DRAIN, OUTPUT_SPEED_LOW, PUPD_NONE, ALT4);   
    
    // init SDA pin B9
    gpio_init(GPIO_B, 9, MODE_ALT, OUTPUT_OPEN_DRAIN, OUTPUT_SPEED_LOW, PUPD_NONE, ALT4);

    // init clock
    struct rcc_reg_map *rcc = RCC_BASE;
    rcc->apb1_enr |= I2C_CLK_EN;

    // init relevant enables: check page 479
    struct i2c_reg_map *i2c = I2C_BASE;
    //set peripheral clock freq 16 Mhz
    i2c->CR2 |= I2C_CLK_FRQ;
    //set clock control registers 0x50
    i2c->CCR |= clk;
    //enable peripheral (CR1), NOTE: peripheral must be enabled last
    //i2c->CR1 |= I2C_ACK_EN;
    i2c->CR1 |= I2C_PE_EN;

    return;
}
/**
 * @brief - send i2c start bit. Used before sending data.
 * @param - None
 */
int i2c_master_start(){
    struct i2c_reg_map *i2c = I2C_BASE;

    //set start bit
    i2c->CR1 |= I2C_START_EN;
    
    //wait till EV5
    while (!(i2c->SR1 & I2C_SB)); 

    return 0;
}

/**
 * @brief - send i2c stop bit. Used after sending data.
 * @param - None
 */

int i2c_master_stop(){
    struct i2c_reg_map *i2c = I2C_BASE;
    
    //set stop bit
    i2c->CR1 |= I2C_STOP_EN;
    
    return 0;
}

/**
 * @brief - writes using i2c protocol
 * @param[in] buf buffer of data to be sent 
 * @param[in] len length of buffer to be sent
 * @param[in] slave_addr address of slave device
 * @return returns 0 when done
 */

int i2c_master_write(uint8_t *buf, uint16_t len, uint8_t slave_addr){
    i2c_master_start();

    struct i2c_reg_map *i2c = I2C_BASE;
    //slave_addr = slave_addr << 1;
    
    //send slave address
    i2c->DR = slave_addr;
   
    // wait for ev6
    while(!(i2c->SR1 & I2C_ADDR_EN));
    //here
    int x = i2c->SR2;
    (void) x;
    // wait for ev8_1;
    while(!(i2c->SR1 & I2C_TxE_EN));

    // send data
    for (int i = 0; i < len; i++) {
	 
        
        // send data
        i2c->DR = buf[i];

        // wait for ev8
 	    while (!(i2c->SR1 & I2C_TxE_EN));
    }
    
    //wait for EV8_2
    while( ((i2c->SR1 & I2C_TxE_EN) == 0) || ((i2c->SR1 & I2C_BTF_EN) == 0) );

    i2c_master_stop();
    
    return 0;
}

/**
 * @brief - reads using i2c protocol
 * @param[in] buf buffer where read data will be stored
 * @param[in] len length of buffer to be read
 * @param[in] slave_addr address of slave device
 */

int i2c_master_read(uint8_t *buf, uint16_t len, uint8_t slave_addr){
    (void) buf;
    (void) len;
    (void) slave_addr;

    return 0;
}
