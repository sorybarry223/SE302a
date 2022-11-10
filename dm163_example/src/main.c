#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/kernel.h>
#include "dm163.h"
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include<stdlib.h>

#define DM163_NODE DT_NODELABEL(dm163)
static const struct device *dm163_dev = DEVICE_DT_GET(DM163_NODE);

#define RGB_MATRIX_NODE DT_NODELABEL(rgb_matrix)

BUILD_ASSERT(DT_PROP_LEN(RGB_MATRIX_NODE, rows_gpios) == 8);

static const struct gpio_dt_spec rows[] = {
    GPIO_DT_SPEC_GET_BY_IDX(RGB_MATRIX_NODE, rows_gpios, 0),
    GPIO_DT_SPEC_GET_BY_IDX(RGB_MATRIX_NODE, rows_gpios, 1),
    GPIO_DT_SPEC_GET_BY_IDX(RGB_MATRIX_NODE, rows_gpios, 2),
    GPIO_DT_SPEC_GET_BY_IDX(RGB_MATRIX_NODE, rows_gpios, 3),
    GPIO_DT_SPEC_GET_BY_IDX(RGB_MATRIX_NODE, rows_gpios, 4),
    GPIO_DT_SPEC_GET_BY_IDX(RGB_MATRIX_NODE, rows_gpios, 5),
    GPIO_DT_SPEC_GET_BY_IDX(RGB_MATRIX_NODE, rows_gpios, 6),
    GPIO_DT_SPEC_GET_BY_IDX(RGB_MATRIX_NODE, rows_gpios, 7),
};

//Threads
#define STACKSIZE 500
#define MY_PRIORITY 5

//Defining a semaphore
K_SEM_DEFINE(my_sem, 0, 1);

#define NUM_LEDS 8
#define NUM_CHANNELS (NUM_LEDS * 3)
//uint8_t buffer[NUM_CHANNELS*NUM_LEDS];

struct {
  uint8_t channels[NUM_CHANNELS*NUM_LEDS];
}image;


//A callback for the timer
void timer_callback(struct k_timer* t) {
    k_sem_give(&my_sem);
}

K_TIMER_DEFINE(timer, timer_callback, NULL);

  
/********************* Everything about tilt 	angles before main****************/
#define LSM6DSL_NODE DT_NODELABEL(lsm6dsl)
static const struct i2c_dt_spec lsm6dsl  = I2C_DT_SPEC_GET(LSM6DSL_NODE);
static const struct gpio_dt_spec lsm6dsl_int = GPIO_DT_SPEC_GET(LSM6DSL_NODE,irq_gpios);

static struct gpio_callback lsm6dsl_int_cb;
static struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios,
						     {0});


#define PI 3.141592654
#define FULL_SCALE 250/32767

uint8_t adresses=0x28;
int8_t buffer[6] ={};
//Gyroscope output adresses
uint8_t g_adresses=0x22;
int8_t g_buffer[6]={};
//Mutex global variable
struct k_mutex my_mutex;

struct k_work work, my_work;
void callback_intrpt(const struct device *dev, struct gpio_callback *cb,
		    uint32_t pins)
{
	//k_work_schedule_for_queue(&my_work_q,dwork,K_SECONDS(1));
	k_work_submit(&work);
	k_work_submit(&my_work);
	printk("New messure done\n");
	
}
//This function allows me to compute the output results of the accelerometer 
//in 16-bit to double by first concatenating them in 32 bits
double calcul_axe(int8_t a, int8_t b){
	return (double) (a<<8 | b);
}

//This function computes the gyroscope output values concatenated in a 
//32 bits value. Not forgetting that the values of the gyro are in two's complement
double calcul_axe_gyro(int8_t a, int8_t b){
	return (double) (a<<8 | b);
}
//These functions allow me to conpute tilt axes

double phi_and_rho_funct(double ax,double ay,double az){
	return (atan(ax/sqrt(ay*ay + az*az))*180)/PI;
}

double tetha_funct(double ax,double ay,double az){
	return (atan(sqrt(ax*ax + ay*ay)/az)*180)/PI;
}


static void work_handler(struct k_work *work);
static void thread1(struct k_work *my_work);

//Tilt angles
double rho=0;
double phi=0;
double teta=0;
//Axes
double ax,ay,az;
//Old angular rates of gyroscope
double axg_o,ayg_o,azg_o;

//New angular rates of gyroscope
double axg_n,ayg_n,azg_n;

//Gyroscope tilt angles from delta
double tilt_x_delta, tilt_y_delta,tilt_z_delta;
//Gyroscope tilt angles
double tilt_x=0;
double tilt_y=0;
double tilt_z=0;
//Tilt angles with filter
static double filter_rho,filter_phi, filter_tetha;

//STATUS_REG variable
uint8_t STATUS_REG;
/********************* Everything about tilt 	angles before main****************/

void display(void){
int row=0;
    int last_row=row-1;
    if(row>0){
      last_row=7;}
  //gpio_pin_set_dt(&rows[row],0);
  dm163_turn_off_row(dm163_dev, rows, row);
  led_write_channels(dm163_dev,0,3*8,image.channels+8*3*row);
  gpio_pin_set_dt(&rows[row],1);
  row++;
  if(row==8){
    row=0;}
}  

int main() {
  
/********************* Everything about tilt angles in main****************/
int ret = 0;
	//Accelerometer
	uint8_t INT1_CTRL_addr=0x0d;
	uint8_t INT1_CTRL_value=0x01;
	//Control registers
	uint8_t CTRL1_XL_addr = 0x10;
	uint16_t CTRL1_XL=0b10111000;	//So the frequency is 1.6Hz
	uint8_t CTRL3_C_addr=0x12;
	//Gyroscope
	uint8_t CTRL2_G=0b00010000;	//Configuration of Gyroscope mode
	
	//Iniiating system workq
	k_work_init(&work,work_handler);
	k_work_init(&my_work,thread1);

	if (!device_is_ready(lsm6dsl_int.port)) {
		printk("Error: device %s is not ready\n",
		       lsm6dsl_int.port->name);
		return;
	}
	//Reseting the sensor
	i2c_reg_write_byte_dt(&lsm6dsl,CTRL3_C_addr,0x1);
	//Configure INT1_CTRL register so that an interuption is raised on INT1 pad
	i2c_reg_write_byte_dt(&lsm6dsl,INT1_CTRL_addr,INT1_CTRL_value);
	//Writing into control registers so that I can set the frequency to 1.6Hz and enable the accelerometer by the way
	i2c_reg_write_byte_dt(&lsm6dsl,CTRL1_XL_addr,CTRL1_XL);
	//The Gyroscope is activated
	i2c_reg_write_byte_dt(&lsm6dsl,0x11,CTRL2_G);
	
	ret = gpio_pin_configure_dt(&lsm6dsl_int, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n",
		       ret, lsm6dsl_int.port->name, lsm6dsl_int.pin);
		return;
	}

	gpio_init_callback(&lsm6dsl_int_cb, callback_intrpt, BIT(lsm6dsl_int.pin));
	gpio_add_callback(lsm6dsl_int.port, &lsm6dsl_int_cb);

/********************* Everything about tilt angles in main****************/ 

if (!device_is_ready(dm163_dev)) {
    return -ENODEV;
  }


  for (int row = 0; row < 8; row++)
    gpio_pin_configure_dt(&rows[row], GPIO_OUTPUT_INACTIVE);
  // Set brightness to 50% for all leds so that we don't become blind
  for (int i = 0; i < 8; i++)
    led_set_brightness(dm163_dev, i, 50);

  //Set the period for the timer
  k_timeout_t timer_period = K_MSEC(1000/(72 * 8));
  k_timer_start(&timer, timer_period,timer_period); 
  for (;;) {
i2c_burst_read_dt(&lsm6dsl,adresses,buffer,6);

  }
  return 0;
}

//The old time
uint32_t t_old=0;
static void work_handler(struct k_work *work)
{

	uint32_t t_new=0;
	uint32_t delta=0;
	k_mutex_init(&my_mutex);
  	k_mutex_lock(&my_mutex,K_MSEC(100));
	//Reading the accelerometer output registers
		i2c_burst_read_dt(&lsm6dsl,adresses,buffer,6);
		ax=  calcul_axe(buffer[1],buffer[0])*FULL_SCALE;
		ay=  calcul_axe(buffer[3],buffer[2])*FULL_SCALE;
		az=  calcul_axe(buffer[5],buffer[4])*FULL_SCALE;
		rho=phi_and_rho_funct(ax,ay,az);
		phi=phi_and_rho_funct(ay,ax,az);
		teta=tetha_funct(ax,ay,az);
		
		//Compute old angular rates of the gyroscope
		//Reading relevant data for the gyroscope
		i2c_reg_read_byte_dt(&lsm6dsl, 0x1E, &STATUS_REG);
		if(STATUS_REG==7 || STATUS_REG==6 || STATUS_REG==3 || STATUS_REG==2){
			i2c_burst_read_dt(&lsm6dsl,g_adresses,g_buffer,6);
			axg_o=  calcul_axe_gyro(g_buffer[1],g_buffer[0])*FULL_SCALE;
			ayg_o=  calcul_axe_gyro(g_buffer[3],g_buffer[2])*FULL_SCALE;
			azg_o=  calcul_axe_gyro(g_buffer[5],g_buffer[4])*FULL_SCALE;
		}
		//Get the current time then the difference with the old one
		t_new= k_uptime_get_32();
		delta= t_new - t_old;
		//Compute the angles from delta
		tilt_x_delta=axg_o*delta;
		tilt_y_delta=ayg_o*delta;
		tilt_z_delta=azg_o*delta;
		//Compute the angles
		tilt_x= (tilt_x + tilt_x_delta)/10;
		tilt_y= (tilt_y + tilt_y_delta)/10;
		tilt_z= (tilt_z + tilt_z_delta)/10;

		t_old=t_new;

		k_mutex_unlock(&my_mutex);

		printk("Rho = %.3f, Phi = %.3f, Tetha = %.3f\n", rho, phi,teta);
		printk("tilt x  = %.3f, tilt y  = %.3f, tilt z = %.3f\n", tilt_x,tilt_y,tilt_z);
}  

static void thread1(struct k_work *my_work){
	int ret=0;
	//Apply filter
	filter_rho   = 0.95*rho  + 0.05*tilt_x;
	filter_phi   = 0.95*phi  + 0.05*tilt_y;
	filter_tetha = 0.95*teta + 0.05*tilt_z;
	
	if (led.port && !device_is_ready(led.port)) {
		printk("Error %d: LED device %s is not ready; ignoring it\n",
		       ret, led.port->name);
		led.port = NULL;
	}
	gpio_pin_set_dt(&led,filter_tetha);
	
	/* refresh and display */
	for(int i=0;i<8;i++){
    for(int j=0;j<8;j++){
      image.channels[3*8*i + 3*j+0]=abs((int8_t) filter_rho);    //R
      image.channels[3*8*i + 3*j+1]=abs((int8_t) filter_rho);    //G
      image.channels[3*8*i + 3*j+2]=abs((int8_t) filter_rho);    //B
    }
  }
	display();
    
    for (int row = 0; row < 8; row++) {
      gpio_pin_set_dt(&rows[row], 1);
    }
    k_sem_take(&my_sem, K_FOREVER);


	printf("Filter rho = %.3f\n", filter_rho);
	printf("Filter phi = %.3f\n", filter_phi);
	printf("Filter tetha = %.3f\n", filter_tetha);
	k_sleep(K_MSEC(1000));
}
