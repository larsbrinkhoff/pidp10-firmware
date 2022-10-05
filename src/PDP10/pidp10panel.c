// PiDP10 (first prototype) has momentary switches soldered in upside down...
// PiDP10a (second prototype) has then put in the correct way.
// so first prototype's switch signal should be inverted, thus:
#define PIDP10
#ifdef PIDP10
#define MREVERSE !
#else
#define MREVERSE
#endif


// PiDP-10 front panel driver.
// based on https://raspberry-projects.com/pi/programming-in-c/i2c/using-the-i2c-interface
// based on PiDP-11 multiplexer, Joerg Hoppe & Oscar Vermeulen
//
// 20200305 OV First test

#include <stdio.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h> // extra
#include <time.h>				//Needed for nanosleep
#include <pthread.h>			//Needed for pthread
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <string.h>
#include <inttypes.h>
typedef uint64_t uint64;
#include <pidp10panel.h>

// the interface variables shared with other threads
volatile u_int16_t gpio_switchstatus[5] ; // bitfields: 5 rows of up to 16 switches
volatile u_int16_t gpio_ledstatus[8] ;	// bitfields: 8 ledrows of up to 16 LEDs


// for multiplexing, sleep time between row switches in ns (but excl. overhead!)
long intervl = 50000;	//	300000;		// light each row of leds this long
// for i2c stuff:
int file_i2c;							// i2c channel
unsigned char buffer[60] = {0};			// data exchange buffer for i2c use


#define BLOCK_SIZE 		(4*1024)
// IO Acces
struct bcm2835_peripheral {
    unsigned long addr_p;
    int mem_fd;
    void *map;
    volatile unsigned int *addr;
};
struct bcm2835_peripheral gpio; // needs initialisation
static unsigned bcm_host_get_peripheral_address(void); // find Pi's gpio base address
static unsigned get_dt_ranges(const char *filename, unsigned offset);
void short_wait(void);

// GPIO setup macros.
// In early versions INP_GPIO(x) was used always before OUT_GPIO(x),
// this is disabled now by INO_GPIO(g)
#define INO_GPIO(g) //INP_GPIO(g) // Use this before OUT_GPIO
#define INP_GPIO(g)   *(gpio.addr + ((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g)   *(gpio.addr + ((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio.addr + (((g)/10))) |= (((a)<=3?(a) + 4:(a)==4?3:2)<<(((g)%10)*3))
#define GPIO_SET  *(gpio.addr + 7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR  *(gpio.addr + 10) // clears bits which are 1 ignores bits which are 0
#define GPIO_READ(g)  *(gpio.addr + 13) &= (1<<(g))
#define GPIO_PULL *(gpio.addr + 37) // pull up/pull down
#define GPIO_PULLCLK0 *(gpio.addr + 38) // pull up/pull down clock
// Pi 4 update, 
/* https://github.com/RPi-Distro/raspi-gpio/blob/master/raspi-gpio.c */	
/* 2711 has a different mechanism for pin pull-up/down/enable  */
#define GPPUPPDN0                57        /* Pin pull-up/down for pins 15:0  */
#define GPPUPPDN1                58        /* Pin pull-up/down for pins 31:16 */
#define GPPUPPDN2                59        /* Pin pull-up/down for pins 47:32 */
#define GPPUPPDN3                60        /* Pin pull-up/down for pins 57:48 */


// GPIO pin definitions
//		8 ledrows each provide power to a block of 16 LEDs (when set to output high)
u_int8_t ledrows[8] = { 16,17,18,19, 20,21,22,23};
//		5 rows each provide a current sink to a block of 16 switches (when set to output low)
u_int8_t rows[5] = { 4,5,6,7,8 };
//		16 column pins in the MCP23017 provide a current sink to light up one of 16 LEDs (when set to output low)



// helper functions: 1 -- gpio
//
// map GPIO into virtual memory space ------------------------
int map_peripheral(struct bcm2835_peripheral *p)
{
	if ((p->mem_fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
		printf("Failed to open /dev/mem, try checking permissions.\n");
		return -1;
	}
	p->map = mmap(
	NULL, BLOCK_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, p->mem_fd, // File descriptor to physical memory virtual file '/dev/mem'
			p->addr_p); // Address in physical map that we want this memory block to expose
	if (p->map == MAP_FAILED) {
		perror("mmap");
		return -1;
	}
	p->addr = (volatile unsigned int *) p->map;
	return 0;
}

void unmap_peripheral(struct bcm2835_peripheral *p)
{
	munmap(p->map, BLOCK_SIZE);
	close(p->mem_fd);
}

static unsigned bcm_host_get_peripheral_address(void) // find Pi's gpio base address
{
// Pi 4 fix: https://github.com/raspberrypi/userland/blob/master/host_applications/linux/libs/bcm_host/bcm_host.c
   unsigned address = get_dt_ranges("/proc/device-tree/soc/ranges", 4);
   if (address == 0)
      address = get_dt_ranges("/proc/device-tree/soc/ranges", 8);
   return address == ~0 ? 0x20000000 : address;

}

static unsigned get_dt_ranges(const char *filename, unsigned offset)
{
	unsigned address = ~0;
	FILE *fp = fopen(filename, "rb");
	if (fp) {
		unsigned char buf[4];
		fseek(fp, offset, SEEK_SET);
		if (fread(buf, 1, sizeof buf, fp) == sizeof buf)
			address = buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3] << 0;
		fclose(fp);
	}
	return address;
}

void short_wait(void) // creates pause required in between clocked GPIO settings changes
{
	fflush(stdout); //
	usleep(1); // suggested as alternative for asm which c99 does not accept
}



// helper functions: 2 -- parallel threads for GPIO mux and LED brightness averaging
//
void *blink(void *ptr); // the real-time GPIO multiplexing process to start up
void *gpiopattern_update_leds(void *ptr); // the averaging thread
void gpio_mux_thread_start(void);

pthread_t blink_thread;
int blink_thread_terminate = 0;

void gpio_mux_thread_start()
{
    int res;
    res = pthread_create(&blink_thread, NULL, blink, &blink_thread_terminate);
    if (res) {
        fprintf(stderr, "Error creating gpio_mux thread, return code %d\n", res);
        exit(EXIT_FAILURE);
    }
    printf("Created blink_thread\n");
    sleep(2); // allow 2 sec for multiplex to start
}


// 3 - set MCP23017 'column' pins to input with pullup
static void mcp23017_to_input(void);
static void mcp23017_to_output(void);

static void mcp23017_to_input(void)
{
	// ----- set to 16 bits of input -----
	buffer[0] = 0x00; 	buffer[1] = 0xff; 	buffer[2] = 0xff;
	if (write(file_i2c, buffer, 3) != 3)
		printf("Failed to write to the i2c bus.\n");

	// ----- enable pullups -----
	buffer[0] = 0x0c;	buffer[1] = 0xff;	buffer[2] = 0xff;
	if (write(file_i2c, buffer, 3) != 3)
		printf("Failed to write to the i2c bus.\n");
}

static void mcp23017_to_output(void)
{	
	// ---- set to 16 bits of output -----
	buffer[0] = 0x00;	buffer[1] = 0x00;	buffer[2] = 0x00;
	if (write(file_i2c, buffer, 3) != 3)
		printf("Failed to write to the i2c bus.\n");
}


// A - the multiplexing thread 'blink'
//
void *blink(void *ptr)
{
	int	*terminate = (int *)ptr;
	int	phase, ledrow, row, i;

	// set thread to real time priority -----------------
	struct sched_param sp;
	sp.sched_priority = 98; // maybe 99, 32, 31?
	if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp))
		fprintf(stderr, "warning: failed to set RT priority\n");
		
	//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";
	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{	printf("Failed to open the i2c bus");	exit(1);		}
	if (ioctl(file_i2c, I2C_SLAVE, 0x20) < 0)
	{	printf("Failed bus access and/or talk to slave.\n");	exit(1);	}
	printf("OK: access to MCP\r\n");


	// Find gpio address (different for Pi 1,2 and 4) ----------
	gpio.addr_p = bcm_host_get_peripheral_address() + +0x200000;
	if (gpio.addr_p == 0x20200000) 		printf("*** RPi Plus detected\n");
	else if (gpio.addr_p == 0x3f000000) printf("*** RPi 2/3/Z detected\n");
	else if (gpio.addr_p == 0xfe200000) printf("*** RPi 4 detected\n");

	// map GPIO into virtual memory space ------------------------
	if (map_peripheral(&gpio) == -1) {
		printf("Failed to map the physical GPIO registers into the virtual memory space.\n");
		return (void *) -1;
	}

	// initialise GPIO (all pins used as inputs, with pull-ups enabled on cols)
	for (ledrow = 0; ledrow < 8; ledrow++) { // Define ledrows as input
		INP_GPIO(ledrows[ledrow]);
		GPIO_CLR = 1 << ledrows[ledrow]; // so go to Low when switched to output
	}
	mcp23017_to_input(); // Define cols as input with pullups

	// initialise GPIO pullups. Different for Pi 4 and older Pi's
	if (gpio.addr_p==0xfe200000)
	{
		//printf("Configuring pullups for Pi 4\r\n");
		/* https://github.com/RPi-Distro/raspi-gpio/blob/master/raspi-gpio.c */	
		/* 2711 has a different mechanism for pin pull-up/down/enable  */
		int gpiox;
		int pullreg;
		int pullshift;
		unsigned int pullbits;
		unsigned int pull;

		// GPIO column pins
		// -- already done during setup

		// GPIO row pins
		for (i=0;i<5;i++)
		{
			gpiox = rows[i];
			pullreg = GPPUPPDN0 + (gpiox>>4);
			pullshift = (gpiox & 0xf) << 1;
			pull = 0;	// pullup

			pullbits = *(gpio.addr + pullreg);
			pullbits &= ~(3 << pullshift);
			pullbits |= (pull << pullshift);
			*(gpio.addr + pullreg) = pullbits;
		}
		// GPIO ledrow pins
		for (i=0;i<8;i++)
		{
			gpiox = ledrows[i];
			pullreg = GPPUPPDN0 + (gpiox>>4);
			pullshift = (gpiox & 0xf) << 1;
			pull = 0;	// pullup

			pullbits = *(gpio.addr + pullreg);
			pullbits &= ~(3 << pullshift);
			pullbits |= (pull << pullshift);
			*(gpio.addr + pullreg) = pullbits;
		}
	}
	else 	// configure pullups for older Pis
	{
/*		// BCM2835 ARM Peripherals PDF p 101 & elinux.org/RPi_Low-level_peripherals#Internal_Pull-Ups_.26_Pull-Downs
		GPIO_PULL = 2; // pull-up
		short_wait(); // must wait 150 cycles
		GPIO_PULLCLK0 = 0x0c003ff0; // selects GPIO pins 4..13 and 26,27

		short_wait();
		GPIO_PULL = 0; // reset GPPUD register
		short_wait();
		GPIO_PULLCLK0 = 0; // remove clock
		short_wait(); // probably unnecessary
*/
		// BCM2835 ARM Peripherals PDF p 101 & elinux.org/RPi_Low-level_peripherals#Internal_Pull-Ups_.26_Pull-Downs
		GPIO_PULL = 0; // no pull-up no pull-down just float
		short_wait(); // must wait 150 cycles
		//GPIO_PULLCLK0 = 0x03f00000; // selects GPIO pins 20..25
		GPIO_PULLCLK0 = 0x0ff0000; // selects GPIO pins 16..23 (ledrow)
		short_wait();
		GPIO_PULL = 0; // reset GPPUD register
		short_wait();
		GPIO_PULLCLK0 = 0; // remove clock
		short_wait(); // probably unnecessary

		// BCM2835 ARM Peripherals PDF p 101 & elinux.org/RPi_Low-level_peripherals#Internal_Pull-Ups_.26_Pull-Downs
		GPIO_PULL = 0; // no pull-up no pull down just float
		short_wait(); // must wait 150 cycles
		//GPIO_PULLCLK0 = 0x070000; // selects GPIO pins 16..18
		GPIO_PULLCLK0 = 0x01f0; // selects GPIO pins 4..8 (row)
		short_wait();
		GPIO_PULL = 0; // reset GPPUD register
		short_wait();
		GPIO_PULLCLK0 = 0; // remove clock
		short_wait(); // probably unnecessary
	}
	// --------------------------------------------------

	// printf("\nPiDP-10 FP on\n");


	// start the actual multiplexing
		
	while (*terminate == 0)
	{	
		// ---- set to 16 bits of output -----
		buffer[0] = 0x00;	buffer[1] = 0x00;	buffer[2] = 0x00;
		if (write(file_i2c, buffer, 3) != 3)	printf("Failed i2c write (1).\n");

		for (phase=0; phase<6; phase++)		// 6 phases: 6 levels of brightness
		{
			for (ledrow=0; ledrow<8; ledrow++)	// 8 rows of LEDS get lit of for this phase
			{
				// ----- set MCP23017 IO pin values 
				//       (determines which of the 16 LEDs will light up) -----
				buffer[0] = 0x14;
				buffer[2] = gpio_ledstatus[ledrow] >> 8;
				buffer[1] = gpio_ledstatus[ledrow] & 0x00ff;
				if (write(file_i2c, buffer, 3) != 3)	printf("Failed i2c write (2)\n");

				// ----- set ledrow pin to high (switch on the power) -----
				INO_GPIO(ledrows[ledrow]);
				GPIO_SET = 1 << ledrows[ledrow]; // could be done once... TODO
				OUT_GPIO(ledrows[ledrow]);

				// ----- now wait a while with LEDs burning -----
				nanosleep((struct timespec[]) {	{	0, intervl}}, NULL);

				// ----- Toggle ledrow off (cut the power) -----
				GPIO_CLR = 1 << ledrows[ledrow];	// superfluous given next line
				INP_GPIO(ledrows[ledrow]);
			}
		}
		
		// ----- prepare for reading the switches
		//       note: INP_GPIO has been done already for the ledrows, 
		//       so they are inputs already, nothing to be done
		
		// ----- set MCP23017 IO pins to input with pullups enabled -----
		mcp23017_to_input();

		for (row=0; row<5; row++)		// there are 5 rows of 16 switches each
		{
			// ----- for this row, output 0V to overrule built-in pull-up 
			//       from column input pin -----
			INO_GPIO(rows[row]); 
			OUT_GPIO(rows[row]); 
			GPIO_CLR = 1 << rows[row]; 

			//nanosleep((struct timespec[]) { { 0, intervl / 100}}, NULL); 
			//// probably unnecessary long wait, maybe put above this loop also

			// ----- read switches (request MCP23017 gpio values) -----
			buffer[0] = 0x12;
			if (write(file_i2c, buffer, 1) != 1)	printf("Failed i2c write (3)\n");
			if (read(file_i2c, buffer, 2) != 2)		printf("Failed i2c read (3)\n");
			else
			{
				gpio_switchstatus[row] = (buffer[1] << 8) + buffer[0];
				//printf("Data read: %d %d\n", buffer[0], buffer[1]);
			}
			
			// stop sinking current from this row of switches
			INP_GPIO(rows[row]); 
		}
		
		// done with reading the switches, so start the next cycle of lighting up LEDs
	}
	
	// received terminate signal, close down
}

/*
// test main function for debugging standalone
int main(void)
{
	time_t start, end;
	double elapsed;  // seconds
	int i;
	
	printf("Start\n");
	
	// start up multiplexing thread
	gpio_mux_thread_start();
	//gpiopattern_start_thread();


	printf("Run for X seconds\n");
	start = time(NULL);
	int terminate = 1;
	while (1)	//(terminate) 
	{
		for (i=0; i<8; i++)
		{
			gpio_ledstatus[i] = (u_int16_t) (rand() << 8) + rand();
		}
		
		for (i=0; i<5; i++)
		{
//			if (gpio_switchstatus[i]!=65535)
//				printf("%d-%d | ", i, gpio_switchstatus[i]);
		}
//		printf("\n");

		end = time(NULL);
		elapsed = difftime(end, start);
//		if (elapsed >= 60.0 ) // 60 seconds
//			terminate = 0;
//		else  // No need to sleep when 90.0 seconds elapsed.
			usleep(50000);
	}
	

	printf("Terminate threads\n");
	blink_thread_terminate=1;
//	gpiopattern_thread_terminate=1;
	
	sleep (2);	// allow threads to close down
	printf("Done\n");
	return 0;
}
*/

// should include kx10defs.h, but for overview temporarily here:
typedef u_int64_t uint64;
typedef u_int64_t t_addr;

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

// global variables in kx10cpu.c, snooped at here
extern t_addr  PC;                                   /* Program counter */
extern uint32_t  IR;                                   /* Instruction register */
extern uint32_t  AC;                                   /* Operand accumulator */
extern t_addr  AB;                                   /* Memory address buffer */
extern uint64  AR;                                   /* Primary work register */
//extern uint64	MB;					/* Memory buffer *** maybe unnecessary *** */


// ==== functions to read out address and data switches
extern uint64  SW;                                   /* Switch register */
uint64 getPIDPSwitchRegister()
{
	// SR is row0[15..0] + row1[15..0] + row2[15..12]
	return		((uint64)gpio_switchstatus[0]<<20) 
			+ (gpio_switchstatus[1]<<4) 
 			+ ((gpio_switchstatus[2] & 0170000)>>12);
}
uint64 getPIDPAdressSwitches()
{
	// is row3[15..0] + row4[15..14]
	return 		(((uint64)gpio_switchstatus[3])<<2) 
			+ (((gpio_switchstatus[4])>>14) & 03);

}
// ===================



//void printBits(size_t const size, void const * const ptr);
u_int16_t getSWITCH(uint8_t row);
void injectCMD(char *pidpCMD);
void pidpCMD(char *cptr);

// string that contains a simh command to be injected into simh command line
char cmdBuffer[255];
// can be called to inject a command in simh command line.
// it will be picked up in scp.c's read_line() function, see code there
void injectCMD(char *pidpCMD)
{
	strcpy(cmdBuffer, pidpCMD);
	//printf("injected %s\r\n", cmdBuffer);
}
// used after command injection to forget command (and not inject it again)
void pidpCMD(char *cptr)
{
	strcpy (cptr, cmdBuffer);
	cmdBuffer[0]='\0';	// cmd is now sent to simh, done
	//printf("cptr is now %s\r\n", cptr);
}

// ====================================================================================
// functions to deal with reading the switches on the PIDP 
// ====================================================================================
// let simh access one of the 5 rows (of 16 switches each) on PIDP panel:
u_int16_t getSWITCH(uint8_t row)
{
	return(gpio_switchstatus[row]);
}
// let simh scp know CONT was pressed
int pidpCONT(void)
{
	if (MREVERSE(getSWITCH(4) & 020))	// CONT switch
		return 1;
	else
		return 0;
}
void dumpREG(void)
{
	printf("PC %llo   IR %llo   AC %llo  AB %llo   AR %llo \r\n", PC, IR, AC, AB, AR);
}



// ====================================================================================
// functions to update all the LEDs on the front panel from CPU registers in kx10_cpu.c
// ====================================================================================
// to avoid interference with multiplexing thread, update LEDs in temp buffer first
static u_int16_t temp_ledstatus[8] = { 0x00 };

void updatePC(int PC)
{
	int	i;
	
		temp_ledstatus[7] &= ~0xC000;
		temp_ledstatus[7] |= (u_int16_t) ((/*0123456*/PC & 0x03   ) << 14);	// LS 2 bits of PC on col15-14 ledrow 7
		temp_ledstatus[6] = (u_int16_t) ((/*0123456*/PC & 0x03fffc) >> 2);	// MS 16 bits on col 15-0 ledrow 6
		gpio_ledstatus[7]=temp_ledstatus[7]^0x0ffff;	// invert because 1 is led on, 0 led off
		gpio_ledstatus[6]=temp_ledstatus[6]^0x0ffff;
}

void updateIR(int IR)
{
		temp_ledstatus[4] &= ~0xc000;
		temp_ledstatus[3] = (u_int16_t) ((IR >> 9) & 0777 ) << 7;	// IR col 7-15 on ledrow 3
		temp_ledstatus[3] += (u_int16_t) (((IR >> 5) & 0x0f ) << 3);	// AC col 3-6 on ledrow 3
		temp_ledstatus[3] += (u_int16_t) (((IR >> 4) & 1) << 2);	// I bit col 2 on ledrow 3
		temp_ledstatus[3] += (u_int16_t) ((IR >> 2) & 0x03); 		// IDX col 1-0 on ledrow 3
		temp_ledstatus[4] |= (u_int16_t) ((IR & 0x03) << 14);		// -and col 15-14 on ledrow 4
		gpio_ledstatus[3]=temp_ledstatus[3]^0x0ffff;
		gpio_ledstatus[4]=temp_ledstatus[4]^0x0ffff;
}

void updateAB(int AB)
{
		temp_ledstatus[5] &= ~0xf000;
		temp_ledstatus[4] &= ~0x3fff;
		temp_ledstatus[5] |= (u_int16_t) ((AB & 0x0f) << 12);
		temp_ledstatus[4] |= (u_int16_t) ((AB & 0x03fff0) >> 4);
		gpio_ledstatus[4]=temp_ledstatus[4]^0x0ffff;
		gpio_ledstatus[5]=temp_ledstatus[5]^0x0ffff;
}

void updateMI(uint64 data)
{
		temp_ledstatus[2] &= ~0xf000;
		temp_ledstatus[2] |= (u_int16_t) ((data & 0x000000000f) << 12);
		temp_ledstatus[1] = (u_int16_t) ((data & 0x00000ffff0) >> 4);
		temp_ledstatus[0] = (u_int16_t) ((data & 0x0ffff00000) >> 20);
		gpio_ledstatus[0]=temp_ledstatus[0]^0x0ffff;
		gpio_ledstatus[1]=temp_ledstatus[1]^0x0ffff;
		gpio_ledstatus[2]=temp_ledstatus[2]^0x0ffff;
}

void updateMI_PROG(int data)
{
  		// row 2 led 11
		temp_ledstatus[2] &= ~(1 << 11);
		temp_ledstatus[2] |= (u_int16_t) ((data & 1) << 11);
		gpio_ledstatus[2]=temp_ledstatus[2]^0x0ffff;
}

void updateMI_MEM(int data)
{

// IMPORTANT REMINDER: V1 prototype has a
// HARDWARE BUG: P> (program data led) has same row/col as AR: led2col12...
// JUST DON'T FORGET, HAVE TO LIVE WITH IT FOR NOW
  		// row 2 led/column 12
		temp_ledstatus[2] &= ~(1 << 12);
		temp_ledstatus[2] |= (u_int16_t) ((data & 1) << 12);
		gpio_ledstatus[2]=temp_ledstatus[2]^0x0ffff;
}

void updateIOB_PIR(int data)
{
  // row 7, led 7-13
  data &= 0177;
  temp_ledstatus[7] &= ~(0177 << 7);
  temp_ledstatus[7] |= data << 7;
  gpio_ledstatus[7] = temp_ledstatus[7]^0x0ffff;
}

void updatePIR(int data)
{
  // row 5, led 0-6
  data &= 0177;
  temp_ledstatus[5] &= ~0177;
  temp_ledstatus[5] |= data;
  gpio_ledstatus[5] = temp_ledstatus[5]^0x0ffff;
}

void updatePIH(int data)
{
  // row 7, led 0-6
  data &= 0177;
  temp_ledstatus[7] &= ~0177;
  temp_ledstatus[7] |= data;
  gpio_ledstatus[7] = temp_ledstatus[7]^0x0ffff;
}

void updatePIE(int data)
{
  // row 2, 0-6
  data &= 0177;
  temp_ledstatus[2] &= ~0177;
  temp_ledstatus[2] |= data;
  gpio_ledstatus[2] = temp_ledstatus[2]^0x0ffff;
}

void updateRUN(int data)
{
  data &= 1;
  temp_ledstatus[2] &= ~(1 << 7);
  temp_ledstatus[2] |= data << 7;
  gpio_ledstatus[2] = temp_ledstatus[2]^0x0ffff;
  // row 2, 7
}

void updatePOWER(int data)
{
  data &= 1;
  temp_ledstatus[2] &= ~(1 << 9);
  temp_ledstatus[2] |= data << 9;
  gpio_ledstatus[2] = temp_ledstatus[2]^0x0ffff;
  // row 2, 9
}

void updatePION(int data)
{
  data &= 1;
  temp_ledstatus[2] &= ~(1 << 8);
  temp_ledstatus[2] |= data << 8;
  gpio_ledstatus[2] = temp_ledstatus[2]^0x0ffff;
  // row 2, 8
}

void updatePSTOP(int data)
{
  data &= 1;
  temp_ledstatus[5] &= ~(1 << 7);
  temp_ledstatus[5] |= data << 7;
  gpio_ledstatus[5] = temp_ledstatus[5]^0x0ffff;
  // row 5, 7
}

void updateMSTOP(int data)
{
  data &= 1;
  temp_ledstatus[5] &= ~(1 << 9);
  temp_ledstatus[5] |= data << 9;
  gpio_ledstatus[5] = temp_ledstatus[5]^0x0ffff;
  // row 5, 9
}

void updateUSER(int data)
{
  data = !!data;
  temp_ledstatus[5] &= ~(1 << 8);
  temp_ledstatus[5] |= data << 8;
  gpio_ledstatus[5] = temp_ledstatus[5]^0x0ffff;
  // row 5, 8
}
/*
void pidp10_lights_main (uint64 data)
{
    unsigned char buffer[8];
	static uint64 lights_main;

    lights_main = data;

    buffer[0] = (lights_main >> 32) & 0377;
    buffer[1] = (lights_main >> 24) & 0377;
    buffer[2] = (lights_main >> 16) & 0377;
    buffer[3] = (lights_main >> 8) & 0377;
    buffer[4] = lights_main & 0377;

	printf("%d %d %d %d %d\n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
	
//    buffer[5] = (lights_aux << 4) & 0340;

}
*/
/*
//assumes little endian
void printBits(size_t const size, void const * const ptr)
{
    unsigned char *b = (unsigned char*) ptr;
    unsigned char byte;
    int i, j;

    for (i=size-1;i>=0;i--)
    {
        for (j=7;j>=0;j--)
        {
            byte = (b[i] >> j) & 1;
            printf("%u", byte);
        }
    }
    puts("");
}
*/
