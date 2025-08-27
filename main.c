#include <inttypes.h>
#include <stdbool.h>

//Represent the gpio in memory, broken up in 32bit blocks
struct gpio {
    volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
};
//RCC in memory
struct rcc {
  volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR,
      RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, AHB3ENR,
      RESERVED2, APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR,
      AHB3LPENR, RESERVED4, APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR,
      RESERVED6[2], SSCGR, PLLI2SCFGR;
};
//GPIO Modes to set in gpio->MODER. 
//                 00                01            10                11
enum {GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG};

//Defining memory
//Gets start of selected GPIO bank
#define GPIO(bank) ((struct gpio *) (0x40020000 + 0x400 * (bank)))
//Start of RCC
#define RCC ((struct rcc *) 0x40023800)

//Helper to shift bits
#define BIT(x) (1UL << (x))
//Gets pin where first 8 bits represent bank and last 8 represent pin number
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
//Pulls pin number out of a pin
#define PINNO(pin) (pin & 255)
//Pulls bank number out of a pin
#define PINBANK(pin) (pin >> 8)
//Example for above types
// PIN('A', 3) gets 0000 0101 0000 1011
//             'G' - 'A' =  3        11

//Inline helper functions
//Given a pin defined by PIN, and a mode defined by 
static inline void gpio_set_mode(uint16_t pin, uint8_t mode) {
  struct gpio *gpio = GPIO(PINBANK(pin)); // GPIO bank
  int n = PINNO(pin);                 // Pin number
  gpio->MODER &= ~(3U << (n * 2));        // Clear existing setting
  gpio->MODER |= (mode & 3) << (n * 2);   // Set new mode
}

//Writes to GPIO of given pin
static inline void gpio_write(uint16_t pin, bool val) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  gpio->BSRR = (1U << PINNO(pin)) << (val ? 0 : 16);
}

//Delete this eventually
static inline void spin(volatile uint32_t count) {
  while (count--) asm("nop");
}

int main(void) {
  uint16_t pin = PIN('B', 7);
  RCC->AHB1ENR |= BIT(PINBANK(pin));
  gpio_set_mode(pin, GPIO_MODE_OUTPUT);  // Set blue LED to output mode
  for (;;) {
    gpio_write(pin, true);
    spin(999999);
    gpio_write(pin, false);
    spin(999999);
  }
  return 0;
}

// Startup code
__attribute__((naked, noreturn)) void _reset(void) {
  // memset .bss to zero, and copy .data section to RAM region
  extern long _sbss, _ebss, _sdata, _edata, _sidata;
  for (long *dst = &_sbss; dst < &_ebss; dst++) *dst = 0;
  for (long *dst = &_sdata, *src = &_sidata; dst < &_edata;) *dst++ = *src++;

  main();             // Call main()
  for (;;) (void) 0;  // Infinite loop in the case if main() returns
}