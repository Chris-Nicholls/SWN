#ifndef STM32_MOCK_H
#define STM32_MOCK_H

#include <stdint.h>

typedef struct {
    volatile uint32_t BSRR;
    volatile uint32_t IDR;
} GPIO_TypeDef;

#define GPIOA ((GPIO_TypeDef *)0x40020000)
#define GPIOB ((GPIO_TypeDef *)0x40020400)
#define GPIOC ((GPIO_TypeDef *)0x40020800)
#define GPIOD ((GPIO_TypeDef *)0x40020C00)
#define GPIOE ((GPIO_TypeDef *)0x40021000)
#define GPIOF ((GPIO_TypeDef *)0x40021400)
#define GPIOG ((GPIO_TypeDef *)0x40021800)
#define GPIOH ((GPIO_TypeDef *)0x40021C00)

#define GPIO_PIN_0 0x0001
#define GPIO_PIN_1 0x0002
#define GPIO_PIN_2 0x0004
#define GPIO_PIN_3 0x0008
#define GPIO_PIN_4 0x0010
#define GPIO_PIN_5 0x0020
#define GPIO_PIN_6 0x0040
#define GPIO_PIN_7 0x0080
#define GPIO_PIN_8 0x0100
#define GPIO_PIN_9 0x0200
#define GPIO_PIN_10 0x0400
#define GPIO_PIN_11 0x0800
#define GPIO_PIN_12 0x1000
#define GPIO_PIN_13 0x2000
#define GPIO_PIN_14 0x4000
#define GPIO_PIN_15 0x8000

#endif
