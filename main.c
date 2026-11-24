/**
 * Embedded Opgave 4 - Eksternt interrupt og display
 * Mikrocontroller: ATmega328PB Xplained Mini, CPU-frekvens = 16 MHz
 * UART0/UART1: 9600 baud, 8 databits, ingen paritet, 1 stopbit
 * OLED: SSD1306 128x64, I2C-adresse 0x3C, TWI0 paa PC4/PC5
 * Trykknap: PD2 (INT0) til GND, intern pull-up aktiveret
 */
#define F_CPU 16000000UL

#include "ssd1306.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include <util/atomic.h>

#define UART_BAUD_UBRR 103U
#define RX_SIZE                                                                \
  64U // Potens af 2, saa ringbufferens indeks kan beregnes med bitmaske
#define RX_MASK (RX_SIZE - 1U)
#define DEBOUNCE_MS 30UL
#define OLED_ADDRESS 0x3CU

// Modtagebuffere: ISR skriver tegn, mens hovedloekken laeser dem.
static volatile uint8_t uart0Rx[RX_SIZE];
static volatile uint8_t uart0Head = 0, uart0Tail = 0;
static volatile uint8_t uart1Rx[RX_SIZE];
static volatile uint8_t uart1Head = 0, uart1Tail = 0;

// Taellere og variabler til timere og det eksterne interrupt.
static volatile uint32_t milliseconds = 0;
static volatile uint32_t seconds = 0;
static volatile uint32_t edgeTime = 0;
static volatile uint8_t edgePending = 0;
static uint8_t expectPress = 1;
static uint32_t pressCount = 0;
static uint8_t oledAvailable = 0;

static void UART_Init(void) {
  UBRR0H = 0;
  UBRR0L = UART_BAUD_UBRR;
  UCSR0A = 0; // Normal UART-hastighed: U2X0 = 0
  UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
  UCSR0C =
      (1 << UCSZ01) | (1 << UCSZ00); // 8 databits, ingen paritet og 1 stopbit

  UBRR1H = 0;
  UBRR1L = UART_BAUD_UBRR;
  UCSR1A = 0;
  UCSR1B = (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1);
  UCSR1C =
      (1 << UCSZ11) | (1 << UCSZ10); // 8 databits, ingen paritet og 1 stopbit
}

static void UART0_Write(char c) {
  while (!(UCSR0A & (1 << UDRE0))) {
  }
  UDR0 = (uint8_t)c;
}

static void UART1_Write(char c) {
  while (!(UCSR1A & (1 << UDRE1))) {
  }
  UDR1 = (uint8_t)c;
}

static void UART0_Text(const char *s) {
  while (*s)
    UART0_Write(*s++);
}

static void UART1_Text(const char *s) {
  while (*s)
    UART1_Write(*s++);
}

// Omdan et 32-bit heltal uden fortegn til decimal tekst uden printf.
static void FormatU32(uint32_t value, char *out) {
  char reversed[10];
  uint8_t n = 0;
  do {
    reversed[n++] = (char)('0' + (value % 10UL));
    value /= 10UL;
  } while (value != 0);
  while (n > 0)
    *out++ = reversed[--n];
  *out = '\0';
}

static void PrintCount(void) {
  char number[11];
  FormatU32(pressCount, number);
  UART0_Text("INT0 tryk: ");
  UART0_Text(number);
  UART0_Text("\r\n");
  UART1_Text("INT0 tryk: ");
  UART1_Text(number);
  UART1_Text("\r\n");
}

// UART-modtageinterrupts: omdan bogstaver, foer de gemmes i bufferen.
ISR(USART0_RX_vect) {
  uint8_t c = UDR0;
  if (c >= 'a' && c <= 'z')
    c -= (uint8_t)('a' - 'A');
  uint8_t next = (uint8_t)((uart0Head + 1U) & RX_MASK);
  if (next != uart0Tail) {
    uart0Rx[uart0Head] = c;
    uart0Head = next;
  }
}

ISR(USART1_RX_vect) {
  uint8_t c = UDR1;
  if (c >= 'A' && c <= 'Z')
    c += (uint8_t)('a' - 'A');
  uint8_t next = (uint8_t)((uart1Head + 1U) & RX_MASK);
  if (next != uart1Tail) {
    uart1Rx[uart1Head] = c;
    uart1Head = next;
  }
}

static uint8_t UART0_Read(uint8_t *c) {
  if (uart0Head == uart0Tail)
    return 0;
  *c = uart0Rx[uart0Tail];
  uart0Tail = (uint8_t)((uart0Tail + 1U) & RX_MASK);
  return 1;
}

static uint8_t UART1_Read(uint8_t *c) {
  if (uart1Head == uart1Tail)
    return 0;
  *c = uart1Rx[uart1Tail];
  uart1Tail = (uint8_t)((uart1Tail + 1U) & RX_MASK);
  return 1;
}

static void Timers_Init(void) {
  // Timer0: interrupt hvert millisekund i CTC-tilstand (16 MHz / 64 / 250 =
  // 1000 Hz).
  TCCR0A = (1 << WGM01);
  TCCR0B = (1 << CS01) | (1 << CS00);
  OCR0A = 249;
  TIMSK0 = (1 << OCIE0A);

  // Timer1: interrupt hvert sekund i CTC-tilstand (16 MHz / 256 / 62500 = 1
  // Hz).
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS12);
  OCR1A = 62499;
  TIMSK1 = (1 << OCIE1A);
}

ISR(TIMER0_COMPA_vect) { milliseconds++; }

ISR(TIMER1_COMPA_vect) { seconds++; }

static void Button_Init(void) {
  DDRD &= ~(1 << DDD2);   // PD2 indstilles som indgang
  PORTD |= (1 << PORTD2); // Aktiver intern pull-up-modstand

  // INT0 udloeses foerst ved faldende flanke: ISC01:ISC00 = 10.
  EICRA = (uint8_t)((EICRA | (1 << ISC01)) & ~(1 << ISC00));
  EIFR = (1 << INTF0);  // Ryd eventuelt ventende interruptflag
  EIMSK |= (1 << INT0); // Aktiver eksternt interrupt INT0
}

ISR(INT0_vect) {
  // Deaktiver kun INT0 under kontaktprel; timer- og UART-interrupts
  // fortsaetter.
  EIMSK &= ~(1 << INT0);
  edgeTime = milliseconds;
  edgePending = 1;
}

static void Button_Process(void) {
  uint8_t pending;
  uint32_t start, now;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pending = edgePending;
    start = edgeTime;
    now = milliseconds;
  }
  if (!pending || (uint32_t)(now - start) < DEBOUNCE_MS)
    return;

  uint8_t pinHigh = (PIND & (1 << PIND2)) != 0;
  uint8_t wasPressed = 0;

  if (expectPress && !pinHigh) {
    // Bekraeftet tryk: tael en gang, men tael ikke naar knappen slippes.
    pressCount++;
    expectPress = 0;
    wasPressed = 1;
  } else if (!expectPress && pinHigh) {
    // Bekraeftet slip: goer klar til naeste tryk uden at oeg taelleren.
    expectPress = 1;
  }

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (expectPress) {
      // Faldende flanke: ISC01:ISC00 = 10
      EICRA = (uint8_t)((EICRA | (1 << ISC01)) & ~(1 << ISC00));
    } else {
      // Stigende flanke: ISC01:ISC00 = 11
      EICRA |= (1 << ISC01) | (1 << ISC00);
    }
    EIFR = (1 << INTF0); // Ryd interruptflag efter kontaktprel
    edgePending = 0;
    EIMSK |= (1 << INT0);
  }
  if (wasPressed)
    PrintCount();
}

// SSD1306-display: brug de medfoelgende drivere ssd1306.c og twi.c.
// Displayfunktionerne koerer kun fra hovedprogrammet, aldrig fra en ISR.
static void OLED_Init(void) {
  oledAvailable = (SSD1306_Init(OLED_ADDRESS) == SSD1306_SUCCESS);
  if (oledAvailable) {
    oledAvailable = (SSD1306_ClearScreen() == SSD1306_SUCCESS);
  }
}

static void OLED_ShowCount(void) {
  if (!oledAvailable)
    return;
  char number[11];
  FormatU32(pressCount, number);
  // Ryd tidligere tekst og vis derefter den aktuelle taeller.
  if (SSD1306_SetPosition(0, 0) != SSD1306_SUCCESS)
    return;
  SSD1306_DrawString("INT0 tryk:           ", NORMAL);
  if (SSD1306_SetPosition(0, 1) != SSD1306_SUCCESS)
    return;
  SSD1306_DrawString("                    ", NORMAL);
  SSD1306_SetPosition(0, 1);
  SSD1306_DrawString(number, NORMAL);
}

int main(void) {
  UART_Init();
  Timers_Init();
  Button_Init();
  OLED_Init();
  sei();

  UART0_Text("\r\nOpgave 4 klar - UART0 STORE bogstaver\r\n");
  UART1_Text("\r\nOpgave 4 klar - UART1 smaa bogstaver\r\n");
  if (oledAvailable) {
    OLED_ShowCount();
  } else {
    UART0_Text("OLED init fejl - tjek SDA/SCL og forsyning\r\n");
  }

  uint32_t lastSecondShown = 0;
  uint32_t lastCountShown = 0;
  uint8_t c;

  while (1) {
    // Bogstaverne er allerede omdannet i UART-modtageinterrupts.
    uint8_t budget = 16;
    while (budget-- && UART0_Read(&c))
      UART0_Write((char)c);
    budget = 16;
    while (budget-- && UART1_Read(&c))
      UART1_Write((char)c);

    Button_Process();
    if (pressCount != lastCountShown) {
      lastCountShown = pressCount;
      OLED_ShowCount();
    }

    uint32_t snapshot;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { snapshot = seconds; }
    while (lastSecondShown < snapshot) {
      char number[11];
      FormatU32(++lastSecondShown, number);
      UART1_Text("Sekunder efter opstart: ");
      UART1_Text(number);
      UART1_Text("\r\n");
    }
  }
}
