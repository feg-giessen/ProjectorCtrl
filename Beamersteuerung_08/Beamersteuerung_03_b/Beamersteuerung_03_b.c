/*
 * Beamersteuerung_03.cpp
 *
 * Created: 17.09.2013 17:24:39
 *  Author: Nils
 */ 

#define F_CPU 16000000  //16MHz   

#define BAUD 9600L		//9600 Baudrate


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/setbaud.h>
#include <avr/wdt.h>


// Berechnungen
// clever runden
#define UBRR_VAL  ((F_CPU+BAUD*8)/(BAUD*16)-1)
// Reale Baudrate
#define BAUD_REAL (F_CPU/(16*(UBRR_VAL+1)))
// Fehler in Promille
#define BAUD_ERROR ((BAUD_REAL*1000)/BAUD-1000)

#if ((BAUD_ERROR>10) || (BAUD_ERROR<-10))
#error Systematischer Fehler der Baudrate grösser 1% und damit zu hoch!
#endif



volatile uint8_t flag_1ms;

void uart_init(void)
{
	
	UBRRH = UBRR_VAL >> 8;
	UBRRL = UBRR_VAL & 0xFF;
	
	UCSRB |= (1<<TXEN);                        // UART TX einschalten
	UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);  // Asynchron 8N1
	UCSRB |= (1<<RXEN);                        // UART RX einschalten
	
}

uint8_t uart_getc(void)
{
	while (!(UCSRA & (1<<RXC)))   // warten bis Zeichen verfuegbar
	;
	return UDR;                   // Zeichen aus UDR an Aufrufer zurueckgeben
}

void uart_gets( char* Buffer, uint8_t MaxLen )
{
	cli(); // Interrupts abschalten
	flag_1ms=0;
	uint8_t NextChar;
	uint8_t StringLen = 0;
	
	NextChar = uart_getc();         // Warte auf und empfange das nächste Zeichen
	
	// Sammle solange Zeichen, bis:
	// * entweder das String Ende Zeichen kam // Beim Beamer Epson TW-Serie ist das Ende Zeichen ":" 
	// * oder das aufnehmende Array voll ist
	//while( NextChar != '\r' &&  NextChar != '\n' && NextChar != ':' && StringLen < MaxLen - 1 ) {
	while(  NextChar != ':' && StringLen < MaxLen - 1 ) {
		*Buffer++ = NextChar;
		StringLen++;
		NextChar = uart_getc();
	}
	
	// Noch ein '\0' anhängen um einen Standard
	// C-String daraus zu machen
	*Buffer = '\0';
	sei(); // Interrupts wieder einschalten
}

int uart_putc(unsigned char c)
{
	while (!(UCSRA & (1<<UDRE)))  /* warten bis Senden moeglich */
	{
	}
	UDR = c;                      /* sende Zeichen */
	return 0;
}

void uart_puts (char *s)
{
	while (*s)
	{   /* so lange *s != '\0' also ungleich dem "String-Endezeichen(Terminator)" */
		uart_putc(*s);
		s++;
	}
}

void Timer_io_init(void)
{
  // Table 39 im Atmega8 Datenblatt
  // Waveform generation mode: 
  // Mode 9: PWM Phase and frequency correct
  TCCR1A |= (0 << WGM11) | (1 << WGM10);
  TCCR1B |= (1 << WGM13) | (0 << WGM12);
 
  // Table 40 im Atmega8 Datenblatt
  // Prescaler: clk/256
  TCCR1B |= (1 << CS12)  | (0 << CS11)  | (0 << CS10);
 
  /*
    Wunsch: ca. 1 Hz Blinken
    1 Hz genau - siehe: http://www.mikrocontroller.net/articles/AVR_-_Die_genaue_Sekunde_/_RTC
 
    Setzen von TOP (OCR1A) = Anzahl der Timer1-Takte von IRQ zu IRQ
    Figure 40 im Atmega8 Datenblatt
    1. /2 wegen 2 Halbperioden pro Sekunde bei 1 Hz
    2. /2 wegen Runterzählen von TOP (=OCR1A) nach BOTTOM (=0) 
       und Hochzählen von BOTTOM (=0) bis TOP (=OCR1A) zwischen 
       den Interrupts
  */
 
#define DELAY (3125)//F_CPU /256 /2 / 10 = 0,1s
  OCR1A = DELAY; 
  TCNT1 = - DELAY; // Anlauf für 1. Interrupt verlängern
 
  TIMSK = (1 << OCIE1A);
}

int main(void)
{
	 // IOs initialisieren
	 	  
		DDRB = (0 << 0) | (0 << 1) | (0 << 2) | (0 << 3) | (0 << 4);
		PORTB  |= (1 << 0); // PB0 Pullup aktivieren
		PORTB  |= (1 << 1); // PB1 Pullup aktivieren
		PORTB  |= (1 << 2); // PB2 Pullup aktivieren
		PORTB  |= (1 << 3); // PB3 Pullup aktivieren
		PORTB  |= (1 << 4); // PB4 Pullup aktivieren
		DDRC = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4);
		
		//Source LEDs aufblinken lassen
	 PORTC ^= 0x1C; 
	 _delay_ms(100);
	 PORTC ^= 0x1C;
	
	 // UART initialisieren
	 uart_init();
	 // Timer initialisieren
	 Timer_io_init();
	 // Interrupts global freigeben
	 sei();
	 
	 
	 uint8_t letzter_tasten_status; // letzter Status
	 uint8_t aktuelle_tasten_veraenderung; // sinkende Flanke
	 letzter_tasten_status = 0;
	 aktuelle_tasten_veraenderung = 0;
	 
	 uint8_t x; // Zähler Variable
	 x = 0;
	 
	 char Line[12]; // String für UART-Empfang
	 
	 // Endlose Hauptschleife
    while(1)
    {
        
		if ((UCSRA & (1<<RXC))) //Empfangsdaten vorhanden
		{
			uart_gets(Line,sizeof(Line));
			if (strcmp(Line , "PWR=02\r") == 0) // Power = ON
			{
				PORTC |= (1<<PC1); // Power ON  LED an
				PORTC &= ~(1<<PC0);// Power OFF LED aus
			}
			if (strcmp(Line , "PWR=00\r") == 0) // Power = OFF
			{
				PORTC |= (1<<PC0); // Power OFF LED an
				PORTC &= ~(1<<PC1);// Power ON  LED aus
				
				// Source LEDs aus
				PORTC &= ~(1<<PC2);
				PORTC &= ~(1<<PC3);
				PORTC &= ~(1<<PC4);
			}
			if (strcmp(Line , "SOURCE=30\r") == 0) // Source = HDMI
			{
				// Source HDMI LED an rest aus
				PORTC |= (1<<PC2);
				PORTC &= ~(1<<PC3);
				PORTC &= ~(1<<PC4);
			}
			if (strcmp(Line , "SOURCE=21\r") == 0) // Source = VGA
			{
				// Source VGA LED an rest aus
				PORTC |= (1<<PC3);
				PORTC &= ~(1<<PC2);
				PORTC &= ~(1<<PC4);
			}
			if (strcmp(Line , "SOURCE=41\r") == 0) // Source = Video
			{
				// Source Video LED an rest aus
				PORTC |= (1<<PC4);
				PORTC &= ~(1<<PC2);
				PORTC &= ~(1<<PC3);
			}
		}
		
		if (flag_1ms) { // flag_1ms wird per Interrupt alle 100ms auf true gesetzt
					flag_1ms=0; // zurücksetzen
					if (x == 4) //abwechselnd Zustand AN/AUS und Quelle abfragen
					{
						uart_puts("PWR?\r"); // Power abfragen (1x pro Sekunde)
					}
					if (x == 8 && (PORTC & (1<<PC1)) ) // Source nur abfragen wenn Power = ON
					{
						uart_puts("SOURCE?\r"); // Source abfragen (wenn aktiv 1x pro Sekunde)
					}
					if (x>9){
						x = 0; // bei 10 zurücksetzen
					}
					x++; // hochzählen
					
					aktuelle_tasten_veraenderung = (PINB ^ letzter_tasten_status) & (~PINB); // fallende Flanken erkennen (aktiv LOW)
					if (aktuelle_tasten_veraenderung & (1 << PINB0)){ //OFF
						PORTC |= (1<<PC0); // Power OFF LED an
						PORTC &= ~(1<<PC1);// Power ON  LED aus
						
						// Source LEDs aus
						PORTC &= ~(1<<PC2);
						PORTC &= ~(1<<PC3);
						PORTC &= ~(1<<PC4);
						
						uart_puts("PWR OFF\r"); // Den Beamer ausschalten
					}
					if (aktuelle_tasten_veraenderung & (1 << PINB1)){ //ON
						PORTC |= (1<<PC1); // Power ON  LED an
						PORTC &= ~(1<<PC0);// Power OFF LED aus
						uart_puts("PWR ON\r"); // Den Beamer einschalten
					}					
					if (aktuelle_tasten_veraenderung & (1 << PINB2)  && (PORTC & (1<<PC1)) ){ //Source1 = HDMI
						// Source HDMI LED an rest aus
						PORTC |= (1<<PC2);
						PORTC &= ~(1<<PC3);
						PORTC &= ~(1<<PC4);
						uart_puts("SOURCE 30\r"); // Beamerquelle auf HDMI stellen
					}
					if (aktuelle_tasten_veraenderung & (1 << PINB3)  && (PORTC & (1<<PC1)) ){ //Source2 = VGA
						// Source VGA LED an rest aus
						PORTC |= (1<<PC3);
						PORTC &= ~(1<<PC2);
						PORTC &= ~(1<<PC4);
						uart_puts("SOURCE 21\r"); // Beamerquelle auf VGA stellen
					}
					if (aktuelle_tasten_veraenderung & (1 << PINB4)  && (PORTC & (1<<PC1)) ){ //Source3 = Video
						// Source Video LED an rest aus
						PORTC |= (1<<PC4);
						PORTC &= ~(1<<PC2);
						PORTC &= ~(1<<PC3);
						uart_puts("SOURCE 41\r"); // Beamerquelle auf Video stellen
					}
					letzter_tasten_status = PINB; //Letzten Status merken
															
					if (flag_1ms) { // wenn der flag_1ms wieder aktiv ist sind über 100ms vergangen
							// Laufzeit der Tasks >100ms, Fehlersignalisierung
							// PC4 auf HIGH, Programm stoppen
							PORTC |= (1<<PC4);
							wdt_enable(7); // Watchdog zum automatischen Reset aktivieren
							while(1);
					}								
		}
    }
}

ISR (TIMER1_COMPA_vect)
{
	if (flag_1ms) { // wenn der flag_1ms noch aktiv ist sind über 200ms vergangen
		// Laufzeit der Tasks >200ms, Fehlersignalisierung
		// PC4 auf HIGH, Programm stoppen
		PORTC |= (1<<PC4);
		wdt_enable(7); // Watchdog zum automatischen Reset aktivieren
		while(1);
	}
	flag_1ms = 1;
}
