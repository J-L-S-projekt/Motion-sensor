;***************************************************************************
;*
;* Title: target_state
;* Author: Joshua Sue and Farhan Chowdhury
;* Version: 1.1
;* Last updated: November 17, 2026
;* Target: AVR128DB48 Curiosity Nano
;*
;* DESCRIPTION
;* Extension of hlk_ld2440c_incoming_data (Design Task 3).
;* Reads the auto-report data stream from the HLK-LD2410C human presence
;* sensor on USART0 (ALT1 pins: PA4 = TxD, PA5 = RxD) at 256000 baud, 8N1.
;* The program synchronizes to the frame header 0xF4 0xF3 0xF2 0xF1 and
;* stores one 23-byte frame into a linear buffer in SRAM. After each full
;* frame, the target state byte (buffer index 8) is extracted, the LED on
;* PA3 is updated (ON for 0x01, 0x02, 0x03, OFF for 0x00), and the target
;* state is shown on the SparkFun 20x4 SerLCD. The SerLCD is driven by
;* USART3 (PB0 = TxD) at 9600 baud, 8N1, as in Laboratory 07. The LCD is
;* only rewritten when the target state changes, to avoid flicker.
;* LCD example:
;*     line 1:  Target State: 0x02
;*     line 2:  Stationary target
;*
;* Wiring:
;*   LD2410C TX  -> PA5 (USART0 RxD, ALT1)
;*   LD2410C RX  -> PA4 (USART0 TxD, ALT1), not used in this program
;*   PB0 (USART3 TxD) -> SerLCD RX
;*   PA3 -> resistor -> yellow LED -> GND (active high)
;*   LD2410C OUT -> 330 ohm -> green LED -> GND (Task 1 circuit)
;*   LD2410C VCC -> VBUS (5 V), LD2410C GND -> GND, SerLCD GND -> GND
;*
;* VERSION HISTORY
;* 1.0 Original version
;* 1.1 Display changed from the course LCD driver to the SerLCD on USART3
;***************************************************************************
.nolist
.include "avr128DB48def.inc"            ; register and bit definitions for the AVR128DB48
.list

;---------------------------- constants ------------------------------------
.equ F_CPU        = 4000000             ; clock frequency after reset: 4 MHz
.equ SENSOR_BAUD  = 256000              ; HLK-LD2410C default baud rate
.equ SENSOR_BAUD_VAL = (64*F_CPU + 4*SENSOR_BAUD) / (8*SENSOR_BAUD) ; double-speed mode: BAUD = 64*fclk/(8*baud) = 125
.equ LCD_BAUD_VAL = 1667                ; SerLCD at 9600 baud: 64*4,000,000/(16*9600) = 1666.67, rounded (0x0683)
.equ FRAME_LEN    = 23                  ; number of bytes in one reported data frame
.equ STATE_INDEX  = 8                   ; buffer index of the target state byte
.equ NO_STATE     = 0xFF                ; last_state value meaning "nothing displayed yet"
.equ PA3_MASK     = 0b00001000          ; bit mask for pin 3 (yellow LED)
.equ PA4_MASK     = 0b00010000          ; bit mask for pin 4 (USART0 TxD on ALT1)
.equ PA5_MASK     = 0b00100000          ; bit mask for pin 5 (USART0 RxD on ALT1)
.equ ROUTE_ALT1   = 0b00000001          ; PORTMUX_USARTROUTEA code: USART0 on ALT1 pins
.equ FRAME_8N1    = 0b00000011          ; CTRLC: asynchronous, no parity, 1 stop bit, 8 data bits
.equ RXEN_MASK    = 0b10000000          ; CTRLB bit 7: receiver enable
.equ RXMODE_2X    = 0b00000010          ; CTRLB RXMODE field: double-speed (CLK2X) mode
.equ TXEN_MASK    = 0b01000000          ; CTRLB bit 6: transmitter enable
.equ RXCIF_BIT    = 7                   ; STATUS bit 7: receive complete flag
.equ DREIF_BIT    = 5                   ; STATUS bit 5: data register empty flag
.equ ASCII_0      = 48                  ; ASCII code for the character '0'
.equ ASCII_A_10   = 55                  ; ASCII code for 'A' (65) minus 10, used for hex digits A to F

;---------------------------- register names to make my life easier -------------------------------
.def temp       = r16                   ; scratch register, received byte, and character to send
.def nib        = r19                   ; nibble to convert, then its ASCII character
.def last_state = r21                   ; target state currently shown on the LCD
.def index      = r22                   ; number of bytes stored in the buffer so far
.def expect     = r23                   ; expected header byte while searching for the header
.def state      = r24                   ; target state value taken from the buffer

;---------------------------- data memory ----------------------------------
.dseg                                   ; switch to the data (SRAM) segment
ld2410c_buffer: .byte FRAME_LEN         ; 23-byte linear buffer, starts at 0x4000

;---------------------------- program memory -------------------------------
.cseg                                   ; switch to the code (flash) segment
.org 0x0000                             ; reset vector address
    rjmp main                           ; jump to the start of the program

main:                                   ; program entry point
    cli                                 ; no interrupts are used

    rcall init_led                      ; make PA3 an output with the LED off
    rcall init_usart0                   ; set up USART0 for the sensor (ALT1 pins, 256000 baud, 8N1)
    rcall init_usart3                   ; set up USART3 for the SerLCD (PB0, 9600 baud, 8N1)

    rcall delay_500ms                   ; give the SerLCD time to finish its power-up
    rcall delay_500ms                   ; and splash screen before sending anything

    ldi last_state, NO_STATE            ; nothing is displayed yet, so the first frame will be shown
    clr index                           ; index = 0, no bytes stored yet
    ldi XH, high(ld2410c_buffer)        ; X high byte = high byte of the buffer address
    ldi XL, low(ld2410c_buffer)         ; X low byte = low byte of the buffer address

wait_rx:                                ; wait here for the next received byte
    lds temp, USART0_STATUS             ; read the USART0 status register
    sbrs temp, RXCIF_BIT                ; skip the next instruction if a byte has been received
    rjmp wait_rx                        ; no byte yet, keep waiting
    lds temp, USART0_RXDATAL            ; temp = received byte (also clears RXCIF)

    cpi index, 4                        ; have the 4 header bytes been found yet?
    brsh store_byte                     ; yes (index >= 4), just store this byte

    ldi expect, 0xF4                    ; header bytes are F4, F3, F2, F1 so start from 0xF4
    sub expect, index                   ; expect = 0xF4 - index = the header byte we want next
    cp temp, expect                     ; is the received byte the expected header byte?
    breq store_byte                     ; yes, store it and move on

    clr index                           ; header mismatch, restart the search at index 0
    ldi XH, high(ld2410c_buffer)        ; reset the pointer high byte to the buffer start
    ldi XL, low(ld2410c_buffer)         ; reset the pointer low byte to the buffer start
    cpi temp, 0xF4                      ; could this byte be the start of a new header?
    brne wait_rx                        ; no, throw it away and wait for the next byte
                                        ; yes, fall through and store it as byte 0

store_byte:                             ; store the received byte in the buffer
    st X+, temp                         ; buffer[index] = byte, then advance the pointer
    inc index                           ; one more byte stored
    cpi index, FRAME_LEN                ; is the whole 23-byte frame stored?
    brne wait_rx                        ; not yet, wait for the next byte

    rcall update_led                    ; set the PA3 LED from the target state in the buffer
    rcall display_state                 ; show the target state on the SerLCD if it changed

    clr index                           ; frame done, get ready to sync on the next frame
    ldi XH, high(ld2410c_buffer)        ; reset the pointer high byte to the buffer start
    ldi XL, low(ld2410c_buffer)         ; reset the pointer low byte to the buffer start

frame_done:                             ; a full frame is in the buffer and shown on the LCD
    nop                                 ; <-- put your breakpoint on this line
    rjmp wait_rx                        ; go wait for the next frame

;***************************************************************************
;* 
;* "init_led" - Initialize the PA3 presence LED
;*
;* Description:
;* Makes PA3 an output and drives it low so the LED starts OFF.
;*
;* Author: Joshua Sue and Farhan Chowdhury
;* Version: 1.0
;* Last updated: November 16, 2026
;* Target: AVR128DB48
;* Number of words: 6
;* Number of cycles: 9 (estimate, not counting the rcall)
;* Low registers modified: none
;* High registers modified: r16 (temp)
;*
;* Parameters: none
;*
;* Returns: none
;*
;* Notes: The LED is assumed to be active high (PA3 high = LED on).
;*
;***************************************************************************
init_led:                               ; start of init_led subroutine
    ldi temp, PA3_MASK                  ; temp = bit mask for pin 3
    sts PORTA_DIRSET, temp              ; make PA3 an output
    sts PORTA_OUTCLR, temp              ; drive PA3 low so the LED is off
    ret                                 ; return to the caller

;***************************************************************************
;* 
;* "init_usart0" - Initialize USART0 for the HLK-LD2410C
;*
;* Description:
;* Routes USART0 to the ALT1 pins (PA4 = TxD, PA5 = RxD), sets PA4 as an
;* output and PA5 as an input, loads the baud rate for 256000 baud, selects
;* asynchronous 8N1 frames, and enables the receiver in double-speed mode.
;*
;* Author: Joshua Sue and Farhan Chowdhury
;* Version: 1.1
;* Last updated: November 17, 2026
;* Target: AVR128DB48
;* Number of words: 22
;* Number of cycles: 25 (estimate, not counting the rcall)
;* Low registers modified: none
;* High registers modified: r16 (temp)
;*
;* Parameters: none
;*
;* Returns: none
;*
;* Notes: Double-speed mode samples 8 times per bit instead of 16, which
;* is needed to reach 256000 baud from the 4 MHz clock (normal mode tops
;* out at 250000 baud). SENSOR_BAUD_VAL = 125 for this mode. The default
;* USART0 pins (PA0, PA1) cannot be used because they connect to the
;* Curiosity Nano's on-board crystal.
;*
;***************************************************************************
init_usart0:                            ; start of init_usart0 subroutine
    ldi temp, ROUTE_ALT1                ; temp = routing code for USART0 on ALT1 pins
    sts PORTMUX_USARTROUTEA, temp       ; route USART0 to PA4 (TxD) and PA5 (RxD)

    ldi temp, PA4_MASK                  ; temp = bit mask for pin 4
    sts PORTA_DIRSET, temp              ; make PA4 (TxD) an output
    ldi temp, PA5_MASK                  ; temp = bit mask for pin 5
    sts PORTA_DIRCLR, temp              ; make PA5 (RxD) an input

    ldi temp, low(SENSOR_BAUD_VAL)      ; temp = low byte of the baud rate value
    sts USART0_BAUDL, temp              ; write the baud rate low byte
    ldi temp, high(SENSOR_BAUD_VAL)     ; temp = high byte of the baud rate value
    sts USART0_BAUDH, temp              ; write the baud rate high byte

    ldi temp, FRAME_8N1                 ; temp = asynchronous, no parity, 1 stop bit, 8 data bits
    sts USART0_CTRLC, temp              ; set the 8N1 frame format

    ldi temp, RXEN_MASK | RXMODE_2X     ; temp = receiver enable and double-speed mode
    sts USART0_CTRLB, temp              ; enable the receiver (transmitter stays off)
    ret                                 ; return to the caller

;***************************************************************************
;* 
;* "init_usart3" - Initialize USART3 for the SerLCD
;*
;* Description:
;* Configures USART3 as an asynchronous 8N1 transmitter at 9600 baud.
;* PB0 is the default TxD pin for USART3. The receiver is not enabled.
;*
;* Author: Joshua Sue and Farhan Chowdhury
;* Version: 1.0
;* Last updated: November 17, 2026
;* Target: AVR128DB48
;* Number of words: 15
;* Number of cycles: 18 (estimate, not counting the rcall)
;* Low registers modified: none
;* High registers modified: r16 (temp)
;*
;* Parameters: none
;*
;* Returns: none
;*
;* Notes: Taken from Laboratory 07 (USART_write_message). LCD_BAUD_VAL =
;* 1667 gives about 9598 baud with a 4 MHz clock.
;*
;***************************************************************************
init_usart3:                            ; start of init_usart3 subroutine
    sbi VPORTB_OUT, 0                   ; make sure PB0 idles high
    sbi VPORTB_DIR, 0                   ; PB0 (TxD) must be an output
    ldi temp, low(LCD_BAUD_VAL)         ; temp = low byte of the baud rate value
    sts USART3_BAUDL, temp              ; write the low byte first
    ldi temp, high(LCD_BAUD_VAL)        ; temp = high byte of the baud rate value
    sts USART3_BAUDH, temp              ; writing the high byte completes the update
    ldi temp, FRAME_8N1                 ; temp = asynchronous, no parity, 1 stop bit, 8 data bits
    sts USART3_CTRLC, temp              ; set the 8N1 frame format
    ldi temp, TXEN_MASK                 ; temp = transmitter enable bit
    sts USART3_CTRLB, temp              ; enable the transmitter only
    ret                                 ; return to the caller

;***************************************************************************
;* 
;* "update_led" - Update the presence LED from the target state
;*
;* Description:
;* Reads the target state byte (index 8) from ld2410c_buffer. If the state
;* is 0x01, 0x02, or 0x03 the LED on PA3 is turned ON. If it is 0x00 or
;* 0x04 and above, the LED is turned OFF.
;*
;* Author: Joshua Sue and Farhan Chowdhury
;* Version: 1.0
;* Last updated: November 16, 2026
;* Target: AVR128DB48
;* Number of words: 14
;* Number of cycles: 13 to 15 (estimate, not counting the rcall)
;* Low registers modified: none
;* High registers modified: r16 (temp)
;*
;* Parameters: ld2410c_buffer[STATE_INDEX] in SRAM holds the target state.
;*
;* Returns: none
;*
;* Notes: States 0x04 to 0x06 only occur during background noise detection,
;* so they are treated as no presence.
;*
;***************************************************************************
update_led:                             ; start of update_led subroutine
    lds temp, ld2410c_buffer + STATE_INDEX ; temp = target state value from the buffer
    tst temp                            ; is the target state 0x00 (no target)?
    breq led_off                        ; yes, turn the LED off
    cpi temp, 0x04                      ; is the target state 0x04 or higher?
    brsh led_off                        ; yes, not a presence value, turn the LED off

led_on:                                 ; target state is 0x01, 0x02, or 0x03
    ldi temp, PA3_MASK                  ; temp = bit mask for pin 3
    sts PORTA_OUTSET, temp              ; drive PA3 high so the LED is on
    ret                                 ; return to the caller

led_off:                                ; no one is present
    ldi temp, PA3_MASK                  ; temp = bit mask for pin 3
    sts PORTA_OUTCLR, temp              ; drive PA3 low so the LED is off
    ret                                 ; return to the caller

;***************************************************************************
;* 
;* "display_state" - Show the target state on the SerLCD
;*
;* Description:
;* Extracts the target state byte (index 8) from ld2410c_buffer. If it is
;* different from the state already on the LCD, the screen is cleared and
;* rewritten: line 1 = "Target State: 0xNN" and line 2 = a short text
;* describing the state. If the state is unchanged, nothing is sent.
;*
;* Author: Joshua Sue and Farhan Chowdhury
;* Version: 1.0
;* Last updated: November 17, 2026
;* Target: AVR128DB48
;* Number of words: 31
;* Number of cycles: not counted, sending the LCD text takes about 60 ms
;* Low registers modified: r0 (through send_string)
;* High registers modified: r16 (temp), r17, r19 (nib), r20, r21 (last_state), r24 (state), Z
;*
;* Parameters: ld2410c_buffer[STATE_INDEX] in SRAM holds the target state.
;*             last_state (r21) = state currently shown on the LCD.
;*
;* Returns: last_state (r21) updated to the state just displayed.
;*
;* Notes: The SerLCD command "|" then "-" clears the screen and homes the
;* cursor. Carriage return and line feed (0x0D, 0x0A) move to line 2. The
;* sensor sends a frame about every 99 ms, so one LCD update fits between
;* frames, and the header search resyncs if a frame is missed.
;*
;***************************************************************************
display_state:                          ; start of display_state subroutine
    lds state, ld2410c_buffer + STATE_INDEX ; state = target state value from the buffer
    cp state, last_state                ; is it the same state already on the LCD?
    breq ds_done                        ; yes, nothing to redraw
    mov last_state, state               ; remember the state we are about to display

    ldi temp, '|'                       ; SerLCD setting-mode character (0x7C)
    rcall usart_send                    ; send it
    ldi temp, '-'                       ; SerLCD clear-screen command (0x2D), cursor to home
    rcall usart_send                    ; send it
    rcall delay_10ms                    ; let the SerLCD finish clearing the screen
    rcall delay_10ms                    ; a little extra margin

    ldi ZH, high(msg_title << 1)        ; Z high byte = flash byte address of "Target State: 0x"
    ldi ZL, low(msg_title << 1)         ; Z low byte = flash byte address of "Target State: 0x"
    rcall send_string                   ; send the title text to the LCD

    mov nib, state                      ; nib = copy of the target state
    swap nib                            ; swap nibbles so the upper 4 bits move to the lower 4 bits
    andi nib, 0x0F                      ; keep only the upper nibble of the state value
    rcall nibble_to_ascii               ; nib = ASCII character for the upper hex digit
    mov temp, nib                       ; temp = character to send
    rcall usart_send                    ; send the upper hex digit
    mov nib, state                      ; nib = copy of the target state again
    andi nib, 0x0F                      ; keep only the lower nibble of the state value
    rcall nibble_to_ascii               ; nib = ASCII character for the lower hex digit
    mov temp, nib                       ; temp = character to send
    rcall usart_send                    ; send the lower hex digit

    ldi temp, 0x0D                      ; carriage return
    rcall usart_send                    ; send CR
    ldi temp, 0x0A                      ; line feed
    rcall usart_send                    ; send LF, cursor is now on line 2

    rcall send_state_text               ; send the text that describes the state
ds_done:                                ; nothing more to do
    ret                                 ; return to the caller

;***************************************************************************
;* 
;* "send_state_text" - Send the description of the target state
;*
;* Description:
;* Sends a short text for the target state to the SerLCD: 0x00 = "No
;* target", 0x01 = "Moving target", 0x02 = "Stationary target", 0x03 =
;* "Moving+Stationary", anything else = "Noise detection".
;*
;* Author: Joshua Sue and Farhan Chowdhury
;* Version: 1.0
;* Last updated: November 17, 2026
;* Target: AVR128DB48
;* Number of words: 28
;* Number of cycles: not counted, depends on the text length
;* Low registers modified: r0 (through send_string)
;* High registers modified: r16 (temp), r17, Z
;*
;* Parameters: state (r24) = target state value
;*
;* Returns: none
;*
;* Notes: Table 13 of the sensor datasheet calls 0x01 "Campaign target",
;* which is a mistranslation of "movement".
;*
;***************************************************************************
send_state_text:                        ; start of send_state_text subroutine
    cpi state, 0x00                     ; is the state 0x00?
    brne st_check1                      ; no, try the next state
    ldi ZH, high(msg_state0 << 1)       ; Z high byte = flash byte address of "No target"
    ldi ZL, low(msg_state0 << 1)        ; Z low byte = flash byte address of "No target"
    rjmp st_send                        ; go send the text

st_check1:                              ; check for state 0x01
    cpi state, 0x01                     ; is the state 0x01?
    brne st_check2                      ; no, try the next state
    ldi ZH, high(msg_state1 << 1)       ; Z high byte = flash byte address of "Moving target"
    ldi ZL, low(msg_state1 << 1)        ; Z low byte = flash byte address of "Moving target"
    rjmp st_send                        ; go send the text

st_check2:                              ; check for state 0x02
    cpi state, 0x02                     ; is the state 0x02?
    brne st_check3                      ; no, try the next state
    ldi ZH, high(msg_state2 << 1)       ; Z high byte = flash byte address of "Stationary target"
    ldi ZL, low(msg_state2 << 1)        ; Z low byte = flash byte address of "Stationary target"
    rjmp st_send                        ; go send the text

st_check3:                              ; check for state 0x03
    cpi state, 0x03                     ; is the state 0x03?
    brne st_other                       ; no, it is some other value
    ldi ZH, high(msg_state3 << 1)       ; Z high byte = flash byte address of "Moving+Stationary"
    ldi ZL, low(msg_state3 << 1)        ; Z low byte = flash byte address of "Moving+Stationary"
    rjmp st_send                        ; go send the text

st_other:                               ; state is 0x04 or higher
    ldi ZH, high(msg_other << 1)        ; Z high byte = flash byte address of "Noise detection"
    ldi ZL, low(msg_other << 1)         ; Z low byte = flash byte address of "Noise detection"

st_send:                                ; Z now points at the text to send
    rcall send_string                   ; send the text to the LCD
    ret                                 ; return to the caller

;***************************************************************************
;* 
;* "nibble_to_ascii" - Convert a 4-bit value to a hexadecimal ASCII character
;*
;* Description:
;* Converts a value from 0 to 15 into the ASCII code for '0' to '9' or
;* 'A' to 'F'.
;*
;* Author: Joshua Sue and Farhan Chowdhury
;* Version: 1.0
;* Last updated: November 17, 2026
;* Target: AVR128DB48
;* Number of words: 6
;* Number of cycles: 7 to 8 (estimate, not counting the rcall)
;* Low registers modified: none
;* High registers modified: r19 (nib)
;*
;* Parameters: nib (r19) = value from 0 to 15
;*
;* Returns: nib (r19) = ASCII character for that hex digit
;*
;* Notes: subi with a negative constant adds the constant to the register.
;*
;***************************************************************************
nibble_to_ascii:                        ; start of nibble_to_ascii subroutine
    cpi nib, 10                         ; is the value 10 or more?
    brsh hex_letter                     ; yes, it is a letter A to F
    subi nib, -ASCII_0                  ; 0 to 9: add 48 to get '0' to '9'
    ret                                 ; return to the caller

hex_letter:                             ; value is 10 to 15
    subi nib, -ASCII_A_10               ; add 55 to get 'A' to 'F' (10 + 55 = 65 = 'A')
    ret                                 ; return to the caller

;***************************************************************************
;* 
;* "usart_send" - Send one character on USART3
;*
;* Description:
;* Waits until the USART3 transmit data register is empty (DREIF = 1),
;* then writes the character to TXDATAL, which starts the transmission.
;*
;* Author: Joshua Sue and Farhan Chowdhury
;* Version: 1.0
;* Last updated: November 17, 2026
;* Target: AVR128DB48
;* Number of words: 7
;* Number of cycles: varies, it waits for the previous character to finish
;* Low registers modified: none
;* High registers modified: r17
;*
;* Parameters: temp (r16) = character byte to send
;*
;* Returns: none
;*
;* Notes: Taken from Laboratory 07 (USART_write_message). The USART
;* finishes sending the frame by itself after this routine returns.
;*
;***************************************************************************
usart_send:                             ; start of usart_send subroutine
    lds r17, USART3_STATUS              ; read the USART3 status register
    sbrs r17, DREIF_BIT                 ; skip the rjmp when DREIF (bit 5) is set
    rjmp usart_send                     ; not empty yet, keep waiting
    sts USART3_TXDATAL, temp            ; write the character, hardware sends the frame
    ret                                 ; return to the caller

;***************************************************************************
;* 
;* "send_string" - Send a zero-terminated string from flash on USART3
;*
;* Description:
;* Sends each character of a zero-terminated string in program memory
;* with usart_send until the zero terminator is reached.
;*
;* Author: Joshua Sue and Farhan Chowdhury
;* Version: 1.0
;* Last updated: November 17, 2026
;* Target: AVR128DB48
;* Number of words: 6
;* Number of cycles: varies with the string length
;* Low registers modified: none
;* High registers modified: r16 (temp), r17, Z
;*
;* Parameters: Z (r31:r30) = byte address of the string in program memory
;*
;* Returns: none (Z ends up just past the terminator)
;*
;* Notes: Taken from Laboratory 07 (USART_write_message).
;*
;***************************************************************************
send_string:                            ; start of send_string subroutine
    lpm temp, Z+                        ; load the next character from flash, advance Z
    tst temp                            ; is it the zero terminator?
    breq ss_done                        ; yes, the whole string has been sent
    rcall usart_send                    ; no, transmit the character
    rjmp send_string                    ; go get the next character
ss_done:                                ; end of the string
    ret                                 ; return to the caller

;***************************************************************************
;* 
;* "delay_10ms" - Delay of about 10 ms
;*
;* Description:
;* Software delay of approximately 10 ms with a 4 MHz clock, built from
;* two nested counting loops.
;*
;* Author: Joshua Sue and Farhan Chowdhury
;* Version: 1.0
;* Last updated: November 17, 2026
;* Target: AVR128DB48
;* Number of words: 7
;* Number of cycles: about 40,000 (estimate)
;* Low registers modified: none
;* High registers modified: r16, r17
;*
;* Parameters: none
;*
;* Returns: none
;*
;* Notes: Taken from Laboratory 07.
;*
;***************************************************************************
delay_10ms:                             ; start of delay_10ms subroutine
    ldi r16, 100                        ; load the outer delay count
d10_outer:                              ; outer loop
    ldi r17, 133                        ; load the inner delay count
d10_inner:                              ; inner loop
    dec r17                             ; decrease the inner counter
    brne d10_inner                      ; repeat the inner loop
    dec r16                             ; decrease the outer counter
    brne d10_outer                      ; repeat the outer loop
    ret                                 ; return after approximately 10 ms

;***************************************************************************
;* 
;* "delay_500ms" - Delay of about half a second
;*
;* Description:
;* Calls delay_10ms fifty times for a delay of approximately 500 ms.
;*
;* Author: Joshua Sue and Farhan Chowdhury
;* Version: 1.0
;* Last updated: November 17, 2026
;* Target: AVR128DB48
;* Number of words: 5
;* Number of cycles: about 2,000,000 (estimate)
;* Low registers modified: none
;* High registers modified: r16, r17, r20
;*
;* Parameters: none
;*
;* Returns: none
;*
;* Notes: Taken from Laboratory 07.
;*
;***************************************************************************
delay_500ms:                            ; start of delay_500ms subroutine
    ldi r20, 50                         ; select fifty delay repetitions
d500_loop:                              ; repeat the 10 ms delay
    rcall delay_10ms                    ; wait approximately 10 ms
    dec r20                             ; decrease the repetition counter
    brne d500_loop                      ; repeat until all delays finish
    ret                                 ; return to the caller

;---------------------------- text stored in flash -------------------------
; Each string ends with a zero. Every string is padded to an even number
; of bytes so the assembler does not have to add padding.
msg_title:  .db "Target State: 0x", 0, 0 ; title text on line 1, the hex digits follow it
msg_state0: .db "No target", 0          ; state 0x00
msg_state1: .db "Moving target", 0      ; state 0x01
msg_state2: .db "Stationary target", 0  ; state 0x02
msg_state3: .db "Moving+Stationary", 0  ; state 0x03
msg_other:  .db "Noise detection", 0    ; states 0x04 and above