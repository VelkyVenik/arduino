.nolist
.include "./m328Pdef.inc"
.list


.def temp = r16
.def rx_ready = r17
.def button = r18
.def data = r19
.def data_debug = r20
.def overflows = r21

.DSEG
rx_byte:    .byte   1


.CSEG
.ORG 0x0000
    jmp main                ; 1  Reset Vector
.ORG 0x0020                 ; memory location of Timer0 overflow handler
    rjmp overflow_handler   ; go here if a timer0 overflow interrupt occurs 
.ORG 0x0024
    jmp usart_rxc_isr       ; 19 USART Rx Complete


.macro ldidb                ; load .db address into Z register
ldi ZL, low(@0 * 2)
ldi ZH, high(@0 * 2)
.endmacro

.macro debug                ; send one byte to serial
mov data_debug, @0
rcall usart_debug
.endmacro

main:
    ldi temp,low(RAMEND)     ;stack init
    out SPL,temp
    ldi temp,high(RAMEND)
    out SPH,temp

    rcall display_init
    rcall usart_init
    rcall counter_init
    rcall button_init

    sei                     ;enable all interrupts

    rcall cmd_input

main_loop:
    or rx_ready, button         ; data ready or button pressed?
    breq main_loop          ;no (Z=1). Loop

    clr rx_ready            ;indicate no more byte
    clr button

    ; load command
    lds data,rx_byte         ;get the byte
    rcall usart_tx
    rcall usart_nl
    debug data

    ; execute command
    rcall cmd_run

    rcall cmd_input

    jmp main_loop

;-----------------------------------------------------------------
;--------------------- Commands ----------------------------------
;-----------------------------------------------------------------

cmd_input:
    ; print input message
    ldidb s_input
    rcall usart_write
    ret


cmd_run:
    cpi data, 'x'
    breq cmd_x

    cpi data, 'v'
    breq cmd_v

    cpi data, 'h'
    breq cmd_h

    cpi data, 'l'
    breq cmd_l

    cpi data, 'c'
    breq cmd_c

    ;default
    ldidb s_cmd_unknown
    rcall usart_writeln

cmd_run_end:
    ret

cmd_x:
    ldidb s_cmd_x
    rcall usart_writeln
    ; disable UART interface
    clr temp
    sts UCSR0B,temp
    rjmp cmd_run_end

cmd_v:
    ldidb s_cmd_v
    rcall usart_writeln
    rjmp cmd_run_end

cmd_h:
    ldidb s_cmd_h_1
    rcall usart_writeln
    ldidb s_cmd_h_2
    rcall usart_writeln
    rjmp cmd_run_end

cmd_l:
    ldidb s_cmd_l
    rcall usart_write

cmd_l_loop:
    tst rx_ready            ;byte stored?
    breq cmd_l_loop        

    clr rx_ready                ;indicate no more byte

    ; load and print number
    lds data,rx_byte            ;get the byte from UART
    rcall usart_tx
    rcall usart_nl

    sbci data, '0'
    rcall display_show

    rjmp cmd_run_end

cmd_c:
    clr data

cmd_c_loop:
    rcall display_show
    rcall delay
    inc data
    cpi data, 10
    brlt cmd_c_loop

    rjmp cmd_run_end

;-----------------------------------------------------------------
;--------------------- Button ------------------------------------
;-----------------------------------------------------------------
button_init:
    sbi PORTB, 3             ; enable pull up resistor for input pin
    clr button
    ret


;-----------------------------------------------------------------
;--------------------- Led ---------------------------------------
;-----------------------------------------------------------------

display_init:
    ldi temp, 0b11111100     ; set PD2 - PD7 to output
    out DDRD, temp
    ldi temp, 0b00000111     ; set PB0 - PB1 to output
    out DDRB, temp

    ser temp;                    ; turn of all segments = 1=off, 0=on
    out PORTD, temp
    out PORTB, temp

    ret

display_show:                   ; display one number stored in data register
    cpi data, 10
    brlt display_show_start
    rcall display_hide
    ret

display_show_start:
    ldidb d_display_1           ; set Z to table
    clr r0

    add ZL, data
    adc ZH, r0
    lpm r1, Z                  ; loada number data
    com r1                     ; invert data - 1=off, 0=on

    mov temp, r1
    lsl temp
    lsl temp
    out PORTD, temp

    sbrs r1, 6
    cbi PORTB, 0

    sbrc r1, 6
    sbi PORTB, 0

    ret

display_hide:
    sbi PORTB, 0
    ser temp
    out PORTD, temp

display_dot_1_on:
    cbi PORTB, 1
    ret

display_dot_1_off:
    sbi PORTB, 1
    ret

display_dot_2_on:
    cbi PORTB, 2
    ret

display_dot_2_off:
    sbi PORTB, 2
    ret

display_dot_toggle:
    sbic PORTB, 1
    rjmp display_dot_togle_on
    rcall display_dot_1_off
    rcall display_dot_2_on
    rjmp display_dot_toggle_done

display_dot_togle_on:
    rcall display_dot_1_on
    rcall display_dot_2_off

display_dot_toggle_done:
    ret


;-----------------------------------------------------------------
;--------------------- USART -------------------------------------
;-----------------------------------------------------------------

usart_rxc_isr:
    push r16            ;save r16 in stack
    in r16,SREG
    push r16            ;save SREG in stack

    tst rx_ready        ;byte already received?
    breq rx_store       ;not received (0x00, Z=1). store this one

    pop r16             ;retrieve SREG from stack
    out SREG,r16
    pop r16             ;retrieve r16 from stack
    reti

rx_store:               ;store received byte
    lds r16,UDR0        ;read byte
    sts rx_byte,r16     ;store it
    ser rx_ready        ;indicate rx received

    pop r16             ;retrieve SREG from stack
    out SREG,r16
    pop r16             ;retrieve r16 from stack
    reti

usart_tx:               ; wait for empty buffer and then send data
    lds     temp, UCSR0A
    sbrs    temp, UDRE0
    rjmp    USART_TX
    sts     UDR0, data
    ret

usart_debug:                ; convert register value to ascii hex and send to UART
    push data
    push ZL
    push ZH

    clr r0                  ; used for adc

    ldi data, '0'           ; send 0x hexa prefix
    rcall usart_tx
    ldi data, 'x'
    rcall usart_tx

    ldidb s_hex             ; get higher nibble asci value
    mov temp, data_debug
    andi temp, 0xF0
    swap temp
    add ZL, temp
    adc ZH, r0
    lpm data, Z
    rcall usart_tx

    ldidb s_hex             ; get lower nibble asci value
    mov temp, data_debug
    andi temp, 0x0F
    add ZL, temp
    adc ZH, r0
    lpm data, Z
    rcall usart_tx

    rcall usart_nl

    pop ZH
    pop ZL
    pop data
    ret


usart_init:
    ldi temp, (1<<RXCIE0) | (1<<RXEN0) | (1<<TXEN0)   ; enable RX Complete interrupt
    sts UCSR0B,temp

    ldi temp,103         ;set baud rate (9600bds / 16MHz)
    sts UBRR0L,temp
    ldi temp,0
    sts UBRR0H,temp
    ;keep default 8bits, 1 stop and no parity

    clr rx_ready        ;no data ready to send in buffer rx_byte
    ret


usart_write:            ; send data located on address stored in Z register

write_loop:
    lpm data, Z+

    tst data
    breq write_finish

    rcall usart_tx
    rjmp write_loop

write_finish:
    ret

usart_nl:               ; send new line
    push data

    ldi data, 0x0d
    rcall usart_tx
    ldi data, 0x0a
    rcall usart_tx

    pop data

    ret

usart_writeln:
    rcall usart_write
    rcall usart_nl
    ret


;-----------------------------------------------------------------
;--------------------- Countert ----------------------------------
;-----------------------------------------------------------------
counter_init:
    ldi temp,  0b00000101
    out TCCR0B, temp        ; set the Clock Selector Bits CS00, CS01, CS02 to 101
                            ; this puts Timer Counter0, TCNT0 in to FCPU/1024 mode
                            ; so it ticks at the CPU freq/1024
    ldi temp, 0b00000001
    sts TIMSK0, temp        ; set the Timer Overflow Interrupt Enable (TOIE0) bit 
                            ; of the Timer Interrupt Mask Register (TIMSK0)

    clr temp
    out TCNT0, temp         ; initialize the Timer/Counter to 0

    ret

delay:
    clr overflows           ; set overflows to 0 
    sec_count:
    cpi overflows,15        ; compare number of overflows and 30
    brne sec_count          ; branch to back to sec_count if not equal 
    ret                     ; if 30 overflows have occured return to blink

; TODO calculate limit values for both functions
overflow_handler: 
    in temp, SREG
    push temp

    inc overflows           ; add 1 to the overflows variable
    cpi overflows, 20       ; compare with 61
    brne overflow_handler_done  ; Program Counter + 2 (skip next line) if not equal
    rcall display_dot_toggle
    clr overflows           ; if 61 overflows occured reset the counter to zero

overflow_handler_done:
    pop temp
    out SREG, temp
    reti                    ; return from interrupt



;-----------------------------------------------------------------
;--------------------- DATA --------------------------------------
;-----------------------------------------------------------------

s_input:        .db ">> ", 0
s_cmd_x:        .db "Exit.....", 0
s_cmd_v:        .db "Console Demo :)", 0x0d, 0x0a, "build %YEAR%%MONTH%%DAY%%HOUR%%MINUTE%", 0
s_cmd_h_1:      .db "Help:", 0x0d, 0x0a, " h - this help", 0x0d, 0x0a, " v - program version", 0x0d, 0x0a, " x - exit console", 0
s_cmd_h_2:      .db " l - control display", 0x0d, 0x0a, " c - count on display", 0
s_cmd_unknown:  .db "Unknown command, enter h for help", 0
s_cmd_l:        .db "Enter number? ", 0

d_display_1:    .db 0b0111111, 0b0000110, 0b1011011, 0b1001111, 0b1100110, 0b1101101, 0b1111101, 0b0000111, 0b1111111, 0b1101111
s_hex:          .db "0123456789abcdef"
