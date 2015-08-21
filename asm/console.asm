.nolist
.include "./m328Pdef.inc"
.list


.def temp = r16
.def rx_ready = r17
.def led = r18
.def data = r19

.DSEG
rx_byte:    .byte   1


.CSEG
.ORG 0x0000
    jmp main ; 1  Reset Vector
.ORG 0x0024
    jmp usart_rxc_isr ; 19 USART Rx Complete

.macro ldidb            ; load .db address into Z register
ldi ZL, low(@0 * 2)
ldi ZH, high(@0 * 2)
.endmacro

main:
    ldi temp,low(RAMEND)     ;stack init
    out SPL,temp
    ldi temp,high(RAMEND)
    out SPH,temp

    rcall led_init
    rcall usart_init

    sei                     ;enable all interrupts

    rcall cmd_input

main_loop:
    tst rx_ready            ;byte stored?
    breq main_loop          ;no (Z=1). Loop

    clr rx_ready            ;indicate no more byte

    ; load command
    lds data,rx_byte         ;get the byte
    rcall usart_tx
    rcall usart_nl

    ; execute command
    rcall cmd_run

    rcall led_debug
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
    ldidb s_cmd_h
    rcall usart_writeln
    rjmp cmd_run_end


;-----------------------------------------------------------------
;--------------------- Utils -------------------------------------
;-----------------------------------------------------------------


led_init:
    ldi temp, 0b11111100     ; set PD2 - 7 to output
    out DDRD, temp
    ldi temp, 0b00000011     ; set PB0 - PB1 to output
    out DDRB, temp
    ldi led, 0b00000001

    ret


led_debug:
    push r20

    mov r20, led
    lsl r20
    lsl r20
    out PORTD, r20

    mov r20, led
    lsr r20
    lsr r20
    lsr r20
    lsr r20
    lsr r20
    lsr r20
    out PORTB, r20

    lsl led
    brcc PC+2
    ldi led, 0b00000001

    pop r20

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
;--------------------- DATA --------------------------------------
;-----------------------------------------------------------------

s_input:        .db ">> ", 0
s_cmd_x:        .db "Exit.....", 0
s_cmd_v:        .db "Console Demo :)", 0x0d, 0x0a, "build %YEAR%%MONTH%%DAY%%HOUR%%MINUTE%", 0
s_cmd_h:        .db "Help:", 0x0d, 0x0a, " h - this help", 0x0d, 0x0a, " v - program version", 0x0d, 0x0a, " x - exit console", 0
s_cmd_unknown:  .db "Unknown command, enter h for help", 0
