;; Written by Jan Kansky, 5/15/2001.
;;
;; Copyright (C) 2001  Jan Edward Kansky  - kansky@jeklink.net
;;
;; This program is free software; you can redistribute it and/or
;; modify it under the terms of the GNU General Public License as
;; published by the Free Software Foundation; either version 2 of the
;; License, or (at your option) any later version.
;;
;; This program is distributed in the hope that it will be useful, but
;; WITHOUT ANY WARRANTY; without even the implied warranty of
;; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;; General Public License for more details.
;;
;; You should have received a copy of the GNU General Public License
;; along with this program; if not, write to the Free Software
;; Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

	;; .device	AT90S1200
	.include "1200def.inc"

	;; ***** Global Register Variables

	.def ch1_lsb=r0
	.def ch1_msb=r1
	.def ch2_lsb=r2
	.def ch2_msb=r3
	.def ch3_lsb=r4
	.def ch3_msb=r5
	.def ch4_lsb=r6
	.def ch4_msb=r7
	.def ch5_lsb=r8
	.def ch5_msb=r9
	.def ch6_lsb=r10
	.def ch6_msb=r11
	.def ch7_lsb=r12
	.def ch7_msb=r13
	.def ch8_lsb=r14
	.def ch8_msb=r15
	.def ch9_lsb=r16
	.def ch9_msb=r17
	.def ch10_lsb=r18
	.def ch10_msb=r19
	.def temp=r20
	.def save_state=r21
	.def current_ppm=r22	; Current input channel for PPM decoding
	.def current_out=r23	; Current output nibblet for LPT interface*2
	.def timer_val=r24	; Timer value at PPM irq
	.def overflows=r25	; Number of timer overflows since last PPM irq
	.def num_channels=r26
	.def temp2=r27
	.def y_low_byte=r28
	.def temp5=r29
	.def temp3=r30
	.def temp4=r31

	;; ***** Code
.cseg
.org 0
	rjmp	RESET		; Reset interrupt handler
	rjmp	PPM_INTERRUPT
	rjmp	TIMER_OVERFLOW
	rjmp	RESET
RESET:	ldi temp,$1f		; PB4 is the PtrClk output, lsn out
	out DDRB,temp
	ldi temp,$F0		; PtrClk high, enable pullups on top 3 bits
	out PORTB,temp		;  No wait signal, msn pullups on
	ldi temp,$00
	out DDRD,temp		;  Port D are all not used, so set to inputs
	ldi temp,$FB
	out PORTD,temp		; activate all pullups on PORTD, except irq
	ldi temp,$00
	mov ch1_lsb,temp	; Set all channel values to zero initially
	mov ch1_msb,temp
	mov ch2_lsb,temp
	mov ch2_msb,temp
	mov ch3_lsb,temp
	mov ch3_msb,temp
	mov ch4_lsb,temp
	mov ch4_msb,temp
	mov ch5_lsb,temp
	mov ch5_msb,temp
	mov ch6_lsb,temp
	mov ch6_msb,temp
	mov ch7_lsb,temp
	mov ch7_msb,temp
	mov ch8_lsb,temp
	mov ch8_msb,temp
	mov ch9_lsb,temp
	mov ch9_msb,temp
	mov ch10_lsb,temp
	mov ch10_msb,temp
	mov num_channels,temp	;  Number of detected channels initialized to 0
	mov current_ppm,temp	;  Current channel is 0, indicating not synched
	ldi current_out,0	;  Nibble 0 is to be output on LPT
	mov overflows,temp	;  No 8 bit timer overflows have occured.
	clr y_low_byte		;  Starting outputting channel 1 lsn
	mov num_channels,temp   ;  No channels have been detected.
	ldi temp,$02		;  Set falling edge interrupts.
	out MCUCR,temp
	ldi temp,$40
	out GIMSK,temp		;  Enable external interrupt
	ldi temp,$02
	out TIFR,temp
	out TIMSK,temp		;  Enable timer overflow irqs
	sei			;  Enable interrupts
	ldi temp,1
	out TCCR0,temp		;  Timer running with no prescaling
	ldi temp,$00
	out TCNT0,temp
	ldi temp2,$00
	
START:	mov temp3,y_low_byte	;  Copy current byte address to Z register
	ld temp,Z		;  Load data to prepare for output
	ldi temp2,$F0		;  Keep pullups on, and PtrClk high for now
	sbrc current_out,0	;  See if we are on an odd nibble
	swap temp		;  if so then swap nibbles
	andi temp,$0F		;  Mask of least significant nibble
	or temp2,temp		;  Or that nibble into the output byte
	ldi temp,$F0		;  Set PtrClk, maintain pullups
	cpi y_low_byte,0	;  See if we should output a sync pulse
	brne sync_it_up		;  Current output channel is zero
	cpi current_out,0	;  Current nibble output is zero
	brne sync_it_up
	ldi temp,$F1		;  Set PtrClk, maintain pullups, set sync
sync_it_up:	
	out PORTB,temp
POLL:	sbic PINB,5		;  Wait for host to ask for data
	rjmp POLL
	out PORTB,temp2		;  Output data to host
	andi temp2,$EF		;  Set Ptr Clock low, indicating data ready
	out PORTB,temp2		;  Output data to host
	inc current_out		;  Add one to prepare for next nibble output
	cpi current_out,$02	;  If we just finished outputting second nibble
	brne dont_update_y
	inc y_low_byte		;  increment the byte address
dont_update_y:		
	cpi current_out,$03	;  If we finished the third nibble, start over
	brlo channel_not_done
	ldi current_out,0	;  Back to starting nibble
	inc y_low_byte		;  Switch to next channel
channel_not_done:
REPLY:	sbis PINB,5		;  Wait for host to finish reading data
	rjmp REPLY		
	cpi y_low_byte,$14	;  If done with 20 bytes, (9 channels+1 sync)*2
	brne cycle_not_done
	inc y_low_byte		;  Increment address so irqs still registers
no_new_data_yet:	
	cpi temp4,$FF		;  Wait for new frame of data to complete
	brne no_new_data_yet
	ldi temp4,$00
	ldi temp,$F1		;  Tell host we are ready to transfer new frame
	out PORTB,temp
dont_go_back_to_zero_yet:
	sbic PINB,5		;  Wait for host to ask for new frame
	rjmp dont_go_back_to_zero_yet
	clr y_low_byte		;  Reset address to zero
cycle_not_done:	
	rjmp START

TIMER_OVERFLOW:
	in save_state,SREG
	inc overflows
	out SREG,save_state
	reti
	
PPM_INTERRUPT:
	in save_state,SREG
	ldi temp4,0
	out TCCR0,temp4		;  Stop timer
	mov temp5,temp3		;  Backup Z register
	in timer_val,TCNT0	;  Read timer value
	in temp4,TIFR		;  Read in Timer interrupt flag register
	ldi temp3,$00		;  Reset counter to zero, prepare for next irq
	out TCNT0,temp3
	ldi temp3,1
	out TCCR0,temp3		;  Timer running with no prescaling	
	sbrs temp4,1
	rjmp no_overflow	;  If not set, continue
	inc overflows		;  Increment overflow counter
	ldi temp3,$02
	out TIFR,temp3		;  Clear the overflow
no_overflow:
	cpi overflows,$0E	; Check if this is a sync pulse
	brlo check_sync		; If not, put data into appropriate channel
	ldi current_ppm,1	; if it is reset channel counter to first chanl
	mov temp4,y_low_byte	; Check to make sure we aren't currently
	lsr temp4		;  sending this data to the LPT	
	ldi temp3,$09	  
	cp temp3,temp4
	breq dont_change_now
	mov temp3,timer_val
	lsr temp3
	lsr temp3
	lsr temp3
	mov temp4,overflows
	sbrc temp4,0
	sbr temp3,$20
	sbrc temp4,1
	sbr temp3,$40
	sbrc temp4,2
	sbr temp3,$80
	lsr temp4
	lsr temp4
	lsr temp4
	mov ch10_lsb,temp3	; Store the sync pulse width in ch10
	mov ch10_msb,temp4	; this will allow for total cycle time comp.
	mov temp3,num_channels  ; Clear unused channels
	ldi num_channels,$00
	lsl temp3
continue_clearing:	
	cpi temp3,$12		; See if we've cleared up to and including 9
	breq done_clearing
	st Z,num_channels	;  Clear low byte
	inc temp3
	st Z,num_channels	;  Clear high byte
	inc temp3
	rjmp continue_clearing
done_clearing:	
	ldi temp4,$FF
dont_change_now:		
	rjmp exit_ppm		; and exit to await next irq
check_sync:	
	cpi current_ppm,0	; If not a sync pulse, check to see if we
	brne save_data		; have achieved sync.  If not, exit, else
	rjmp exit_ppm		; see which channel the irq belongs to.
save_data:
	cp current_ppm,num_channels;  Determine how many channels are 
	brlo not_new_max	; being output by the radio transmitter
	mov num_channels,current_ppm
not_new_max:
	mov temp4,y_low_byte	; Check to make sure we aren't currently
	lsr temp4		;  sending this data to the LPT
	mov temp3,current_ppm	;  Prepare Z register for Immediate with offset
	dec temp3
	cp temp3,temp4
	breq dont_save
	lsl temp3
	ldi temp4,0
	st Z,timer_val
	inc temp3
	st Z,overflows
dont_save:	
	ldi temp3,0
	ldi temp4,9
	cp current_ppm,temp4
	brlo done_ppm
	dec current_ppm	  ; Decrement current_ppm so ch 10 or higher go into 9
done_ppm:	
	inc current_ppm		; Increment for the next channel
exit_ppm:	
	ldi overflows,0
	mov temp3,temp5		;  Restore Z register
	out SREG,save_state
	reti	




















