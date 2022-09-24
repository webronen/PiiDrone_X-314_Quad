.include "environmentWideMacros.inc"
; .include "interruptVectorMapping.inc"
; .include "interruptVectorSubroutines.inc"

.macro ESC_PWM ; Total: 8 cycles
	sts TCA0_SPLIT_LCMP2, @0 ;  7 = PB2 = WO2, CW2
	sts TCA0_SPLIT_HCMP0, @1 ; 13 = PA3 = WO3, CCW2
	sts TCA0_SPLIT_HCMP1, @2 ;  2 = PA4 = WO4, CW1
	sts TCA0_SPLIT_HCMP2, @3 ;  3 = PA5 = WO5, CCW1
.endmacro

setup:

	CLK_MAIN_setup
	TCA_MODE_setup
	
	ldi r16, 8
	ldi r17, 32
	ldi r18, 64
	ldi r19, 255
	
	ESC_PWM r16, r17, r18, r19
	
loop:

	; 1. Read IMU (Gyroscope, Accelerometer, Magnetometer? and Barometer?) max. 6B/48b
	; 2. Read RCU (Remote control unit) max. 2B/16b
	; 3. Compare values to pre-calculated control model
	
	;ldi r16, 8
	;ldi r17, 32
	;ldi r18, 64
	;ldi r19, 255
	
	;ESC_PWM r16, r17, r18, r19
	
	rjmp loop