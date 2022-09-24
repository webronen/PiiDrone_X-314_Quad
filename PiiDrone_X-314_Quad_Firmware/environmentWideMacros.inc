.macro protectedIOREGwrite
	ldi r16, CPU_CCP_IOREG_gc
	sts CPU_CCP, r16
.endmacro

.macro softwareReset
	protectedIOREGwrite
	ldi r16, RSTCTRL_SWRE_bm
	sts RSTCTRL_SWRR, r16
.endmacro

.macro CLK_MAIN_setup

	; CLKCTRL - Clock Controller
	; Source: "file:///C:/Users/Webronen/Documents/AVR/AVRxt_ATtiny3224.pdf#page=92"
	
	; CLK_MAIN = 32.768 kHz internal ultra low-power oscillator
	protectedIOREGwrite
	ldi r16, CLKCTRL_CLKSEL_OSCULP32K_gc
	sts CLKCTRL_MCLKCTRLA, r16
	
	; CLK_PER = CLK_MAIN
	protectedIOREGwrite
	ldi r16, CLKCTRL_PEN_bp
	sts CLKCTRL_MCLKCTRLB, r16
	
	; Lock Control A and B registers
	protectedIOREGwrite
	ldi r16, CLKCTRL_LOCKEN_bm
	sts CLKCTRL_MCLKLOCK, r16
	
.endmacro

.macro TCA_MODE_setup

	; TCA - 16-bit Timer/Counter Type A
	; Source: "file:///C:/Users/Webronen/Documents/AVR/AVRxt_ATtiny3224.pdf#page=228"
	
	; Enable TCA
	ldi r16, TCA_SPLIT_ENABLE_bm
	sts TCA0_SPLIT_CTRLA, r16
	
	; Enable WO2, WO3, WO4 and WO5 output generation
	ldi r16, TCA_SPLIT_LCMP2EN_bm | \
					 TCA_SPLIT_HCMP0EN_bm | \
					 TCA_SPLIT_HCMP1EN_bm | \
					 TCA_SPLIT_HCMP2EN_bm
					 
	sts TCA0_SPLIT_CTRLB, r16
	
	; Enable split mode, max. 6 independent 8bit PWM channels
	ldi r16, TCA_SPLIT_SPLITM_bm
	sts TCA0_SPLIT_CTRLD, r16
	
	; Set WO2, WO3, WO4 and WO5 data direction to output
	sbi VPORTB_DIR, 2
	sbi VPORTA_DIR, 3
	sbi VPORTA_DIR, 4
	sbi VPORTA_DIR, 5
	
.endmacro