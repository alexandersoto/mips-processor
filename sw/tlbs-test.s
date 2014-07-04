# This tests is we can jump and execute simple instructions 
# with the TLB
.ktext

	# Change Index
	li $4, 0x10
	mtc0 $4, $0
	# Change EntryHi (Tag / Virtual Address)
	li $4, 0x40000000
	mtc0 $4, $10
	# Change EntryLo (Data / Physical Address)	
	li $4, 0x00000700
	mtc0 $4, $2
	# tlbwi
	.word 0x42000002
	
	#ERET to user space
	li $4, 0x40000000
	mtc0 $4, $14
	.word 0x42000018	

# General Exception Handler
.ktext 0x80000180
	li $6, 0xFa17Fa17
	.word 0xE

# TLB Exception Handler
.ktext 0x80000200
	li $10, 0xC0070002
	.word 0xE
	li $10, 0xFa17Fa17
	
# This is Virtual Address 0x4000_0000
.text 

	# TLB Entry doesn't exsist!
	li $9, 0x60000000
	sw $4, 0($9)
	
	# Shouldn't get here
	li $10, 0xDEADC0DE
	.word 0xE
