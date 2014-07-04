# This tests is we can jump and execute simple instructions 
# with the TLB
.ktext

	# Change Index
	li $4, 0x11
	mtc0 $4, $0

	# Change EntryHi (Tag / Virtual Address)
	li $4, 0x40009000
	mtc0 $4, $10

	# Change EntryLo (Data / Physical Address)	
	li $4, 0x00000700
	mtc0 $4, $2

	# tlbwi
	.word 0x42000002
	
	# ERET to user space
	li $4, 0x40009000
	mtc0 $4, $14
	.word 0x42000018	

# General Exception Handler
.ktext 0x80000180
	li $5, 0xc0070002
	.word 0xE

	li $10, 0xFa17Fa17
	.word 0xE
	li $10, 0xFa17Fa17

# TLB Exception Handler
# Find the hashed page table address
.ktext 0x80000200
	# Calculate hashed page table addr
	# Base + {VPN[7:0], 3'b000} == base + (badVaddr[19:12] << 3)
	# K0 = BadVAddr
	
	mfc0 $k0, $8 
	li $k1, 0x000ff000
	and $k0, $k0, $k1

	# K0 = {VPN[7:0], 3'b000}
	srl $k0, $k0, 9 				
	
	# Base = 0xA000_3000
	li $k1, 0xA0003000
	add $k0, $k0, $k1
	
	# store the page table entry in the TLB
	# (ASSUMING NOT A PAGE FAULT)
	# Get Data
	lw $k1, 0($k0)
	mtc0 $k1, $2

	# Get Tag
	lw $k1, 4($k0)
	mtc0 $k1, $10
	
	# tlbwr
	.word 0x42000006

	# Return and Retry the instruction with an ERET
	.word 0x42000018

# Page Table
.kdata
	# Data (not used)
	.word 0x00001700
	# Tag (not used)
	.word 0x40000000

	# Data (not used)
	.word 0x00001700
	# Tag (not used)
	.word 0x50001000

	# Data
	.word 0x00001700
	# Tag
	.word 0x50002000
		
# This is Virtual Address 0x4000_0000
.text 
	# TLB Entry does not exsist, miss handle
	li $9, 0x40000004
	lw $10, 0($9)	

	li $9, 0x50001010
	lw $11, 0($9)	

	li $9, 0x5000201C
	lw $12, 0($9)	

	.word 0xE

.data
	.word 0xfa17fa17
	.word 0x13371337
	.word 0xfa17fa17
	.word 0xfa17fa17
	.word 0x13371338
	.word 0xfa17fa17
	.word 0xfa17fa17
	.word 0x13371339
	.word 0xfa17fa17
