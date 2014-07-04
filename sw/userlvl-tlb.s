# This tests if we can jump and execute simple instructions with the TLB
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
	
	# Go into user space
	li $4, 0x40000000
	jr $4

.text 

	# This is Virtual Address 0x4000_0000
    # branch after branch
	lui $6, 0x1000
	addi $7, $6, 0x1337
	li $8, 0x01020304
	li $9, 0xc007c007
	.word 0xE
