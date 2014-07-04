# This test tests to make sure address translations of kuseg
# is handled correctly
.ktext

	# FOR LOADING 0x00018240	
	# Change Index
	li $4, 0x10
	mtc0 $4, $0

	# Change EntryHi (Tag / Virtual Address)
	li $4, 0x00000008
	mtc0 $4, $10

	# Change EntryLo (Data / Physical Address)	
	li $4, 0x00000700
	mtc0 $4, $2

	# tlbwi
	.word 0x42000002


	# FOR LOADING 0x18447000
	# Change Index
	li $4, 0x12
	mtc0 $4, $0
	
	# Change EntryHi (Tag / Virtual Address)
	li $4, 0x65400018
	mtc0 $4, $10

	# Change EntryLo (Data / Physical Address)	
	li $4, 0x00000700
	mtc0 $4, $2

	# tlbwi
	.word 0x42000002



	# FOR LOADING 0x18740000
	
	# Change EntryHi (Tag / Virtual Address)
	li $4, 0x32100018
	mtc0 $4, $10

	# Change EntryLo (Data / Physical Address)	
	li $4, 0x00000700
	mtc0 $4, $2

	# tlbwr
	.word 0x42000006


	# Get data from memory
	li $4, 0x00000008
	lw $5, 0($4)

	li $4, 0x65400018
	lw $6, 0($4)

	li $4, 0x32100024
	lw $7, 0($4)


	.word 0xE

.text
	.word 0xfa17fa17
	.word 0xfa17fa17
	.word 0x00018240
	.word 0xfa17fa17
	.word 0xfa17fa17
	.word 0xfa17fa17
	.word 0x18447000
	.word 0xfa17fa17
	.word 0xfa17fa17
	.word 0x00018740
	.word 0xfa17fa17
