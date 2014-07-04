# This test tests to make sure address translations of kseg0
# is handled correctly
.ktext
	
	# Kseg0 is unmapped, therefore no need for TLB

	# Get data from memory
	li $4, 0x80003008
	lw $5, 0($4)

	li $4, 0x80003018
	lw $6, 0($4)

	li $4, 0x80003024
	lw $7, 0($4)

	.word 0xE

.kdata
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
