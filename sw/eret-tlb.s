# This test tests to make sure address translations of kseg2
# is handled correctly
.ktext

	# Change Index
	li $4, 0x10
	mtc0 $4, $0
	# Change EntryHi (Tag / Virtual Address)
	li $4, 0x00400000
	mtc0 $4, $10
	# Change EntryLo (Data / Physical Address)	
	li $4, 0x00000700
	mtc0 $4, $2
	# tlbwi
	.word 0x42000002
	
	# Go into user space
	lui $4, 0x00400000
	jr $4

.ktext 0x80000180
	mfc0 $2, $14
	addiu $2, $2, 4
	mtc0 $2, $14
	beq $a0, $zero, done
	.word 0x42000018 # eret
	li $7, 0xFA17
done:
	mfc0 $3, $13
	ori $a2, $a2, 0x1337
	.word 0xE # testdone
	li $6, 0xFA17

.text
	addiu $a0, $zero, 0x1234
	syscall
	addiu $a1, $zero, 0x5678
	addiu $a0, $zero, 0x0000
	syscall
	li $5, 0xFA17
