# Our small stub kernel
.ktext
	# Set up stack space - address 90000000 will be the "stack" pointer always
	lui	$k1, 0x90000000
	sw $k1, 0($k1)
	
	# Jump to the user program with an ERET instruction
	lui $4, 0x00400000
	mtc0 $4, $14
	.word 0x42000018
	
# Exception Handler Code
.ktext 0x80000180
	# Get the cause code
	# $k0 == cause code

	mfc0 $k0, $13

	# System Call Exception
	addi $k1, $zero, 32
	beq $k0, $k1, systemCall  

	# AdEL or AdES Exception
	addi $k1, $zero, 16
	beq $k0, $k1, AdE  

	addi $k1, $zero, 20
	beq $k0, $k1, AdE  

	# It must be another type of exception
	beq $zero, $zero, otherExceptions

systemCall:
	# Branch based on the value of v0
	addi $k1, $zero, 0xA
	beq $v0, $k1, vZeroA
	
	addi $k1, $zero, 0x20
	beq $v0, $k1, vZero20
	
	addi $k1, $zero, 0x21	
	beq $v0, $k1, vZero21

	# It must be another type of syscall
	beq $zero, $zero, otherExceptions

vZeroA:
	# Halt with TESTDONE instruction
	.word 0xE	
vZero20:
	# $k0 == EPC
	mfc0 $k0, $14
	
	# EPC += 4
	addi $k0, $k0, 4
	mtc0 $k0, $14

	# Store the value of v1 in memory
	lui	$k1, 0x90000000
	
	# $k0 ==  "stack" pointer
	lw $k0, 0($k1)
	# Increment
	addi $k0, $k0, 4

	# Store v1, and restore SP into 90000000
	sw $v1, 0($k0)
	
	sw $k0, 0($k1)

	# Return with ERET		
	.word 0x42000018
vZero21:
	# $k0 == EPC
	mfc0 $k0, $14
	
	# EPC += 4
	addi $k0, $k0, 4
	mtc0 $k0, $14

	# Load the value of "stack" into a0
	lui	$k1, 0x90000000
	
	# $k0 == "stack" pointer
	lw $k0, 0($k1)
	
	# Load a0
	lw $a0, 0($k0)

	# Decrement stack pointer
	addi $k0, $k0, -4
	# Restore into memory
	sw $k0, 0($k1)	

	# Return with ERET		
	.word 0x42000018

AdE:
	# $k0 == EPC
	mfc0 $k0, $14
	
	# EPC += 4
	addi $k0, $k0, 4
	mtc0 $k0, $14
	
	# Return with ERET		
	.word 0x42000018

otherExceptions:
	# cause code is in k0

	# Put 0xDEADC0DE in k1
	addi $k1, $zero, 0
	lui $k1, 0xDEAD
	addi $k1, $k1, 0xC0DE

	# Halt with TESTDONE instruction
	.word 0xE

# User level Code
.text
	addi $6, $zero, 0x1337
	addi $v0, $zero, 0x20
	addi $v1, $zero, 0xaa
	syscall
	addi $v1, $zero, 0xbb
	syscall
	
	addi $v0, $zero, 0x21
	syscall
	addi $11, $a0, 0
	syscall
	addi $10, $a0, 0

	addi $7, $zero, 0x1337
	addi $v0, $zero, 0x20
	addi $v1, $zero, 0x11
	syscall
	addi $v1, $zero, 0x22
	syscall
	addi $v1, $zero, 0x33
	syscall
	addi $v1, $zero, 0x44
	syscall	

	addi $v0, $zero, 0x21
	syscall
	addi $12, $a0, 0
	syscall
	addi $13, $a0, 0
	syscall
	addi $14, $a0, 0
	syscall
	addi $15, $a0, 0

	
	# Done
	addi $v0, $zero, 0xa
	syscall
