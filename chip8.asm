; chip8.asm
;
; nasm -f elf chip8.asm
; gcc -o chip8 chip8.o

; opcode: value of opcode
; address: value of address
%macro DEBUG_OP 0
	push eax
	push ebx
	movzx ebx, word [I]
	movzx eax, word [inst]
	push dword ebx
	push dword eax
	push dword [PC]
	push dword unk_op_str
	call printf
	add esp, 16
	pop ebx
	pop eax
%endmacro	

%macro PRINT_REGS 0
	push eax
	mov ecx, 16
%%print_loop
	movzx eax, byte [V+ecx-1]
	push dword eax
	dec ecx
	cmp ecx, 0
	jnz %%print_loop
	push dword print_reg
	call printf
	add esp, 68
	pop eax
%endmacro

segment .data

%include "chip8_rom.inc"

opcodes			dd cpu_misc0
dd cpu_jp
				dd cpu_call
				dd cpu_skeq_im
				dd cpu_skne_im
				dd cpu_skeq_re
				dd cpu_mov
				dd cpu_add
				dd cpu_math
				dd cpu_skne_re
				dd cpu_mvi
				dd cpu_jmi
				dd cpu_rand
				dd cpu_draw
				dd cpu_keys
				dd cpu_miscF

math_codes		dd math_mov
				dd math_or
				dd math_and
				dd math_xor
				dd math_add
				dd math_sub
				dd math_shr
				dd math_rsb
				dd math_shl

unk_op_str		db "PC=0%x: %x (I: %x)", 10, 0
print_reg		db " Registers: %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x %.02x", 10, 0
print_xy		db "x: %x y: %x", 10, 0
print_str		db "im here", 10, 0
debug_str		db "debug: %x", 10, 0
romfile			db "kaleid.c8", 0
filemode		db 'r', 0
misc0_str		db "misc0", 10, 0
jp_str			db "jp", 10, 0
call_str		db "call", 10, 0
skeq_im_str		db "skeq_im", 10, 0
skne_im_str		db "skne_im", 10, 0
skeq_re_str		db "skeq_re_str", 10, 0
mov_str			db "mov", 10, 0
add_str			db "add", 10, 0
math_str		db "math", 10, 0
skne_re_str		db "skne_re", 10, 0
mvi_str			db "mvi", 10, 0
jmi_str			db "jmi", 10, 0
rand_str		db "rand", 10, 0
draw_str		db "draw", 10, 0
keys_str		db "keys", 10, 0
miscF_str		db "miscF", 10, 0
blank_str		db 10, 0
display_char_str	db "X", 0
display_blank_str	db " ", 0

%define			DISPLAY_CHAR	'X'
%define			DISPLAY_BLANK	' '
%define			DISPLAY_Y	32
%define			DISPLAY_X	64

segment .bss


;struc cpu
V				resb 16
I				resw 1
delay			resb 1
sound			resb 1
SP_w			resw 1
PC				resw 1
step			resb 1
showdebug		resb 1
RAM 			resb 0xFFF
key 			resb 16
schip 			resb 1
display 		resb 64*128
;endstruc
x_offset		resd 1
y_offset		resd 1

inst			resw 1
fp				resd 1
rx				resb 1		; V[x]
ry				resb 1		; V[y]

x			resb 1
y			resb 1
xo			resb 1
xy			resb 1

segment .text
	global main
	extern printf
	extern fread
	extern fopen
	extern fclose
	extern puts
	extern putchar

main:
	push ebp
	mov ebp, esp
	push ebx
	push esi
	push edi
	;;;

	; setup default values
	call reset_cpu
	; load rom image
	call load_rom

main_loop:
	xor eax, eax
	mov edi, RAM
	movzx ebx, word [PC]
	add edi, ebx
	inc edi					; RAM+PC+1
	mov esi, RAM
	movzx ebx, word [PC]
	add esi, ebx			; RAM+PC
	mov ah, byte [esi]
	mov al, byte [edi]
	mov word [inst], ax		; save it
	DEBUG_OP
	xor ebx, ebx
	mov bx, ax				; gonna fill X
	and bx, 0xF00
	shr bx, 8
	mov byte [rx], bl

	mov bx, ax				; gonna fill Y
	and bx, 0x0F0
	shr bx, 4
	mov byte [ry], bl

	PRINT_REGS
	and ax, 0xF000			; get that first part alone
	shr ax, 12				; set ax to that first part
	call dword [opcodes+eax*4]
	call display_screen

	jmp main_loop

end_main:

	;;;
	pop edi
	pop esi
	pop ebx
	mov esp, ebp
	pop ebp
	ret

display_screen:
	xor ecx, ecx
	xor ebx, ebx

	mov cl, DISPLAY_Y
.display_y
	push ecx

	mov bl, DISPLAY_X
.display_x
	push ebx

	; see if we should display it
	xor eax, eax
	xor edx, edx
	mov eax, ecx
	mov edx, DISPLAY_Y
	mul edx
	add eax, ebx
	mov al, byte [display+eax]
	cmp al, 0
	jz .blank_char

	push dword display_char_str
	call printf
	add esp, 4
	jmp .next_step

.blank_char
	push dword display_blank_str
	call printf
	add esp, 4
    
.next_step
	pop ebx
	dec bl
	jnz .display_x

	push dword blank_str
	call printf
	add esp, 4
	
	pop ecx
	loop .display_y
	ret

reset_cpu:
	mov word [PC], 0x200			; start execution of rom at 0x200
	mov word [I], 0
	mov word [SP_w], 0x1E0			; stack, but why 0x1E0

	; zero out V[0-F], replace this with string functions
	mov ecx, 15
.zero_V	
	mov byte [V+ecx], 0
	dec ecx
	cmp ecx, 0
	jne .zero_V

	; fill ram from 0-0x200 with chip8_rom
	cld								; set direction bit
	xor ecx, ecx					; pos in RAM
	xor eax, eax					; gonna use al
	mov edi, RAM
.fill_ram	
	mov al, byte [chip8_rom+ecx]
	stosb
	inc ecx
	cmp ecx, 0x200
	jne .fill_ram

	; zero display
	cld								; set direction bit
	mov ecx, DISPLAY_X*DISPLAY_Y					; display size
	mov edi, display
	xor al, al						; fill it with zeros
	rep stosb	
	
	mov byte [delay], 0
	mov byte [sound], 0
	mov byte [step], 0
	mov byte [showdebug], 0
	mov byte [schip], 0
	; clear display
	ret

load_rom:
	push dword filemode
	push dword romfile
	call fopen
	add esp, 8
	mov dword [fp], eax		; save file pointer
	push eax				; eax has filehandle
	push dword 4096			; the size of RAM
	push dword 1			; 1 byte at a time
	push dword RAM+0x200	; start after chip8_rom
	call fread
	add esp, 16
	mov eax, dword [fp]
	push eax
	call fclose
	add esp, 4
	ret

cpu_misc0:
	push dword misc0_str
	call printf
	add esp, 4
	ret

cpu_jp:
	push dword jp_str
	call printf
	add esp, 4
	ret

cpu_call:
	push dword call_str
	call printf
	add esp, 4

	mov ax, word [PC]
	shr ax, 8
	xor ebx, ebx
	mov bx, word [SP_w]
	mov [RAM+ebx], eax
	mov ax, word [PC]
	and ax, 0xFF
	mov [RAM+ebx+1], eax
	add word [SP_w], 2
	mov ax, word [inst]
	and ax, 0x0FFF
	mov word [PC], ax

	ret

cpu_skeq_im:
	push dword skeq_im_str
	call printf
	add esp, 4

	mov ax, word [inst]
	and ax, 0x0FF
	cmp byte [rx], al
	jne .skip_next_inst
	add word [PC], 2
	ret
.skip_next_inst
	add word [PC], 4
	ret

cpu_skne_im:				; 4XKK	Skip next instruction if VX != KK
	push dword skne_im_str
	call printf
	add esp, 4

	mov ax, word [inst]
	and ax, 0x0FF
	cmp byte [rx], al
	jne .skip_next_inst
	add word [PC], 2
	ret
.skip_next_inst
	add word [PC], 4
	ret

cpu_skeq_re:
	push dword skeq_re_str
	call printf
	add esp, 4

	xor eax, eax
	mov al, byte [rx]
	cmp al, byte [ry]
	jne .skip_next_inst
	add word [PC], 2
	ret
.skip_next_inst
	add word [PC], 4
	ret

cpu_mov:
	push dword mov_str
	call printf
	add esp, 4

	xor eax, eax
	xor ebx, ebx
	mov ax, word [inst]
	and ax, 0xFF
	mov bl, byte [rx]
	mov byte [V+ebx], al

	add word [PC], 2
	ret

cpu_add:				; 7XKK   VX = VX + KK
	push dword add_str
	call printf
	add esp, 4
	
	xor eax, eax
	xor ebx, ebx
	mov ax, word [inst]
	and ax, 0xFF
	add byte [rx], al

	add word [PC], 2
	ret

cpu_math:
	push dword math_str
	call printf
	add esp, 4

	xor edx,edx
	mov dx, word [inst]
	and dx, 0xF
	call dword [math_codes+edx*4]

	add word [PC], 2
	ret

math_mov:
	xor eax, eax
	xor ebx, ebx
	xor ecx, ecx
	mov al, byte [rx]
	mov bl, byte [ry]
	mov cl, byte [V+ebx]
	mov byte [V+eax], cl
	ret

math_or:
	ret

math_and:
	xor eax, eax
	xor ecx, ecx
	xor edx, edx
	mov al, byte [rx]
	mov cl, byte [ry]
	mov dl, byte [V+ecx]
	and byte [V+eax], dl
	ret

math_xor:
	ret

math_add:
	xor eax, eax
	xor ebx, ebx
	mov al, byte [rx]
	mov bl, byte [ry]
	add ax, bx			; final result should be word for shift
	mov byte [rx], al
	shr ax, 8
	mov byte [V+0xF], al
	ret

math_sub:
	xor eax, eax
	xor ebx, ebx
	xor ecx, ecx

	mov al, byte [rx]
	mov bl, byte [V+eax]

	mov al, byte [ry]
	mov cl, byte [V+eax]

	sub bl, cl

	mov al, byte [rx]
	mov byte [V+eax], bl

	shr ax, 8
	inc ax
	mov byte [V+0xF], al
	ret

math_shr:
	ret

math_rsb:
	ret

math_shl:
	ret
	
cpu_skne_re:
	push dword skne_re_str
	call printf
	add esp, 4
	ret

cpu_mvi:
	push dword mvi_str
	call printf
	add esp, 4

	mov ax, word [inst]
	and ax, 0x0FFF
	mov word [I], ax
	
	add word [PC], 2
	ret

cpu_jmi:
	push dword jmi_str
	call printf
	add esp, 4
	ret

cpu_rand:
	push dword rand_str
	call printf
	add esp, 4
	ret

cpu_draw:
	push dword draw_str
	call printf
	add esp, 4

	xor ebx,ebx
	xor eax,eax

	; get the height first, in cx
	mov cx, word [inst]
	and cx, 0xF

	; ax holds y
	mov ax, 0

.y_loop
	mov bx, 0		; bx holds x
.x_loop
	call cpu_sprite

	;pop dx
	;pop cx

	inc bx
	cmp bx, 8
	jnz .x_loop
	; end x loop

	inc ax
	cmp ax, cx
	jnz .y_loop
	; end y loop
	
	add word [PC], 2
	ret
;;;;;;;;;;;;;;;;;;;;;

; ax: y
; bx: x
cpu_sprite:
	push eax
	push ebx

	xor ecx, ecx

	mov cx, ax
	add cl, byte [ry]

	; y+yo % vid_h
	mov eax, ecx
	push ecx
	mov cx, DISPLAY_Y
	div cx
	pop ecx
	add cx, dx

	mov eax, ecx
	mov edx, 64
	mul edx

	push eax

	xor ecx, ecx
	mov cx, bx
	add cl, byte [rx]

	; x + xo % vid_h
	mov eax, ecx
	push ecx
	mov cx, DISPLAY_X
	div cx
	pop ecx
	add cx, dx

	mov eax, ecx
	mov edx, 128
	mul edx

	mov edx, display
	add edx, eax
	pop eax
	add edx, eax
	mov dl, byte [edx] ; edx now holds d
	
	ret

;	mov ebx, dword RAM	; get byte offset
;	mov ax, word [I]
;	pop ecx
;	add ax, cx
;	push ecx
;	add edx, eax
;	xor ecx, ecx
;	mov cl, byte [edx]
;	xor edx, edx
;	mov dl, cl
;	xor ecx, ecx

	xor eax, eax
	xor ebx, ebx
	mov bx, dx		; x var
	sub bx, 7
	mov ax, 1
	push ecx
	mov cl, bl
	shl ax, cl
	pop ecx
	
	mov ebx, dword RAM
	add bx, word [I]
	add bx, cx		; ebx has left side of AND
    
	and eax, ebx		; eax has char for display

	xor ebx, ebx
	mov ebx, display
	;add	    ; XXXX I'm here, gotta do display[(y+yo)*COLS][x+xo]

	mov cl, byte [ry]
	mov bl, byte [rx]
	mov edx, DISPLAY_Y
	mul edx
	add eax, ebx
	;mov al, byte [display+eax]

	pop ebx
	dec bx
	;jnz .width_loop

	pop ecx
	;loop .height_loop
	
	add word [PC], 2
	ret

cpu_keys:
	push dword keys_str
	call printf
	add esp, 4
	ret

cpu_miscF:
	push dword miscF_str
	call printf
	add esp, 4
	ret
