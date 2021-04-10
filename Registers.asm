.model tiny
.code
org 100h

;;
;   COLOR CONSTANT
;;
color_frame      = 04eh
color_inside     = 01A00h
right_high_angle = 0bbh
left_high_angle  = 0c9h
left_low_angle   = 0c8h
right_low_angle  = 0bch

horizontal_line  = 0cdh
vertical_line    = 0bah
color_clear      = 0020h

;;
;   SIZE CONSTANT
;;

size_console = 25 * 80 

ymin = 3    ;cx
ymax = 17
xmin = 64   ;dx
xmax = 79
vram_ptr = 0b800h

;;
;   INTERRUPT CONSTANT
;;
int08 = 08h
int09 = 09h

;====================================================================================================
;====================================================================================================
;=========================================== MAIN ===================================================
;====================================================================================================
;====================================================================================================

start:

    mov ax, int08
    mov di, offset Old08
    mov cx, offset Register_updater
    call New_interrupt

    mov ax, int09
    mov di, offset Old09
    mov cx, offset Register_toggle
    call New_interrupt

    mov ax, 3100h
    mov dx, offset END_OF_HALL
    shr dx, 4
    inc dx
    int 21h
;======================================================


;====================================================================================================
;====================================================================================================
;====================================== NEW INTERRUPTS ==============================================
;====================================================================================================
;====================================================================================================

;------------------------------------------------------
; Merge custom interrupt 
; Entry: ax - number of interrupt
;        di - pointer to address with old interrupt 
;        cx - pointer to new interrupt
;
; Destr: ax, bx, di, es
;------------------------------------------------------

New_interrupt proc
    xor bx, bx
    mov es, bx
    shl ax, 2    
    mov bx, ax

    cli

    mov ax, es:[bx]
    mov [di], ax
    mov ax, es:[bx + 2]
    mov [di + 2], ax

   
    mov es:[bx], cx
    mov es:[bx + 2], cs
    sti

    ret
endp
;======================================================

;------------------------------------------------------
; New custom interrupt 09h 
; Toggle frame with all register
;
; Button "on" : "Z"
;
; Destr: save all registers
;------------------------------------------------------

Register_toggle proc
    push ax bx cx dx si di es ds

    mov bx, offset flag_on

    mov ah, 4eh
    in al, 60h

    cmp al, 2ch  ; button "Z" pressed
    je flag_update

    cmp al, 23h  ;  button "H" pressed
    je flag_hex

    ; cmp al, 0ach ; button "Z" released
    jne Standart_int09

flag_update:

    mov cl, byte ptr cs:[bx]

    cmp cl, 0
    je Start_on

    push ax bx cx dx si di es ds
    call Fill_new_vram_of_old_vram
    pop ds es di si dx cx bx ax

    mov byte ptr cs:[bx], 0
    jmp Standart_int09

Start_on: 
    push ax bx cx dx si di es ds
    call Fill_old_vram_of_new_vram
    pop ds es di si dx cx bx ax
    
    mov byte ptr cs:[bx], 1
    jmp Standart_int09

flag_hex:
    mov bx, offset flag_system
    mov cl, byte ptr cs:[bx]

    cmp cl, 0
    je flag_hex_on

    mov byte ptr cs:[bx], 0
    jmp Standart_int09

flag_hex_on:
    mov byte ptr cs:[bx], 1
    jmp Standart_int09


Standart_int09:
				
    pop ds es di si dx cx bx ax
                        
                        ; JMP FAR to old 09h interrupt
                        db 0eah
    Old09				dd 0

endp
;======================================================

;------------------------------------------------------
; New custom interrupt 08h 
; If flag_on == 1, then create frame with registers and it values
;
;
; Destr: save all registers
;------------------------------------------------------

Register_updater proc
    push ax bx cx dx si di es ds

    push cs
    pop ds

    cmp flag_on, 1
    jne Standart_int08

    push ax bx cx dx si di es ds
    call Frame_create
    pop ds es di si dx cx bx ax
    
    push ax bx cx dx si di es ds
    call Print_registers
    pop ds es di si dx cx bx ax

  

Standart_int08:
    
    pop ds es di si dx cx bx ax

                        ; JMP FAR to old 08h interrupt
                        db 0eah
    Old08				dd 0
   
    
endp
;======================================================

;====================================================================================================
;====================================================================================================
;===================================== SCREEN PROC ==================================================
;====================================================================================================
;====================================================================================================

;------------------------------------------------------
; Copy frame from old vram to current vram. 
;
; Destr: ax, bx, cx, dx, di, es 
;------------------------------------------------------

Fill_new_vram_of_old_vram proc
    mov bx, vram_ptr
    mov es, bx

    mov bx, offset old_vram
    mov cx, 0
    mov dx, 0

new_string:

    mov ax, ymin
    add ax, cx

    push bx
    push dx
    mov bx, 80
    mul bx
    pop dx
    pop bx

    add ax, xmin
    add ax, dx
    shl ax, 1
    mov di, ax

print_string:

    mov ax, word ptr cs:[bx]
    mov word ptr es:[di], ax

    add bx, 2
    add di, 2
    inc dx

    cmp dx, xmax - xmin + 1
    jne print_string

    inc cx
    mov dx, 0

    cmp cx, ymax - ymin + 1
    jne new_string

    ret
endp 
;======================================================

;------------------------------------------------------
; Copy frame from current vram to temporary (old) vram. 
;
; Destr: ax, bx, cx, dx, di, es 
;------------------------------------------------------

Fill_old_vram_of_new_vram proc
    mov bx, vram_ptr
    mov es, bx

    lea bx, old_vram
    mov cx, 0
    mov dx, 0

new_string2:

    mov ax, ymin
    add ax, cx

    push bx
    push dx
    mov bx, 80
    mul bx
    pop dx
    pop bx

    add ax, xmin
    add ax, dx
    shl ax, 1
    mov di, ax

print_string2:

    mov ax, word ptr es:[di]
    mov word ptr cs:[bx], ax

    add bx, 2
    add di, 2
    inc dx

    cmp dx, xmax - xmin + 1
    jne print_string2

    inc cx
    mov dx, 0

    cmp cx, ymax - ymin + 1
    jne new_string2

    ret
endp 
;======================================================

;------------------------------------------------------
; Create window.
; 
; Destr: ax, bx, cx, dx, es, di
;------------------------------------------------------

Frame_create proc 
    mov bx, vram_ptr
    mov es, bx

    mov ah, color_frame
    mov al, left_low_angle
    mov cx, ymax
    mov dx, xmin
    call Angle


    mov ah, color_frame
    mov al, right_high_angle
    mov cx, ymin
    mov dx, xmax
    call Angle

    mov cx, 1
    mov dx, 1

Full:

    push dx
    push cx
    call R_vertical
    pop cx
    pop dx

    push dx
    push cx
    call L_vertical
    pop cx
    pop dx

Horizontal:

    push dx
    push cx
    call H_horizontal
    pop cx
    pop dx

    push dx
    push cx
    call L_horizontal
    pop cx
    pop dx

    inc cx
    inc dx

    cmp cx, ymax - ymin - 1
    jbe Full

    cmp dx, xmax - xmin - 1
    jbe Horizontal

    mov ah, color_frame
    mov al, left_high_angle
    mov cx, ymin
    mov dx, xmin
    call Angle

    mov ah, color_frame
    mov al, right_low_angle
    mov cx, ymax
    mov dx, xmax
    call Angle  

    call Screen_fill

    ret
    endp
;======================================================

;------------------------------------------------------
; Create empty console. Size of console: 80 x 25 
; Entry: es - ptr to 0b800h
;
; Destr: ax, cx. di
;------------------------------------------------------

Screen_clear proc

    mov di, 0
    mov ax, color_clear
    mov cx, size_console
    cld
    rep stosw

    ret
    endp 
;======================================================

;------------------------------------------------------
; Fill the inside of the frame (from (xmin + 1; ymin + 1) to (xmax - 1; ymax - 1)). 
; Entry: es - ptr to 0b800h
;
; Destr: ax, cx. di
;------------------------------------------------------

Screen_fill proc
    mov cx, 0
    mov ax, color_inside or ' '  
    mov di, ((ymin + 1) * 80 + xmin + 1) * 2

string:
    push cx

    mov cx, xmax - xmin - 1
    cld 
    rep stosw

    add di, 80 * 2
    sub di, (xmax - xmin - 1) * 2
    pop cx
    inc cx

    cmp cx, ymax - ymin - 1
    jne string

    ret 
    endp
;======================================================

;------------------------------------------------------
; Draw point of low horizontal line from (xmin, ymax) to (xmax, ymax)
; Entry: cx - shift y0
;        dx - shift x0
;
;        es - ptr to 0b800h
;
; Destr: ax, bx, cx, dx 
;------------------------------------------------------

L_horizontal proc
     
    mov ax, ymax
    mov bx, 80

    push dx
    mul bx
    pop dx

    add ax, xmin
    add ax, dx

    shl ax, 1
    mov bx, ax

    mov byte ptr es:[bx], horizontal_line
    mov byte ptr es:[bx + 1], color_frame

    ret
    endp
;======================================================

;------------------------------------------------------
; Draw point of high horizontal line from (xmax, ymin) to (xmin, ymin)
; Entry: cx - shift y0
;        dx - shift x0
;
;        es - ptr to 0b800h
;
; Destr: ax, bx, cx, dx 
;------------------------------------------------------

H_horizontal proc
    
    mov ax, ymin
    mov bx, 80

    push dx
    mul bx
    pop dx

    add ax, xmax
    sub ax, dx
    shl ax, 1

    mov bx, ax

    mov byte ptr es:[bx], horizontal_line
    mov byte ptr es:[bx + 1], color_frame

    ret
    endp
;======================================================

;------------------------------------------------------
; Draw point of left vertical line from (xmin, ymax) to (xmin, ymin)
; Entry: cx - shift y0
;        dx - shift x0
;
;        es - ptr to 0b800h
;
; Destr: ax, bx, cx, dx 
;------------------------------------------------------

L_vertical proc
   
    mov ax, ymax
    sub ax, cx
    mov bx, 80

    push dx
    mul bx
    pop dx

    add ax, xmin
    shl ax, 1
    mov bx, ax

    mov byte ptr es:[bx], vertical_line
    mov byte ptr es:[bx + 1], color_frame

    ret
    endp
;======================================================

;------------------------------------------------------
; Draw point of right vertical line from (xmax, ymin) to (xmax, ymax)
; Entry: cx - shift y0
;        dx - shift x0
;
;        es - ptr to 0b800h
;
; Destr: ax, bx, cx, dx 
;------------------------------------------------------

R_vertical proc
    
    mov ax, ymin
    add ax, cx
    mov bx, 80

    push dx
    mul bx
    pop dx

    add ax, xmax
    shl ax, 1
    mov bx, ax

    mov byte ptr es:[bx], vertical_line
    mov byte ptr es:[bx + 1], color_frame

    ret
    endp
;======================================================

;------------------------------------------------------
; Draw char with (x0, y0)
; Entry: ax - char
;        cx - y0
;        dx - x0
;
;        es - ptr to 0b800h
;
; Destr: ax, bx, cx, dx 
;------------------------------------------------------

Angle proc
    push ax
    push dx

    mov ax, cx
    mov bx, 80
    mul bx
    
    pop dx
    add ax, dx
    shl ax, 1
    mov bx, ax

    pop ax
    
    mov byte ptr es:[bx], al
    mov byte ptr es:[bx + 1], ah

    ret
    endp
;=======================================================

;====================================================================================================
;====================================================================================================
;=================================== PRINT REGISTERS ================================================
;====================================================================================================
;====================================================================================================

;------------------------------------------------------
; Print ax, bx, cx, dx, di, es and it values in frame
;
;
; Destr: ax, bx, cx, dx, di, es
;-------------------------------------------------------

Print_registers proc 

    push ax bx cx dx di es

    mov di, ((ymin + 2) * 80 + (xmin + 2)) * 2 

    IRP regs, <ax, bx, cx, dx, di, es>
        lea si, msg_reg_&regs
        mov cx, 10
        call Print_base

        add di, 80 * 4 - 20
    ENDM

    mov si, ((ymax - 2) * 80 + (xmin + 7)) * 2

    IRP regs, <es, di, dx, cx, bx, ax>

        pop ax

        mov bx, offset flag_system
        mov cl, byte ptr cs:[bx]
        cmp cl, 0
        je Dec_print_&regs


        mov cl, 'h'
        mov di, 1111b
        mov dx, 4

        call Hex_convert
        jmp End_print_&regs

Dec_print_&regs:

        mov cl, 'd'
        mov di, 10
        call Dec_convert
        jmp End_print_&regs

End_print_&regs:
        sub si, 80 * 4
    ENDM
    ret
endp    
;=====================================================

;------------------------------------------------------
; Print dec (or other system foundation in register di) representation of number located in ax
; Print begin with point on address (in bx)
; Use jmp to Print (label in Dec_convert proc)
; Entry: ax - our number
;        si - shift vram to print
;        es - ptr to 0b800h
;        di - system foundation
;        cl - symbol of system 
;
; Destr: ax, bx, cx. di, dl, si
;------------------------------------------------------

Dec_convert proc

    add si, 10
    mov byte ptr es:[si], cl
    mov byte ptr es:[si + 1], 01Ah
    sub si, 10

    mov bx, 0

Deg: 

    mov dx, 0
    mov cx, di
    div cx
    push dx
    inc bx
   
    cmp ax, 0
    jne Deg

    mov cx, bx
    mov bx, si
    add bx, 10
    sub bx, cx
    sub bx, cx


;------------------------------------------------------
; Print the digits from Stack
; Entry: cx - number of digits
;        es - ptr to 0b800h (video ram)
;        bx - offset to the print location
;
; Destr: cx, di, bx
;------------------------------------------------------

Print:
    
    pop di
    mov dl, byte ptr cs:[offset System + di]

    mov byte ptr es:[bx], dl
    mov byte ptr es:[bx + 1], 01Ah
    add bx, 2
    
    dec cx
    cmp cx, 0
    jne Print

    ret
    endp
;======================================================

;------------------------------------------------------
; Print hex (or other system foundation pow 2) representation of number located in ax
; Print begin with point on address (in bx)
; Use jmp to Print (label in Dec_convert proc)
; Entry: ax - our number
;        si - shift vram to print
;        es - ptr to 0b800h
;        di - byte mask
;        dx - byte shift 
;        cl - symbol of system 
;
; Destr: ax, bx, cx. di, dl, si
;------------------------------------------------------

Hex_convert proc

    add si, 10
    mov byte ptr es:[si], cl
    mov byte ptr es:[si + 1], 01Ah
    sub si, 10

    mov cx, 0

Hex: 
    mov bx, ax
    and bx, di
    push bx
    push cx
    mov cl, dl
    shr ax, cl
    pop cx
    inc cx
    
    cmp ax, 0
    jne Hex

    mov bx, si
    add bx, 10
    sub bx, cx
    sub bx, cx
    jmp Print

    ret
    endp
;==================================================

;------------------------------------------------------
; Print print base for values registers
; Print begin with point on address (in di)
; Entry: si - pointer to our buffer base 
;        es - ptr to 0b800h
;        cx - number of symbols in strings
;
; Destr: ax, bx, cx. di, dx, si
;------------------------------------------------------

Print_base proc
    mov bx, 0b800h
    mov es, bx

Print_base_reg:
    mov al, byte ptr cs:[si]
    mov byte ptr es:[di], al
    inc si
    add di, 2
    dec cx

    cmp cx, 0
    jne Print_base_reg

    ret
endp
;=======================================================

;;
;   FLAG FOR CUSTUM INTERRUPTS
;;
flag_on     db 0
flag_system db 0

;;
;   BUFFERS FOR BASE PRINT REGISTERS
;;
msg_reg_ax db "ax = 00000 $"
msg_reg_bx db "bx = 00000 $"
msg_reg_cx db "cx = 00000 $"
msg_reg_dx db "dx = 00000 $"
msg_reg_di db "di = 00000 $"
msg_reg_es db "es = 00000 $"


;;
;   STRING FOR QUICK CONVERTETION 
;;
System db '0123456789ABCDEF$'

;;
;   OLD VRAM
;;
old_vram dw (ymax - ymin + 1) * (xmax - xmin + 1) dup (?)

END_OF_HALL:
end start


