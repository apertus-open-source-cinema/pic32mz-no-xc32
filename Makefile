MIPSGNU ?= mipsel-none-elf
LIBCINC ?= /gnu/store/hz9ykcpkwm5p21pilkgx6snkg9z4blyi-newlib-nano-2.4.0/mipsel-none-elf/include/
LIBCLINK ?= /gnu/store/hz9ykcpkwm5p21pilkgx6snkg9z4blyi-newlib-nano-2.4.0/mipsel-none-elf/lib/

CFLAGS = -msoft-float -EL -march=m14k -G0 -Wall -Wno-attributes -nostartfiles -ffreestanding -IVendor -I$(LIBCINC)
ASFLAGS = -msoft-float -EL -G0 -march=m14k -x assembler-with-cpp -IVendor -nostartfiles -ffreestanding -nostdlib
LDFLAGS = -EL -G0 -L$(LIBCLINK) -Tp32MZ2048ECG100.ld

all : usb.hex lcd.hex

clean :
	rm -f *.o
	rm -f *.hex
	rm -f *.elf
	rm -f *.bc

%.o : %.S
	$(MIPSGNU)-gcc $(ASFLAGS) -c $< -o $@

%.o : %.c
	$(MIPSGNU)-gcc $(CFLAGS) -c $< -o $@

%.hex : %.elf
	$(MIPSGNU)-objcopy $< -O ihex $@

usb.elf : init_interrupts.o startup_usb.o startup.o usb.o p32MZ2048ECG100.o libpic32.a
	$(MIPSGNU)-ld $(LDFLAGS) $^ -o $@

lcd.elf : init_interrupts.o startup_lcd.o startup.o lcd.o p32MZ2048ECG100.o libpic32.a
	$(MIPSGNU)-ld $(LDFLAGS) $^ -o $@
