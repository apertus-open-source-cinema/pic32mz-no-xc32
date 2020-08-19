from intelhex import IntelHex16bit

ih = IntelHex16bit()

ih.loadhex('usb.hex')

# ih.dump()
for addr in ih.todict().keys():
    print("0x{:08X}".format(addr))
# print(ih.todict()[0x1F810788])
