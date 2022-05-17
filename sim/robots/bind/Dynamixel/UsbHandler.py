import usb
busses = usb.busses()
for bus in busses:
    devices = bus.devices
    for dev in devices:
        #print("Device:", dev.filename)
        #print("  idVendor: %d (0x%04x)" % (dev.idVendor, dev.idVendor))
        print("  idProduct: %d (0x%04x)" % (dev.idProduct, dev.idProduct))


import usb.core as core

import usb.core

devices = usb.core.find(find_all=True)

dev = next(devices)

print("device bus:", dev.bus)
print("device address:", dev.address)
print("device port:", dev.port_number)
print("device speed:", dev.speed)


dev = core.find(idProduct=24596)

print(dev)
