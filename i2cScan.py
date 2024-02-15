from machine import I2C, Pin

scanner = I2C(1, scl=Pin(15), sda=Pin(14), freq=400000)



devAddr = scanner.scan()

for address in devAddr: 
    print(hex(address))




