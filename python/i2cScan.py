from machine import I2C, Pin

scanner = I2C(0, scl=Pin(17), sda=Pin(16), freq=400000)



devAddr = scanner.scan()

for address in devAddr: 
    print(hex(address))




