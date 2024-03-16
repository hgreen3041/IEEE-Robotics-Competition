from machine import I2C, Pin

scanner = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)



devAddr = scanner.scan()

for address in devAddr: 
    print(hex(address))




