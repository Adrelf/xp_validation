import minimalmodbus

# Configuration de l'instrument
instrument = minimalmodbus.Instrument('/dev/ttyUSB1', 255)
instrument.serial.baudrate = 9600
instrument.serial.timeout = 0.2
instrument.debug = True

try:
    mmHg = 40
    # Conversion mmHg -> Pa
    value_pa = mmHg * 133.322
    instrument.write_float(154, value_pa)

    # Puis demander 100 % de cette consigne (registre 600139)
    instrument.write_register(138, 16384, functioncode=6)

except Exception as e:
    print(f"Erreur Modbus : {e}")
