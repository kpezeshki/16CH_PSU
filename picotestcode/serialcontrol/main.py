from machine import Pin, SPI
import math
import time
import select
import sys

ADC_channel_lookup = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
DAC_channel_lookup = [1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14]
DAC_status         = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0,  0,  0,  0]  # change to 1 if DAC is outputting anything

led_config = Pin(16, Pin.OUT)
led_output_on = Pin(17, Pin.OUT)
led_daccomm = Pin(18, Pin.OUT)
led_sercomm = Pin(19, Pin.OUT)

button_all_off = Pin(20, Pin.IN)
button_dac_rst = Pin(21, Pin.IN)
button_fun = Pin(22, Pin.IN)

class AMC7836:

    def setup_AMC7836(self):
        self.alarmin.value(1)
        
        self.reset.value(0)
        time.sleep(0.1)
        self.reset.value(1)
        
        self.cs.value(1)  # Set CS high initially
        
        # configure device for single instruction, read back from active registers
        self.write_register(0x01, 0x80)
        # turn off low power mode
        self.write_register(0x02, 0x00)
        # set range to 0 to 5 V
        range_10V = 0b01100110
        range_5V  = 0b01110111
        self.write_register(0x1E, range_10V)
        self.write_register(0x1F, range_10V)
        # enable internal reference
        self.write_register(0x10, 0x10)
        # enable all ADCs
        self.write_register(0x13,0xFF)
        self.write_register(0x14,0xFF)
        self.write_register(0x15,0x3F)
        
        time.sleep(0.1)     

    def __init__(self, spi, cs, reset, alarmin):
        # Initialize SPI
        self.reset = reset
        self.spi = spi
        self.cs = cs
        self.alarmin = alarmin

        self.setup_AMC7836()

    def _transfer(self, tx_data, rx=True):
        led_daccomm.value(1)
        self.cs.value(0)  # Assert CS
        rx_data = bytearray(1)
        self.spi.write(tx_data)
        if rx:
            self.spi.readinto(rx_data)
        self.cs.value(1)  # De-assert CS
        led_daccomm.value(0)
        return rx_data

    def write_register(self, address, value):
        tx_data = bytearray([0x00 | (address >> 8 & 0x7F), address & 0xFF, value])
        self._transfer(tx_data, rx=False)
        time.sleep(0.001)

    def read_register(self, address):
        tx_data = bytearray([0x80 | (address >> 8 & 0x7F), address & 0xFF])
        rx_data = self._transfer(tx_data)
        return rx_data[0]
        time.sleep(0.001)

    def set_dac(self, dac_number, value):
        
        if value > 0:
            DAC_status[dac_number] = 1
        else:
            DAC_status[dac_number] = 0
        
        if 0 <= dac_number <= 15 and 0 <= value <= 4095:
            dac_address_low_byte = 0x50 + (dac_number * 2)  # Even address for low byte
            dac_address_high_byte = dac_address_low_byte + 1
            
            low_byte = value & 0xFF
            high_byte = (value >> 8) & 0xFF
            
            self.write_register(dac_address_low_byte, low_byte)
            self.write_register(dac_address_high_byte, high_byte)
            
            # Update DAC output
            self.write_register(0x0F, 0x01)
        else:
            print("Invalid DAC number or value")
            
    def set_dac_voltage(self, dac_number, voltage):
        # assumes 10 V range
        dac_code = int(voltage/10*4095)
        self.set_dac(dac_number, dac_code)
        
    def set_output_voltage(self, dac_number, voltage):
        # assumes noninverting amplifier with 1.67x amplification
        dac_voltage = voltage/1.67
        self.set_dac_voltage(DAC_channel_lookup[dac_number], dac_voltage)

    def read_output_current(self, adc_number):
        # assumes 100 mohm current shunt and 100x amplification
        adc_voltage = self.read_adc_voltage(ADC_channel_lookup[adc_number])
        adc_current = adc_voltage / 0.1 / 100
        return adc_current

    def adc_sample_update(self):
            # Trigger ADC conversion (ICONV)
            self.write_register(0xC0, 0x01)
            # Wait for conversion to complete (you might need to adjust this delay)
            time.sleep_ms(1)
            # copy ADC data to registers
            adc_copy = 0b00010000
            self.write_register(0x0F, adc_copy)
    
    def read_adc(self, adc_number):
        if 0 <= adc_number <= 20:
            
            self.adc_sample_update()
        
            # Read ADC result
            adc_address_low_byte = 0x20 + (adc_number * 2)  # Even address for low byte
            adc_address_high_byte = adc_address_low_byte + 1
            low_byte = self.read_register(adc_address_low_byte)
            high_byte = self.read_register(adc_address_high_byte)
            return (high_byte << 8) | low_byte
        else:
            print("Invalid ADC number")
            return None
        
    def read_adc_voltage(self, adc_number):
        # fails for single-ended ADCs rn
        
        adc_val = self.read_adc(adc_number)
        voltage = 5*(adc_val*5/4096 - 2.5)
        return voltage
        

    def read_identifier(self):
        id_b1 = self.read_register(0x03)
        id_b2 = self.read_register(0x04)
        id_b3 = self.read_register(0x05)
        id_b4 = self.read_register(0x0C)
        id_b5 = self.read_register(0x0D)
        
        print("ID Bytes")
        print(hex(id_b1))
        print(hex(id_b2))
        print(hex(id_b3))
        print(hex(id_b4))
        print(hex(id_b5))
        

    def read_temperature(self):
        
        self.adc_sample_update()
        
        temp_high_byte = self.read_register(0x4B)  # Address for high byte is 0x4B
        temp_low_byte = self.read_register(0x4A)   # Address for low byte is 0x4A
    
        # Combine the two bytes into a 12-bit value (two's complement format)
        temp_raw = (temp_high_byte << 8) | (temp_low_byte)
        
        # Convert the raw value to temperature in Celsius
        if temp_raw & 0b0000100000000000:  # if the sign bit is set, extend the sign
            temp_raw -= 1 << 12
            
        # Each bit represents 0.25 degrees Celsius
        temperature_celsius = temp_raw * 0.25
        
        return temperature_celsius     
    
        
    def sweep_dac_sine_wave(self, dac, amplitude, steps, delay):
        for i in range(steps):
            angle = 2 * math.pi * (i / steps)
            sine_value = amplitude * (1+math.sin(angle))/2
            dac_value = int((sine_value) * 4095)
            self.set_dac(dac, dac_value)
            time.sleep(delay)
            
    def turn_off_all_DACs(self):
        for channel in range(16):
            self.set_output_voltage(channel, 0)

spi = SPI(0, baudrate=100000, polarity=0, phase=0, bits=8, firstbit=SPI.MSB, sck=Pin(2), mosi=Pin(3), miso=Pin(4))
cs = Pin(1, Pin.OUT)
reset = Pin(0, Pin.OUT)
alarmin = Pin(5, Pin.OUT)

amc = AMC7836(spi, cs, reset, alarmin)

# setting up serial object

poll_obj = select.poll()
poll_obj.register(sys.stdin, 1)

# Set all DACs to zero
for channel in range(16):
    amc.set_output_voltage(channel, 0)
    
led_config.value(1)

while True:
    # first, check DAC status and turn on output LED if any DACs enabled
    if sum(DAC_status) > 0: led_output_on.value(1)
    else: led_output_on.value(0)
    
    # second, check output off and dac reset switches
    if button_all_off.value(): amc.turn_off_all_DACs()
    if button_dac_rst.value(): amc.setup_AMC7836()
    if button_fun.value():
        led_config.value(0)
        time.sleep(0.5)
        led_config.value(1)
    
    # the, implement the USB serial commands. We want:
    #     *IDN?                     | prints "16CHSRC_V1.0_0125\r\n" over USB serial
    #     SETV <channel> <voltage>  | calls  amc.set_output_voltage(<channel>, <voltage>), prints "\r\n" over USB serial
    #     READI <channel>           | prints amc.read_output_current(<channel>)+"\r\n" over USB serial
    #     READT <channel>           | prints amc.read_temperature()+"\r\n" over USB serial
    
    if poll_obj.poll(0):
        led_sercomm.value(1)
        input_data = sys.stdin.readline().strip()
    
        if input_data == "*IDN?":
                print("16CHSRC_V1.0_0125")
        elif input_data.startswith("SETV"):
            try:
                _, channel, voltage = input_data.split()
                channel = int(channel)
                voltage = float(voltage)
                amc.set_output_voltage(channel, voltage)
                print()
            except Exception as e:
                print(f"Error: {e}")
        elif input_data.startswith("READI"):
            try:
                _, channel = input_data.split()
                channel = int(channel)
                current = amc.read_output_current(channel)
                print(f"{current}\r\n")
            except Exception as e:
                print(f"Error: {e}")
        elif input_data.startswith("READT"):
            try:
                temperature = amc.read_temperature()
                print(f"{temperature}\r\n")
            except Exception as e:
                print(f"Error: {e}")
        led_sercomm.value(0)
        
    time.sleep(0.01)

    continue

