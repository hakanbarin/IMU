import serial
import time

ser = serial.Serial('COM3', 115200)  # COM portunu kendi cihazına göre değiştir
f = open("mag_out.txt", "w")

print("Veri toplanıyor. Cihazı her yönde çevir!")

start_time = time.time()
duration = 30  # saniye (1 dakika boyunca veri topla)

while time.time() - start_time < duration:
    line = ser.readline().decode('utf-8').strip()
    print(line)
    f.write(line + "\n")

f.close()
ser.close()

print("Kayıt tamamlandı → mag_out.txt")
