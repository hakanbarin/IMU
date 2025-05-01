#!/usr/bin/env python3
"""
Bu Python kodu, Windows işletim sisteminde COM3 portu üzerinden çalışacak şekilde tasarlanmıştır.
Terminalden girilen komutları ayrıştırarak mikrodenetleyiciye RPC mesajları olarak iletecektir.
C tarafında tanımlanan rpc.h ve rpc.c dosyalarındaki servisler aşağıdaki mesaj yapısına göre gönderilir:

Mesaj Yapısı:
- Toplam 20 bayt:
  • 4 bayt: Servis tipi (unsigned int)
  • 16 bayt: Veri (her komutun parametreleri, gerektiğinde padding eklenir)

Örnek Komut Formatları:
  - "pid 1.0,0.1,0.05"
  - "yaw 45"
  - "depth 120"
  - "complementary 0.98,0.95,0.97,0.93"
  - "motors 1000,1000,1000,1000,1000,1000,1000,1000"
  - "arm 1" veya "arm 0"
"""

import serial
import struct

# RPC servis tanımlamaları (rpc.h'deki enum değerleri)
RPC_CALIBRATE_PID                   = 0
RPC_SET_DEGREES_OF_YAW              = 1
RPC_SET_DEPTH_CM                    = 2
RPC_COEFFICIENT_COMPLEMENTARY_FILTER = 3
RPC_PWM_MOTORS_FOR_STOP             = 4
RPC_FOR_ARM                         = 5
led_control

def send_calibrate_pid(ser, kp, ki, kd):
    """
    CALIBRATE_PID komutu için mesaj hazırlanıyor:
      - calibrate_pid_t: 3 adet float (12 bayt)
      - 4 bayt padding ekleniyor: toplam 16 bayt veri alanı
    """
    service = RPC_CALIBRATE_PID
    data = struct.pack("<fff", kp, ki, kd) + b'\x00'*4
    msg = struct.pack("<I", service) + data
    ser.write(msg)
    print("Calibrate PID gönderildi: kp = {}, ki = {}, kd = {}".format(kp, ki, kd))

def send_set_degrees_of_yaw(ser, degrees):
    """
    SET_DEGREES_OF_YAW komutu:
      - set_degrees_of_yaw_t: 1 float (4 bayt)
      - 12 bayt padding: toplam 16 bayt veri
    """
    service = RPC_SET_DEGREES_OF_YAW
    data = struct.pack("<f", degrees) + b'\x00'*12
    msg = struct.pack("<I", service) + data
    ser.write(msg)
    print("Yaw ayarlandı: {} derece".format(degrees))

def send_set_depth_cm(ser, depth_cm):
    """
    SET_DEPTH_CM komutu:
      - set_depth_cm_t: 1 float (4 bayt)
      - 12 bayt padding: toplam 16 bayt veri
    """
    service = RPC_SET_DEPTH_CM
    data = struct.pack("<f", depth_cm) + b'\x00'*12
    msg = struct.pack("<I", service) + data
    ser.write(msg)
    print("Derinlik ayarlandı: {} cm".format(depth_cm))

def send_coefficient_complementary_filter(ser, alpha_pitch, alpha_yaw, alpha_roll, alpha_stabilize):
    """
    COEFFICIENT_COMPLEMENTARY_FILTER komutu:
      - coefficient_complemantary_t: 4 adet float (16 bayt)
    """
    service = RPC_COEFFICIENT_COMPLEMENTARY_FILTER
    data = struct.pack("<ffff", alpha_pitch, alpha_yaw, alpha_roll, alpha_stabilize)
    msg = struct.pack("<I", service) + data
    ser.write(msg)
    print("Filtre katsayıları gönderildi: alpha_pitch = {}, alpha_yaw = {}, alpha_roll = {}, alpha_stabilize = {}"
          .format(alpha_pitch, alpha_yaw, alpha_roll, alpha_stabilize))

def send_pwm_motors_for_stop(ser, pwm_values):
    """
    PWM_MOTORS_FOR_STOP komutu:
      - pwm_motors_for_stop_t: 8 adet uint16 (16 bayt)
    """
    service = RPC_PWM_MOTORS_FOR_STOP
    if len(pwm_values) != 8:
        print("Hata: 8 adet PWM değeri gerekmektedir.")
        return
    data = struct.pack("<8H", *pwm_values)
    msg = struct.pack("<I", service) + data
    ser.write(msg)
    print("PWM motor komutu gönderildi:", pwm_values)

def send_for_arm(ser, arm_flag):
    """
    FOR_ARM komutu:
      - for_arm_t: 1 adet uint8 (1 bayt)
      - 15 bayt padding: toplam 16 bayt veri
    arm_flag: 1 (arm) veya 0 (disarm)
    """
    service = RPC_FOR_ARM
    data = struct.pack("<B", arm_flag) + b'\x00'*15
    msg = struct.pack("<I", service) + data
    ser.write(msg)
    print("Arm/Disarm komutu gönderildi. Değer:", arm_flag)

def process_command(command, ser):
    """
    Terminal üzerinden girilen komutları ayrıştırır ve ilgili RPC fonksiyonunu çağırır.
    """
    try:
        cmd = command.strip().lower()
        if cmd.startswith("pid"):
            # Örnek: "pid 1.0,0.1,0.05"
            parts = cmd[3:].split(',')
            if len(parts) >= 3:
                kp = float(parts[0].strip())
                ki = float(parts[1].strip())
                kd = float(parts[2].strip())
                send_calibrate_pid(ser, kp, ki, kd)
            else:
                print("PID komutu 'pid kp,ki,kd' formatında girilmelidir.")
        elif cmd.startswith("yaw"):
            # Örnek: "yaw 45"
            parts = cmd.split()
            if len(parts) >= 2:
                degrees = float(parts[1].strip())
                send_set_degrees_of_yaw(ser, degrees)
            else:
                print("Yaw komutu 'yaw derece' formatında girilmelidir.")
        elif cmd.startswith("depth"):
            # Örnek: "depth 120"
            parts = cmd.split()
            if len(parts) >= 2:
                depth_cm = float(parts[1].strip())
                send_set_depth_cm(ser, depth_cm)
            else:
                print("Depth komutu 'depth <deger>' formatında girilmelidir.")
        elif cmd.startswith("complementary"):
            # Örnek: "complementary 0.98,0.95,0.97,0.93"
            parts = cmd[len("complementary"):].split(',')
            if len(parts) >= 4:
                alpha_pitch   = float(parts[0].strip())
                alpha_yaw     = float(parts[1].strip())
                alpha_roll    = float(parts[2].strip())
                alpha_stabilize = float(parts[3].strip())
                send_coefficient_complementary_filter(ser, alpha_pitch, alpha_yaw, alpha_roll, alpha_stabilize)
            else:
                print("Complementary komutu 'complementary a_pitch,a_yaw,a_roll,a_stabilize' formatında girilmelidir.")
        elif cmd.startswith("motors"):
            # Örnek: "motors 1000,1000,1000,1000,1000,1000,1000,1000"
            parts = cmd[len("motors"):].split(',')
            if len(parts) >= 8:
                pwm_values = [int(p.strip()) for p in parts[:8]]
                send_pwm_motors_for_stop(ser, pwm_values)
            else:
                print("Motors komutu 8 adet PWM değeri olacak şekilde girilmelidir.")
        elif cmd.startswith("arm"):
            # Örnek: "arm 2" "arm 1" veya "arm 0"
            parts = cmd.split()
            if len(parts) >= 2:
                arm_flag = int(parts[1].strip())
                send_for_arm(ser, arm_flag)
            else:
                print("Arm komutu 'arm 1' (arm) veya 'arm 0' (disarm) formatında girilmelidir.")
        else:
            print("Bilinmeyen komut. Kullanılabilir komutlar: pid, yaw, depth, complementary, motors, arm")
    except Exception as e:
        print("Komut ayrıştırılırken hata oluştu:", e)

def main():
    """
    Ana döngü:
      - COM3 portunu kullanarak seri portu açar.
      - Terminalden girilen komutları bekler ve her komut için ilgili işlemi gerçekleştirir.
    """
    try:
        # Windows üzerinde COM3 portunu kullanın
        ser = serial.Serial('COM5', baudrate=115200, timeout=1)
        print("Seri port (COM5) başarıyla açıldı.")
    except Exception as e:
        print("Seri port açılamadı:", e)
        return

    print("Komut bekleniyor... (Çıkmak için 'exit' veya 'quit' yazın)")
    while True:
        try:
            command = input("Komut: ")
            if command.lower() in ["exit", "quit"]:
                break
            process_command(command, ser)
        except KeyboardInterrupt:
            print("Çıkılıyor.")
            break

    ser.close()

if __name__ == "__main__":
    main()
