# 🏎️ IoT-Finpro: Hybrid RC Car Control System

![Status](https://img.shields.io/badge/Status-Active-success)
![Platform](https://img.shields.io/badge/Platform-ESP32-blue)
![IoT](https://img.shields.io/badge/Protocol-MQTT%20%26%20Bluetooth-orange)
![Dashboard](https://img.shields.io/badge/Web-Vercel-black)

**IoT-Finpro** adalah proyek Tugas Akhir yang menggabungkan kontrol mobil RC konvensional dengan teknologi IoT modern. Kami merancang sistem *hybrid* di mana mobil dapat dikendalikan melalui **Bluetooth (jarak dekat/low latency)** untuk manuver cepat, sekaligus dipantau dan dikendalikan jarak jauh melalui **Web Dashboard (MQTT/WiFi)**.

Sistem ini diotaki oleh **ESP32** yang menjalankan **FreeRTOS** untuk menjamin kestabilan multitasking antara mengurus motor, membaca sensor, dan menjaga koneksi internet tetap hidup secara bersamaan.

---

## 🧠 Arsitektur Sistem (Cara Kerja)

Sistem ini tidak hanya sekadar "remote control", tapi dibagi menjadi dua jalur komunikasi yang berjalan paralel:

### 1. Jalur Kontrol Lokal (Bluetooth Classic)
* **Tools:** Aplikasi Dabble (Android).
* **Fungsi:** Digunakan untuk mengemudi di lapangan (Field Operation).
* **Kenapa?** Bluetooth memiliki *latency* (jeda) yang sangat kecil, sehingga respon gas dan setir sangat instan. Cocok untuk manuver presisi.

### 2. Jalur IoT & Telemetri (MQTT over WiFi)
* **Tools:** Web Dashboard (di-hosting di Vercel).
* **Fungsi:** Monitoring kondisi baterai, kecepatan, dan *Alternative Control* (kendali cadangan).
* **Cara Kerja:**
    * **ESP32 (Publisher):** Secara rutin (tiap 500ms) melapor ke server: "Baterai tinggal sekian Volt", "Lagi ngebut sekian PWM", "Posisi ban lurus/belok".
    * **Web (Subscriber):** Menangkap laporan itu dan menampilkannya dalam bentuk *Gauge* dan *Slider* yang bergerak sendiri (Real-time).
    * **Web (Publisher):** Jika slider di web digeser, web akan kirim perintah ke ESP32 via internet.

### 📊 Diagram Alur Data

```mermaid

graph TD
    %% Node Definitions
    User_HP["📱 HP User (Dabble)"]
    User_Web["💻 Laptop/Web Dashboard"]
    Broker(("☁️ HiveMQ Broker"))
    
    subgraph "UNIT MOBIL (ESP32)"
        ESP32[Microcontroller ESP32]
        TaskBT[Task: Input Bluetooth]
        TaskMotor[Task: Actuator Driver]
        TaskIoT[Task: Telemetry MQTT]
    end

    %% Flow Connections
    User_HP -- "Bluetooth Serial" --> TaskBT
    TaskBT -- "Queue (Data Kontrol)" --> TaskMotor
    
    User_Web -- "WebSocket Secure (WSS)" <--> Broker
    Broker -- "MQTT TCP (Port 1883)" <--> TaskIoT
    
    TaskIoT -- "Mutex (Data Telemetri)" --> TaskMotor
    TaskMotor -- "PWM & Digital Write" --> Driver["Driver L298N & Servo"]
