# 💡 Sistem Monitoring LPJU Menggunakan ESP-NOW Protocol Berbasis IoT

Repository ini berisi kode sumber (*source code*) untuk penelitian skripsi berjudul:
**"Sistem Monitoring Lampu Penerangan Jalan Umum Menggunakan ESP-NOW Protocol Berbasis Internet of Things"**

## 📌 Fitur Utama
- **Jaringan Lokal ESP-NOW**: Komunikasi nirkabel *peer-to-peer* dua arah antara Node Master dan Node Slave tanpa router/kuota.
- **Hardware Load Balancing**: Pemisahan tugas antara ESP8266 (Master Komunikasi Radio) dan ESP32 (Gateway Telemetri Internet).
- **Notifikasi Otomatis WhatsApp Bot**: Mengirim alarm instan saat terjadi kerusakan bohlam fisik di lapangan.
- **Monitoring Parameter Listrik**: Pengukuran Tegangan, Arus, Daya, dan Energi via PZEM-004T.
- **Sistem Geolokasi & Waktu**: Integrasi GPS Neo-6M dan RTC DS3231.

## 🛠️ Komponen & Teknologi
- **Mikrokontroler**: ESP8266, ESP32
- **Sensor & Modul**: Sensor LDR, PZEM-004T, GPS Neo-6M, RTC DS3231, LCD 16x2
- **Software**: Arduino IDE, Node.js, Docker, Portainer, WhatsApp API

## 📂 Struktur Folder
- `/node_slave` : Code C++ Arduino untuk mikrokontroler di tiang lampu (ESP8266 + LDR + Relay)
- `/node_master` : Code C++ Arduino untuk mikrokontroler pusat komunikasi lokal (ESP8266 + ESP-NOW)
- `/node_gateway` : Code C++ Arduino untuk gateway pusat ke server internet (ESP32 + GPS + PZEM + RTC)

---
**Peneliti**: Muhammad Miftahul Ulum  
**Program Studi**: Teknik Informatika - UNISNU Jepara (2026)
